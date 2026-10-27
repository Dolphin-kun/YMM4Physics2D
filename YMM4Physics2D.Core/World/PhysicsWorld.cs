using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Colliders;
using YMM4Physics2D.Core.Collision;
using YMM4Physics2D.Core.Geometry;
using YMM4Physics2D.Core.Shapes;

namespace YMM4Physics2D.Core.World
{
    public sealed partial class PhysicsWorld
    {
        private const int NoFrame = int.MinValue;

        private const int RemovalEvidenceCount = 2;

        private const int ResolveIterationsPerSetting = 3;

        private const int MaxSubsteps = 8;
        private const int MaxFluidSubsteps = 3;
        private static readonly long RemovalEvidenceInterval = Stopwatch.Frequency / 4;

        private const int UpdateWaitMilliseconds = 10;
        private const int MaxUpdateWaits = 4;

        private readonly object syncRoot = new();
        private readonly ConcurrentQueue<(BodyTrack Track, object? Writer, bool Remove)> pendingDetaches = new();
        private int pendingUpdates;
        private long idleSince;
        private bool isRetired;

        private readonly Dictionary<TrackKey, BodyTrack> trackMap = [];
        private readonly List<BodyTrack> tracks = [];
        private readonly List<RigidBody> wallBodies = [];
        private readonly List<RigidBody> activeBodies = [];
        private readonly List<Manifold> manifolds = [];
        private readonly List<(Vector2 A, Vector2 B)> contactOrigins = [];
        private readonly List<(BodyTrack Owner, Vector2 Center, float Strength, float Range)> attractors = [];
        private readonly SortedList<int, WorldSnapshot> snapshots = [];

        private int[] sweepOrder = [];
        private float[] sweepKeys = [];

        private WorldParameters parameters;
        private bool hasParameters;
        private long nextSequence;
        private bool orderDirty;
        private bool bodiesDirty;

        private int liveFrame = NoFrame;
        private int validUntil = NoFrame;
        private WorldSnapshot? previousState;
        private WorldSnapshot? spareState;
        private readonly List<BodyTrack> staleSnapshotTracks = [];
        private object? currentParameters;
        private long pulledSettingsVersion = -1;

        public void SetParameters(in WorldParameters newParameters)
        {
            if (Volatile.Read(ref currentParameters) is WorldParameters current && current == newParameters) return;

            lock (syncRoot)
            {
                if (hasParameters && parameters == newParameters) return;

                bool wallsChanged = !hasParameters || parameters.EnableWalls != newParameters.EnableWalls || parameters.ScreenSize != newParameters.ScreenSize;
                parameters = newParameters;
                hasParameters = true;
                Volatile.Write(ref currentParameters, newParameters);

                if (wallsChanged) RebuildWalls();

                foreach (var track in tracks) track.ClearParameterCache();
                Invalidate(0);
            }
        }

        internal BodyTrack Attach(TrackKey key, object? writer = null)
        {
            lock (syncRoot)
            {
                DrainDetaches();
                var track = GetOrCreateTrack(key);
                track.Writers.Add(writer ?? key.Owner);
                track.RefreshPrimaryWriter();
                track.SuspectCount = 0;
                idleSince = 0;
                return track;
            }
        }

        internal BodyTrack? TryAttachObserver(TrackKey key, object observer)
        {
            if (!Monitor.TryEnter(syncRoot)) return null;
            try
            {
                DrainDetaches();
                var track = GetOrCreateTrack(key);
                track.Observers.Add(observer);
                track.SuspectCount = 0;
                idleSince = 0;
                return track;
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }

        private BodyTrack GetOrCreateTrack(TrackKey key)
        {
            if (!trackMap.TryGetValue(key, out var track))
            {
                track = new BodyTrack(key, nextSequence++);
                trackMap[key] = track;
                tracks.Add(track);
                orderDirty = true;
            }
            return track;
        }

        public void Detach(BodyTrack track, object? writer, bool remove)
        {
            if (!Monitor.TryEnter(syncRoot))
            {
                pendingDetaches.Enqueue((track, writer, remove));
                return;
            }
            try
            {
                DrainDetaches();
                DetachCore(track, writer, remove);
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }

        private void DrainDetaches()
        {
            while (pendingDetaches.TryDequeue(out var pending))
            {
                DetachCore(pending.Track, pending.Writer, pending.Remove);
            }
        }

        private void DetachCore(BodyTrack track, object? writer, bool remove)
        {
            if (track.IsRemoved) return;

            object target = writer ?? track.Key.Owner;
            int index = track.Writers.FindIndex(w => ReferenceEquals(w, target));
            if (index >= 0) track.Writers.RemoveAt(index);
            else track.Observers.Remove(target);
            track.RefreshPrimaryWriter();
            if (remove && track.AttachCount == 0)
            {
                RemoveTrack(track);
            }
        }

        internal bool TryRetireIfIdle(long now, long lifetime)
        {
            if (!Monitor.TryEnter(syncRoot)) return false;
            try
            {
                DrainDetaches();
                if (Volatile.Read(ref pendingUpdates) > 0) return false;

                foreach (var track in tracks)
                {
                    if (track.AttachCount > 0)
                    {
                        idleSince = 0;
                        return false;
                    }
                }

                if (idleSince == 0)
                {
                    idleSince = now;
                    return false;
                }
                if (now - idleSince < lifetime) return false;

                foreach (var track in tracks)
                {
                    track.IsRemoved = true;
                    track.Bodies = [];
                    track.Results.Clear();
                }
                tracks.Clear();
                trackMap.Clear();
                snapshots.Clear();
                previousState = null;
                spareState = null;
                activeBodies.Clear();
                activeJoints.Clear();
                activeAreas.Clear();
                attractors.Clear();
                manifolds.Clear();
                fluidBodies.Clear();
                fluidOwners.Clear();
                Volatile.Write(ref bodyNames, []);
                Volatile.Write(ref triggerNames, []);
                liveFrame = NoFrame;
                validUntil = NoFrame;
                isRetired = true;
                return true;
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }

        public ShapeVersion? FindShape(BodyTrack track, ulong hash)
        {
            lock (syncRoot)
            {
                return track.FindShape(hash);
            }
        }

        public bool HasShape(BodyTrack track)
        {
            lock (syncRoot)
            {
                return track.HasShape;
            }
        }

        public void UpdateTrack(BodyTrack track, TrackUpdate update, object? writer = null)
        {
            Interlocked.Increment(ref pendingUpdates);
            lock (syncRoot)
            {
                try
                {
                    DrainDetaches();
                    if (isRetired) return;
                    if (track.IsInitialized && !track.IsPrimaryWriter(writer))
                    {
                        UpdateTrackSecondary(track, update);
                    }
                    else
                    {
                        UpdateTrackCore(track, update);
                    }
                }
                finally
                {
                    if (Interlocked.Decrement(ref pendingUpdates) == 0) Monitor.PulseAll(syncRoot);
                }
            }
        }

        private void UpdateTrackSecondary(BodyTrack track, TrackUpdate update)
        {
            if (track.IsRemoved || track.HasObservation(update.LocalFrame)) return;

            int? observationChange = track.RecordObservation(update.LocalFrame, update.Observation);
            if (observationChange.HasValue) Invalidate(track.StartFrame + observationChange.Value);

            if (update.Shape != null && !track.HasShape)
            {
                int? shapeChange = track.RecordShape(update.LocalFrame, update.Shape, update.ShapeReplaced);
                if (shapeChange.HasValue) Invalidate(track.StartFrame + shapeChange.Value);
            }
        }

        private void UpdateTrackCore(BodyTrack track, TrackUpdate update)
        {
            if (track.IsRemoved) return;

            int endFrame = update.StartFrame + Math.Max(1, update.Length);

            if (!track.IsInitialized)
            {
                track.IsInitialized = true;
                track.StartFrame = update.StartFrame;
                track.EndFrame = endFrame;
                track.Layer = update.Layer;
                track.BodyType = update.BodyType;
                track.IsSeparate = update.IsSeparate;
                track.CollisionMargin = update.CollisionMargin;
                track.FluidResolution = update.FluidResolution;
                track.ShapeUpdateMode = update.ShapeUpdateMode;
                orderDirty = true;
                Invalidate(update.StartFrame);
            }
            else
            {
                int invalidateFrom = int.MaxValue;

                if (track.StartFrame != update.StartFrame || track.Layer != update.Layer)
                {
                    invalidateFrom = Math.Min(track.StartFrame, update.StartFrame);
                    orderDirty = true;
                }
                if (track.EndFrame != endFrame)
                {
                    invalidateFrom = Math.Min(invalidateFrom, Math.Min(track.EndFrame, endFrame));
                }
                if (track.BodyType != update.BodyType || track.IsSeparate != update.IsSeparate || track.CollisionMargin != update.CollisionMargin || track.FluidResolution != update.FluidResolution)
                {
                    invalidateFrom = Math.Min(invalidateFrom, Math.Min(track.StartFrame, update.StartFrame));
                }
                if (track.ShapeUpdateMode != update.ShapeUpdateMode)
                {
                    track.ClearShapes();
                    invalidateFrom = Math.Min(invalidateFrom, Math.Min(track.StartFrame, update.StartFrame));
                }

                track.StartFrame = update.StartFrame;
                track.EndFrame = endFrame;
                track.Layer = update.Layer;
                track.BodyType = update.BodyType;
                track.IsSeparate = update.IsSeparate;
                track.CollisionMargin = update.CollisionMargin;
                track.FluidResolution = update.FluidResolution;
                track.ShapeUpdateMode = update.ShapeUpdateMode;

                if (invalidateFrom != int.MaxValue)
                {
                    track.ClearParameterCache();
                    Invalidate(invalidateFrom);
                }
            }

            track.ParameterProvider = update.ParameterProvider;
            track.SettingsProvider = update.SettingsProvider;
            track.LastLocal = update.LocalFrame;

            if (track.Area != update.Area || track.AreaBounds != update.AreaBounds)
            {
                bool namesChanged = track.Area?.Kind != update.Area?.Kind;
                track.Area = update.Area;
                track.AreaBounds = update.AreaBounds;
                if (namesChanged) RefreshNames();
                Invalidate(track.StartFrame);
            }
            track.AreaParameterProvider = update.AreaParameterProvider;
            if (track.IsArea && track.CheckAreaParameters(update.LocalFrame, update.AreaParameters))
            {
                Invalidate(track.StartFrame);
            }

            if (track.Name != update.Name)
            {
                track.Name = update.Name;
                RefreshNames();
                Invalidate(track.StartFrame);
            }

            track.UpdateSerial++;
            if (track.PruneJoints()) Invalidate(track.StartFrame);

            if (track.CheckParameters(update.LocalFrame, update.Parameters))
            {
                Invalidate(track.StartFrame);
            }

            int? observationChange = track.RecordObservation(update.LocalFrame, update.Observation);
            if (observationChange.HasValue) Invalidate(track.StartFrame + observationChange.Value);

            if (update.Shape != null)
            {
                int? shapeChange = track.RecordShape(update.LocalFrame, update.Shape, update.ShapeReplaced);
                if (shapeChange.HasValue) Invalidate(track.StartFrame + shapeChange.Value);
            }

            CollectRemovalEvidence(track, update.StartFrame + update.LocalFrame);
        }

        public bool IsSecondary(BodyTrack track, object? writer)
        {
            return track.IsInitialized && !track.IsPrimaryWriter(writer);
        }

        public void TryUpdateSecondary(BodyTrack track, TrackUpdate update)
        {
            if (!Monitor.TryEnter(syncRoot)) return;
            try
            {
                if (track.IsInitialized) UpdateTrackSecondary(track, update);
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }

        public bool TryGetLatestResult(BodyTrack track, int frame, TrackFrameResult result, int maxSteps)
        {
            if (!Monitor.TryEnter(syncRoot)) return false;
            try
            {
                DrainDetaches();
                if (track.IsRemoved || !track.IsInitialized || !hasParameters) return false;
                if (frame < track.StartFrame || frame >= track.EndFrame) return false;

                int local = frame - track.StartFrame;
                bool valid = frame <= validUntil && local < track.Results.Count && track.Results[local].IsWritten;
                if (!valid && validUntil != NoFrame && frame > validUntil && frame - validUntil <= maxSteps)
                {
                    EnsureSimulated(frame);
                }

                for (int i = Math.Min(local, track.Results.Count - 1); i >= 0; i--)
                {
                    if (track.Results[i].IsWritten) return CopyRecord(track.Results[i], result);
                }
                return false;
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }

        public bool NeedsSimulation(BodyTrack track, object? writer, int frame)
        {
            lock (syncRoot)
            {
                if (track.IsRemoved || !track.IsInitialized || !track.IsPrimaryWriter(writer)) return false;
                if (frame < track.StartFrame || frame >= track.EndFrame) return false;
                PullSettings();

                int local = frame - track.StartFrame;
                return frame > validUntil || local >= track.Results.Count || !track.Results[local].IsWritten;
            }
        }

        public bool TryGetResult(BodyTrack track, int frame, TrackFrameResult result)
        {
            lock (syncRoot)
            {
                return TryEnsureRecord(track, frame, out int local) && CopyRecord(track.Results[local], result);
            }
        }

        private bool TryEnsureRecord(BodyTrack track, int frame, out int local)
        {
            PullSettings();
            int waits = 0;
            while (true)
            {
                local = frame - track.StartFrame;
                if (track.IsRemoved || !track.IsInitialized || !hasParameters) return false;
                if (frame < track.StartFrame || frame >= track.EndFrame) return false;

                if (frame <= validUntil && local < track.Results.Count && track.Results[local].IsWritten) return true;

                if (frame <= validUntil) Invalidate(frame);
                if (EnsureSimulated(frame, waits < MaxUpdateWaits)) return local < track.Results.Count;

                WaitForPendingUpdates();
                waits++;
            }
        }

        private void PullSettings()
        {
            DrainDetaches();
            long version = PhysicsManager.SettingsVersion;
            if (version == pulledSettingsVersion) return;
            pulledSettingsVersion = version;

            bool namesChanged = false;

            int fps = hasParameters ? parameters.Fps : 30;
            foreach (var track in tracks)
            {
                if (!track.IsInitialized || track.IsRemoved) continue;

                bool changed = false;
                if (track.ConsumeParameterRefresh())
                {
                    track.ClearParameterCache();
                    changed = true;
                }
                if (track.SettingsProvider is { } settingsProvider)
                {
                    var settings = settingsProvider();
                    if (track.BodyType != settings.BodyType || track.IsSeparate != settings.IsSeparate || track.CollisionMargin != settings.CollisionMargin || track.FluidResolution != settings.FluidResolution)
                    {
                        track.BodyType = settings.BodyType;
                        track.IsSeparate = settings.IsSeparate;
                        track.CollisionMargin = settings.CollisionMargin;
                        track.FluidResolution = settings.FluidResolution;
                        track.ClearParameterCache();
                        changed = true;
                    }
                    if (track.Name != settings.Name)
                    {
                        track.Name = settings.Name;
                        namesChanged = true;
                        changed = true;
                    }
                    if (track.Area != settings.Area)
                    {
                        namesChanged |= track.Area?.Kind != settings.Area?.Kind;
                        track.Area = settings.Area;
                        changed = true;
                    }
                }

                if (track.ParameterProvider is { } parameterProvider &&
                    track.CheckParameters(track.LastLocal, parameterProvider(track.LastLocal, track.Length, fps)))
                {
                    changed = true;
                }
                if (track.IsArea && track.AreaParameterProvider is { } areaProvider &&
                    track.CheckAreaParameters(track.LastLocal, areaProvider(track.LastLocal, track.Length, fps)))
                {
                    changed = true;
                }

                foreach (var joint in track.Joints)
                {
                    if (joint.Provider is not { } jointProvider) continue;
                    var definition = jointProvider();
                    if (definition == joint.Definition) continue;
                    joint.Definition = definition;
                    changed = true;
                }

                if (changed) Invalidate(track.StartFrame);
            }

            if (namesChanged) RefreshNames();
        }

        private static bool CopyRecord(in BodyTrack.FrameRecord record, TrackFrameResult result)
        {
            if (!record.IsWritten || record.Shape == null || record.Poses.Length == 0) return false;

            result.Shape = record.Shape;
            result.ImageTransform = record.Transform;
            result.IsSeparate = record.IsSeparate;
            result.Poses.Clear();
            result.Poses.AddRange(record.Poses);
            result.LocalCentroids.Clear();
            result.LocalCentroids.AddRange(record.Centroids);
            result.FluidShape = record.FluidShape;
            result.FluidPositions.Clear();
            if (record.FluidPositions != null) result.FluidPositions.AddRange(record.FluidPositions);
            result.FluidVelocities.Clear();
            if (record.FluidVelocities != null) result.FluidVelocities.AddRange(record.FluidVelocities);
            result.FluidPresences.Clear();
            if (record.FluidPresences != null) result.FluidPresences.AddRange(record.FluidPresences);
            result.JointLines.Clear();
            if (record.JointFrames != null)
            {
                foreach (var joint in record.JointFrames)
                {
                    if (joint.LineWidth > 0f) result.JointLines.Add(new JointLine(joint.PointA, joint.PointB, joint.LineWidth, joint.LineColor));
                }
            }
            return true;
        }

        #region 削除判定

        private void CollectRemovalEvidence(BodyTrack updatedTrack, int frame)
        {
            long now = Stopwatch.GetTimestamp();

            for (int i = tracks.Count - 1; i >= 0; i--)
            {
                var track = tracks[i];
                if (track == updatedTrack || track.AttachCount > 0 || !track.IsInitialized) continue;
                if (frame < track.StartFrame || frame >= track.EndFrame) continue;

                if (track.SuspectCount == 0)
                {
                    track.SuspectCount = 1;
                    track.LastEvidenceFrame = frame;
                    track.LastEvidenceTime = now;
                    track.MinEvidenceFrame = frame;
                    continue;
                }

                if (frame == track.LastEvidenceFrame && now - track.LastEvidenceTime < RemovalEvidenceInterval) continue;

                track.SuspectCount++;
                track.LastEvidenceFrame = frame;
                track.LastEvidenceTime = now;
                track.MinEvidenceFrame = Math.Min(track.MinEvidenceFrame, frame);

                if (track.SuspectCount < RemovalEvidenceCount) continue;

                if (track.MinEvidenceFrame > track.StartFrame + track.MaxObservedLocal)
                {
                    int newEnd = track.MinEvidenceFrame;
                    if (newEnd < track.EndFrame)
                    {
                        Invalidate(newEnd);
                        track.EndFrame = newEnd;
                    }
                    track.SuspectCount = 0;
                }
                else
                {
                    RemoveTrack(track);
                }
            }
        }

        private void RemoveTrack(BodyTrack track)
        {
            if (track.IsRemoved) return;

            track.IsRemoved = true;
            trackMap.Remove(track.Key);
            tracks.Remove(track);

            if (track.IsInitialized) Invalidate(track.StartFrame);

            foreach (var snapshot in snapshots.Values) snapshot.Tracks.Remove(track);
            previousState?.Tracks.Remove(track);
            spareState?.Tracks.Remove(track);

            Despawn(track);
            track.Results.Clear();
            RefreshNames();
        }

        #endregion

        #region 計算

        private void Invalidate(int fromFrame)
        {
            fromFrame = Math.Max(0, fromFrame);

            if (validUntil != NoFrame && fromFrame - 1 < validUntil)
            {
                validUntil = fromFrame - 1;
            }

            while (snapshots.Count > 0 && snapshots.Keys[^1] >= fromFrame)
            {
                snapshots.RemoveAt(snapshots.Count - 1);
            }

            if (previousState != null && previousState.Frame >= fromFrame)
            {
                spareState = previousState;
                previousState = null;
            }
        }

        private void WaitForPendingUpdates()
        {
            while (Volatile.Read(ref pendingUpdates) > 0)
            {
                Monitor.Wait(syncRoot, UpdateWaitMilliseconds);
            }
        }

        private bool EnsureSimulated(int target, bool yieldToUpdates = true)
        {
            if (target <= validUntil) return true;
            DrainDetaches();

            int origin = int.MaxValue;
            foreach (var track in tracks)
            {
                if (track.IsInitialized) origin = Math.Min(origin, track.StartFrame - 1);
            }
            if (origin == int.MaxValue || target <= origin) return true;

            int limit = validUntil == NoFrame ? NoFrame : Math.Min(validUntil, target - 1);

            int restoreFrame = origin;
            WorldSnapshot? restoreState = null;
            bool useLive = false;

            if (limit != NoFrame)
            {
                if (liveFrame != NoFrame && liveFrame <= limit && liveFrame > restoreFrame)
                {
                    restoreFrame = liveFrame;
                    useLive = true;
                }

                if (previousState != null && previousState.Frame <= limit && previousState.Frame > restoreFrame)
                {
                    restoreFrame = previousState.Frame;
                    restoreState = previousState;
                    useLive = false;
                }

                for (int i = snapshots.Count - 1; i >= 0; i--)
                {
                    int key = snapshots.Keys[i];
                    if (key > limit) continue;
                    if (key > restoreFrame)
                    {
                        restoreFrame = key;
                        restoreState = snapshots.Values[i];
                        useLive = false;
                    }
                    break;
                }
            }

            if (!useLive)
            {
                if (restoreState != null) Restore(restoreState);
                else RestoreEmpty(origin);
            }

            SortTracksIfNeeded();

            try
            {
                for (int frame = restoreFrame + 1; frame <= target; frame++)
                {
                    if (frame == target)
                    {
                        var reuse = previousState ?? spareState;
                        spareState = null;
                        previousState = Capture(frame - 1, reuse);
                    }

                    Step(frame);
                    liveFrame = frame;

                    if (frame % parameters.SnapshotInterval == 0)
                    {
                        snapshots[frame] = Capture(frame);
                    }

                    if (yieldToUpdates && frame < target && Volatile.Read(ref pendingUpdates) > 0)
                    {
                        validUntil = frame;
                        return false;
                    }
                }
            }
            catch
            {
                liveFrame = NoFrame;
                spareState = null;
                previousState = null;
                throw;
            }

            validUntil = target;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void Step(int frame)
        {
            float deltaTime = 1f / parameters.Fps;
            activeAreas.Clear();
            simulationTime = (float)frame / parameters.Fps;

            foreach (var track in tracks)
            {
                if (!track.IsInitialized) continue;

                int local = frame - track.StartFrame;
                if (local < 0 || frame >= track.EndFrame)
                {
                    Despawn(track);
                    continue;
                }

                var observation = track.GetObservation(local);
                if (track.IsArea)
                {
                    if (observation != null) CollectArea(track, local, observation.Value);
                    continue;
                }

                var shape = track.GetShape(local);
                if (observation == null || shape == null || shape.IsEmpty)
                {
                    Despawn(track);
                    continue;
                }

                var imageTransform = observation.Value.ImageTransform;
                if (MathF.Abs(imageTransform.GetDeterminant()) < 1e-8f)
                {
                    Despawn(track);
                    continue;
                }

                var bodyParameters = track.GetParameters(local, parameters.Fps);

                if (!track.IsSpawned)
                {
                    Spawn(track, shape, imageTransform, observation.Value, bodyParameters, local);
                }
                else if (!ReferenceEquals(track.SpawnedShape, shape) || track.SpawnedTransform != imageTransform)
                {
                    Reshape(track, shape, imageTransform, bodyParameters);
                }

                track.CurrentParameters = bodyParameters;
                ApplyParameters(track, bodyParameters);

                if (track.BodyType == BodyType.Static)
                {
                    ApplyKinematicTarget(track, local, observation.Value, deltaTime);
                }
            }

            if (bodiesDirty) RebuildActiveBodies();

            PrepareJoints();

            int substeps = ComputeSubsteps(deltaTime);
            float subDelta = deltaTime / substeps;
            int resolveIterations = parameters.Iterations * ResolveIterationsPerSetting;

            for (int s = 0; s < substeps; s++)
            {
                GatherFluid();
                IntegrateBodies(subDelta);
                ApplySprings(subDelta);
                SolveFluids(subDelta);

                DetectContacts();
                for (int i = 0; i < resolveIterations; i++)
                {
                    ResolveContacts();
                    SolveJoints();
                }
                SolveJointPositions();
                simulationTime += subDelta;
            }

            foreach (var body in activeBodies) body.IsSpawning = false;

            if (activeAreas.Count > 0 || HasFadingBodies()) ProcessTriggers();

            RecordResults(frame);
        }

        private bool HasFadingBodies()
        {
            foreach (var body in activeBodies)
            {
                if (body.IsGhost && body.Opacity > 0f) return true;
            }
            return false;
        }

        private void Spawn(BodyTrack track, ShapeVersion shape, Matrix3x2 imageTransform, TrackObservation observation, BodyParameters bodyParameters, int local)
        {
            var bodies = track.IsFluid ? CreateFluidBodies(track, shape, imageTransform, bodyParameters.Mass)
                : BodyFactory.CreateBodies(shape, imageTransform, track.IsSeparate, track.BodyType, bodyParameters.Mass, track.CollisionMargin);
            foreach (var body in bodies)
            {
                body.Position = observation.Draw + body.LocalCentroid.Rotate(observation.Rotation);
                body.Rotation = body.IsParticle ? 0f : observation.Rotation;
                body.IsSpawning = true;
                body.UpdateWorldAABB();
            }

            track.Bodies = bodies;
            track.IsSpawned = true;
            track.SpawnedShape = shape;
            track.SpawnedTransform = imageTransform;
            track.SpawnLocals.Add(local);
            bodiesDirty = true;
        }

        private void Reshape(BodyTrack track, ShapeVersion shape, Matrix3x2 imageTransform, BodyParameters bodyParameters)
        {
            if (track.IsFluid)
            {
                ReshapeFluid(track, shape, imageTransform, bodyParameters);
                return;
            }

            var oldBodies = track.Bodies;
            var oldTransform = track.SpawnedTransform;
            var newBodies = BodyFactory.CreateBodies(shape, imageTransform, track.IsSeparate, track.BodyType, bodyParameters.Mass, track.CollisionMargin);

            foreach (var body in newBodies)
            {
                RigidBody? source = null;
                float bestDistance = float.MaxValue;
                foreach (var oldBody in oldBodies)
                {
                    float distance = Vector2.DistanceSquared(oldBody.ImageCentroid, body.ImageCentroid);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        source = oldBody;
                    }
                }

                if (source == null) continue;

                Vector2 anchor = Vector2.Transform(body.ImageCentroid, oldTransform) - source.LocalCentroid;
                Vector2 position = source.Position + anchor.Rotate(source.Rotation);

                body.Position = position;
                body.Rotation = source.Rotation;
                body.LinearVelocity = source.LinearVelocity + Vector2Extensions.Cross(source.AngularVelocity, position - source.Position);
                body.AngularVelocity = source.AngularVelocity;
                body.UpdateWorldAABB();
            }

            track.Bodies = newBodies;
            track.SpawnedShape = shape;
            track.SpawnedTransform = imageTransform;
            bodiesDirty = true;
        }

        private void Despawn(BodyTrack track)
        {
            if (!track.IsSpawned) return;

            track.Bodies = [];
            track.IsSpawned = false;
            track.SpawnedShape = null;
            track.FluidShape = null;
            bodiesDirty = true;
        }

        private static void ApplyParameters(BodyTrack track, BodyParameters bodyParameters)
        {
            float massPerBody = track.Bodies.Count > 0 ? bodyParameters.Mass / track.Bodies.Count : bodyParameters.Mass;
            foreach (var body in track.Bodies)
            {
                body.Type = body.IsFrozen || track.BodyType == BodyType.Static ? BodyType.Static : BodyType.Dynamic;
                body.Mass = massPerBody;
                body.Restitution = body.IsParticle ? bodyParameters.Restitution * 0.3f : bodyParameters.Restitution;
                body.Friction = bodyParameters.Friction;
                body.LinearDamping = bodyParameters.LinearDamping;
                body.AngularDamping = bodyParameters.AngularDamping;
            }
        }

        private static void ApplyKinematicTarget(BodyTrack track, int local, TrackObservation observation, float deltaTime)
        {
            var previous = local > 0 ? track.GetObservation(local - 1) : null;

            foreach (var body in track.Bodies)
            {
                Vector2 position = observation.Draw + body.LocalCentroid.Rotate(observation.Rotation);

                if (previous != null && !body.IsSpawning)
                {
                    Vector2 previousCentroid = Vector2.Transform(body.ImageCentroid, previous.Value.ImageTransform);
                    Vector2 previousPosition = previous.Value.Draw + previousCentroid.Rotate(previous.Value.Rotation);
                    body.LinearVelocity = (position - previousPosition) / deltaTime;
                    body.AngularVelocity = (observation.Rotation - previous.Value.Rotation) / deltaTime;
                }
                else
                {
                    body.LinearVelocity = Vector2.Zero;
                    body.AngularVelocity = 0f;
                }

                body.Position = position;
                body.Rotation = observation.Rotation;
                body.UpdateWorldAABB();
            }
        }

        private int ComputeSubsteps(float deltaTime)
        {
            float rigidTravel = 0f, rigidThickness = float.MaxValue;
            float fluidTravel = 0f, fluidThickness = float.MaxValue;

            for (int i = wallBodies.Count; i < activeBodies.Count; i++)
            {
                var body = activeBodies[i];
                float travel = body.IsSpawning ? 0f : body.LinearVelocity.Length() * deltaTime;
                if (body.IsParticle)
                {
                    fluidThickness = MathF.Min(fluidThickness, body.MinThickness);
                    fluidTravel = MathF.Max(fluidTravel, travel);
                }
                else
                {
                    rigidThickness = MathF.Min(rigidThickness, body.MinThickness);
                    if (body.Type != BodyType.Static) rigidTravel = MathF.Max(rigidTravel, travel);
                }
            }

            return Math.Max(RequiredSteps(rigidTravel, rigidThickness, 0.5f, MaxSubsteps), RequiredSteps(fluidTravel, fluidThickness, 1.5f, MaxFluidSubsteps));
        }

        private static int RequiredSteps(float travel, float thickness, float allowance, int maxSteps)
        {
            if (travel <= 0f || thickness == float.MaxValue) return 1;
            return Math.Clamp((int)MathF.Ceiling(travel / MathF.Max(thickness * allowance, 1f)), 1, maxSteps);
        }

        private void RebuildActiveBodies()
        {
            SortTracksIfNeeded();

            activeBodies.Clear();
            activeBodies.AddRange(wallBodies);
            foreach (var track in tracks)
            {
                if (!track.IsSpawned) continue;

                foreach (var body in track.Bodies) body.Owner = track;
                activeBodies.AddRange(track.Bodies);
            }
            bodiesDirty = false;
        }

        private void SortTracksIfNeeded()
        {
            if (!orderDirty) return;

            tracks.Sort((a, b) =>
            {
                int result = a.Layer.CompareTo(b.Layer);
                if (result == 0) result = a.StartFrame.CompareTo(b.StartFrame);
                if (result == 0) result = a.Key.GroupIndex.CompareTo(b.Key.GroupIndex);
                if (result == 0) result = a.Key.InputIndex.CompareTo(b.Key.InputIndex);
                if (result == 0) result = a.Sequence.CompareTo(b.Sequence);
                return result;
            });
            orderDirty = false;
            bodiesDirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void IntegrateBodies(float deltaTime)
        {
            foreach (var wall in wallBodies)
            {
                wall.UpdateWorldAABB();
            }

            CollectAttractors();

            foreach (var track in tracks)
            {
                if (!track.IsSpawned) continue;

                foreach (var body in track.Bodies)
                {
                    IntegrateBody(track, body, deltaTime);
                }
            }
        }

        private void CollectAttractors()
        {
            attractors.Clear();
            foreach (var track in tracks)
            {
                if (!track.IsSpawned || track.Bodies.Count == 0) continue;

                var bodyParameters = track.CurrentParameters;
                if (bodyParameters.Attraction == 0f) continue;

                float strength = bodyParameters.Attraction / track.Bodies.Count;
                foreach (var body in track.Bodies)
                {
                    if (body.IsGhost) continue;
                    attractors.Add((track, body.Position, strength, bodyParameters.AttractionRange));
                }
            }
        }

        private Vector2 GetAttraction(BodyTrack track, Vector2 position)
        {
            Vector2 acceleration = Vector2.Zero;
            foreach (var (owner, center, strength, range) in attractors)
            {
                if (owner == track) continue;

                Vector2 offset = center - position;
                float distance = offset.Length();
                if (distance < 1f) continue;

                float falloff = 1f;
                if (range > 0f)
                {
                    if (distance >= range) continue;
                    falloff = 1f - distance / range;
                }

                acceleration += offset / distance * (strength * falloff);
            }
            return acceleration;
        }

        private void IntegrateBody(BodyTrack track, RigidBody body, float deltaTime)
        {
            if (body.Type != BodyType.Static && !body.IsSpawning && body.Presence > 0f && !(body.IsGhost && body.Opacity <= 0f))
            {
                Vector2 acceleration = (body.Force * body.InvMass) + parameters.Gravity;
                if (attractors.Count > 0) acceleration += GetAttraction(track, body.Position);
                body.LinearVelocity += acceleration * deltaTime;
                if (activeAreas.Count > 0) ApplyAreaForces(track, body, deltaTime);

                body.Position += body.LinearVelocity * deltaTime;
                body.Rotation += body.AngularVelocity * deltaTime;

                body.LinearVelocity *= MathF.Exp(-body.LinearDamping * deltaTime);
                body.AngularVelocity *= MathF.Exp(-body.AngularDamping * deltaTime);

                if (MathF.Abs(body.AngularVelocity) < 0.03f)
                {
                    body.AngularVelocity = 0f;
                }

                body.ClearForces();
            }

            body.UpdateWorldAABB();
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void DetectContacts()
        {
            manifolds.Clear();
            contactOrigins.Clear();

            int count = activeBodies.Count;
            for (int i = 0; i < count; i++)
            {
                if (activeBodies[i].IsParticle) activeBodies[i].TouchingSolid = false;
            }
            if (sweepOrder.Length < count)
            {
                sweepOrder = new int[count];
                sweepKeys = new float[count];
            }
            for (int i = 0; i < count; i++)
            {
                sweepOrder[i] = i;
                sweepKeys[i] = activeBodies[i].WorldAABB.Min.X;
            }
            Array.Sort(sweepKeys, sweepOrder, 0, count);

            for (int a = 0; a < count; a++)
            {
                RigidBody bodyA = activeBodies[sweepOrder[a]];
                if (!bodyA.IsActive) continue;

                bool isParticleA = bodyA.IsParticle;
                float maxX = bodyA.WorldAABB.Max.X;

                for (int b = a + 1; b < count; b++)
                {
                    RigidBody bodyB = activeBodies[sweepOrder[b]];

                    if (bodyB.WorldAABB.Min.X > maxX) break;
                    if (isParticleA && bodyB.IsParticle) continue;
                    if (!bodyB.IsActive) continue;
                    if (IsCollisionDisabled(bodyA, bodyB)) continue;
                    if (bodyA.InvMass == 0f && bodyB.InvMass == 0f) continue;
                    if (!bodyA.WorldAABB.Overlaps(bodyB.WorldAABB)) continue;

                    int first = manifolds.Count;
                    CollisionDetection.Detect(bodyA, bodyB, manifolds);
                    for (int i = first; i < manifolds.Count; i++)
                    {
                        contactOrigins.Add((bodyA.Position, bodyB.Position));
                    }
                    if (manifolds.Count > first && isParticleA != bodyB.IsParticle)
                    {
                        (isParticleA ? bodyA : bodyB).TouchingSolid = true;
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void ResolveContacts()
        {
            for (int i = 0; i < manifolds.Count; i++)
            {
                var m = manifolds[i];
                var (originA, originB) = contactOrigins[i];

                float separated = Vector2.Dot((m.BodyB.Position - originB) - (m.BodyA.Position - originA), m.Normal);
                float depth = m.Depth - separated;

                CollisionResolver.Resolve(m, depth, parameters.MinVelocityForRestitution, parameters.PositionCorrectionPercent, parameters.PositionCorrectionSlop);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void RecordResults(int frame)
        {
            foreach (var track in tracks)
            {
                if (!track.IsInitialized) continue;

                int local = frame - track.StartFrame;
                if (local < 0 || frame >= track.EndFrame) continue;

                while (track.Results.Count <= local) track.Results.Add(default);

                var record = track.Results[local];
                int previousTriggerCount = record.IsWritten ? record.TriggerCount : 0;
                record.IsWritten = true;
                record.TriggerCount = 0;

                if (track.IsArea)
                {
                    foreach (var area in activeAreas)
                    {
                        if (area.Track == track) record.TriggerCount = area.Fired;
                    }
                    record.Shape = null;
                    record.Poses = [];
                    record.Centroids = [];
                    record.JointFrames = [];
                }
                else if (track.IsSpawned && track.IsFluid)
                {
                    RecordFluid(track, ref record);
                    record.JointFrames = CollectJointFrames(track, record.JointFrames);
                    record.Shape = track.SpawnedShape;
                    record.Transform = track.SpawnedTransform;
                    record.IsSeparate = false;
                }
                else if (track.IsSpawned)
                {
                    record.FluidShape = null;
                    record.FluidPositions = null;
                    record.FluidVelocities = null;
                    record.FluidPresences = null;
                    int count = track.Bodies.Count;
                    if (record.Poses == null || record.Poses.Length != count)
                    {
                        record.Poses = new PartPose[count];
                        record.Centroids = new Vector2[count];
                    }

                    for (int i = 0; i < count; i++)
                    {
                        var body = track.Bodies[i];
                        record.Poses[i] = new PartPose(body.Position, body.Rotation, body.Opacity);
                        record.Centroids[i] = body.LocalCentroid;
                    }

                    record.JointFrames = CollectJointFrames(track, record.JointFrames);
                    record.Shape = track.SpawnedShape;
                    record.Transform = track.SpawnedTransform;
                    record.IsSeparate = track.IsSeparate;
                }
                else
                {
                    record.Shape = null;
                    record.Poses = [];
                    record.Centroids = [];
                    record.JointFrames = [];
                    record.FluidShape = null;
                    record.FluidPositions = null;
                    record.FluidVelocities = null;
                    record.FluidPresences = null;
                }

                track.Results[local] = record;
                if (record.TriggerCount > 0)
                {
                    if (local < track.FirstTriggerLocal) track.FirstTriggerLocal = local;
                }
                else if (previousTriggerCount > 0 && local == track.FirstTriggerLocal)
                {
                    track.FirstTriggerLocal = FindFirstTrigger(track, local + 1);
                }
            }
        }

        private static int FindFirstTrigger(BodyTrack track, int from)
        {
            for (int local = from; local < track.Results.Count; local++)
            {
                var record = track.Results[local];
                if (record.IsWritten && record.TriggerCount > 0) return local;
            }
            return int.MaxValue;
        }

        #endregion

        #region 状態の保存と復元

        private WorldSnapshot Capture(int frame, WorldSnapshot? reuse = null)
        {
            var snapshot = reuse ?? new WorldSnapshot();
            snapshot.Frame = frame;

            foreach (var track in snapshot.Tracks.Keys)
            {
                if (!track.IsSpawned || track.SpawnedShape == null) staleSnapshotTracks.Add(track);
            }
            foreach (var track in staleSnapshotTracks) snapshot.Tracks.Remove(track);
            staleSnapshotTracks.Clear();

            foreach (var track in tracks)
            {
                if (!track.IsSpawned || track.SpawnedShape == null) continue;

                if (!snapshot.Tracks.TryGetValue(track, out var state))
                {
                    state = new TrackState();
                    snapshot.Tracks[track] = state;
                }
                if (state.Bodies.Length != track.Bodies.Count) state.Bodies = new BodyState[track.Bodies.Count];

                for (int i = 0; i < state.Bodies.Length; i++)
                {
                    state.Bodies[i] = new BodyState(track.Bodies[i]);
                }
                state.Shape = track.SpawnedShape;
                state.Transform = track.SpawnedTransform;
                state.FluidShape = track.IsFluid ? track.FluidShape : null;
            }
            return snapshot;
        }

        private void Restore(WorldSnapshot snapshot)
        {
            foreach (var track in tracks)
            {
                if (!snapshot.Tracks.TryGetValue(track, out var state))
                {
                    Despawn(track);
                    continue;
                }

                bool reusable = track.IsSpawned &&
                                ReferenceEquals(track.SpawnedShape, state.Shape) &&
                                track.SpawnedTransform == state.Transform &&
                                ReferenceEquals(track.FluidShape, state.FluidShape) &&
                                track.Bodies.Count == state.Bodies.Length;

                if (!reusable)
                {
                    track.Bodies = state.FluidShape != null ? CreateParticles(state.FluidShape, state.Transform, 1f)
                        : BodyFactory.CreateBodies(state.Shape, state.Transform, track.IsSeparate, track.BodyType, 1f, track.CollisionMargin);
                    track.FluidShape = state.FluidShape;
                    track.IsSpawned = true;
                    track.SpawnedShape = state.Shape;
                    track.SpawnedTransform = state.Transform;
                    bodiesDirty = true;
                }

                int count = Math.Min(track.Bodies.Count, state.Bodies.Length);
                for (int i = 0; i < count; i++)
                {
                    state.Bodies[i].ApplyTo(track.Bodies[i]);
                    track.Bodies[i].IsSpawning = false;
                    track.Bodies[i].UpdateWorldAABB();
                }
            }

            liveFrame = snapshot.Frame;
        }

        private void RestoreEmpty(int frame)
        {
            foreach (var track in tracks)
            {
                Despawn(track);
            }
            liveFrame = frame;
        }

        #endregion

        #region 壁

        private void RebuildWalls()
        {
            wallBodies.Clear();
            bodiesDirty = true;

            if (!parameters.EnableWalls) return;

            Size screen = parameters.ScreenSize;
            if (screen.Width <= 0 || screen.Height <= 0) return;

            float width = screen.Width;
            float height = screen.Height;

            float thickness = MathF.Max(width, height);

            CreateWall(new Vector2(0, -(height + thickness) / 2), width + thickness * 2, thickness);
            CreateWall(new Vector2(0, (height + thickness) / 2), width + thickness * 2, thickness);
            CreateWall(new Vector2(-(width + thickness) / 2, 0), thickness, height);
            CreateWall(new Vector2((width + thickness) / 2, 0), thickness, height);
        }

        private void CreateWall(Vector2 position, float width, float height)
        {
            var wall = new RigidBody(position, 0f, BodyType.Static);
            wall.AddColliders([new BoxCollider(width, height)]);
            wall.UpdateWorldAABB();
            wallBodies.Add(wall);
        }

        #endregion
    }
}
