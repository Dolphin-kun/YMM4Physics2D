using System.Numerics;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Shapes;

namespace YMM4Physics2D.Core.World
{
    public sealed class BodyTrack
    {
        internal struct FrameRecord
        {
            public bool IsWritten;
            public ShapeVersion? Shape;
            public Matrix3x2 Transform;
            public bool IsSeparate;
            public PartPose[] Poses;
            public Vector2[] Centroids;
            public JointFrame[] JointFrames;
            public int TriggerCount;
            public FluidShape? FluidShape;
            public Vector2[]? FluidPositions;
            public Vector2[]? FluidVelocities;
            public float[]? FluidPresences;
        }

        internal sealed class JointEntry(object key, JointDefinition definition, long serial, long order)
        {
            public object Key { get; } = key;
            public JointDefinition Definition { get; set; } = definition;
            public Func<JointDefinition>? Provider { get; set; }
            public long Serial { get; set; } = serial;
            public long Order { get; } = order;
        }

        private readonly Dictionary<object, JointEntry> joints = new(ReferenceEqualityComparer.Instance);
        private readonly List<JointEntry> orderedJoints = [];
        private long nextJointOrder;

        private readonly SortedList<int, TrackObservation> observations = [];
        private readonly SortedList<int, ShapeVersion> shapes = [];
        private readonly Dictionary<ulong, ShapeVersion> shapesByHash = [];
        private readonly Dictionary<int, BodyParameters> parameterCache = [];
        private readonly Dictionary<int, AreaParameters> areaParameterCache = [];
        private int onceCapturedLocal = int.MaxValue;
        private int parameterRefreshRequested;
        private volatile object? primaryWriter;

        internal TrackKey Key { get; }
        internal long Sequence { get; }
        internal bool IsInitialized { get; set; }
        internal bool IsRemoved { get; set; }

        internal int StartFrame { get; set; }
        internal int EndFrame { get; set; }
        internal int Layer { get; set; }
        internal BodyType BodyType { get; set; }
        internal bool IsSeparate { get; set; }
        internal float CollisionMargin { get; set; }
        internal int FluidResolution { get; set; }
        internal bool IsFluid => BodyType == BodyType.Fluid;
        internal FluidShape? FluidShape { get; set; }
        internal Dictionary<(ShapeVersion Shape, Matrix3x2 Transform, int Resolution), FluidShape?> FluidShapeCache { get; } = [];
        internal ShapeUpdateMode ShapeUpdateMode { get; set; }
        internal BodyParameterProvider? ParameterProvider { get; set; }
        internal Func<TrackSettings>? SettingsProvider { get; set; }
        internal int LastLocal { get; set; }
        internal int MaxObservedLocal { get; private set; } = -1;
        internal string Name { get; set; } = string.Empty;

        internal AreaDefinition? Area { get; set; }
        internal AreaBounds AreaBounds { get; set; }
        internal AreaParameterProvider? AreaParameterProvider { get; set; }
        internal bool IsArea => Area != null;
        internal int FirstTriggerLocal { get; set; } = int.MaxValue;

        internal long UpdateSerial { get; set; }

        internal IReadOnlyList<JointEntry> Joints => orderedJoints;

        internal List<object> Writers { get; } = [];
        internal List<object> Observers { get; } = [];
        internal int AttachCount => Writers.Count + Observers.Count;
        internal int SuspectCount { get; set; }
        internal long LastEvidenceTime { get; set; }
        internal int LastEvidenceFrame { get; set; }
        internal int MinEvidenceFrame { get; set; }

        internal List<RigidBody> Bodies { get; set; } = [];
        internal BodyParameters CurrentParameters { get; set; }
        internal bool IsSpawned { get; set; }
        internal ShapeVersion? SpawnedShape { get; set; }
        internal Matrix3x2 SpawnedTransform { get; set; }
        internal SortedSet<int> SpawnLocals { get; } = [];
        internal List<FrameRecord> Results { get; } = [];

        internal BodyTrack(TrackKey key, long sequence)
        {
            Key = key;
            Sequence = sequence;
        }

        internal int Length => EndFrame - StartFrame;

        #region 描画情報

        internal TrackObservation? GetObservation(int local)
        {
            int count = observations.Count;
            if (count == 0) return null;

            var keys = observations.Keys;
            var values = observations.Values;

            int index = LowerBound(keys, local);
            if (index < count && keys[index] == local) return values[index];
            if (index == 0) return values[0];

            if (index == count)
            {
                int lastKey = keys[count - 1];
                if (local == lastKey + 1 && count >= 2 && keys[count - 2] == lastKey - 1)
                {
                    return TrackObservation.Lerp(values[count - 2], values[count - 1], 2f);
                }
                return values[count - 1];
            }

            int previousKey = keys[index - 1];
            int nextKey = keys[index];
            float t = (float)(local - previousKey) / (nextKey - previousKey);
            return TrackObservation.Lerp(values[index - 1], values[index], t);
        }

        internal void RefreshPrimaryWriter()
        {
            primaryWriter = Writers.Count > 0 ? Writers[0] : null;
        }

        internal bool IsPrimaryWriter(object? writer)
        {
            return writer == null || primaryWriter is not { } primary || ReferenceEquals(primary, writer);
        }

        internal bool Matches(int startFrame, int layer, int groupIndex, int inputIndex)
        {
            return IsInitialized && !IsRemoved && StartFrame == startFrame && Layer == layer &&
                   Key.GroupIndex == groupIndex && Key.InputIndex == inputIndex;
        }

        internal bool HasObservation(int local) => observations.ContainsKey(local);

        internal int? RecordObservation(int local, in TrackObservation observation)
        {
            MaxObservedLocal = Math.Max(MaxObservedLocal, local);

            if (observations.TryGetValue(local, out var existing))
            {
                if (existing.NearlyEquals(observation)) return null;

                observations.Clear();
                observations[local] = observation;
                return 0;
            }

            var before0 = GetObservation(local);
            var before1 = GetObservation(local + 1);

            var keys = observations.Keys;
            int index = LowerBound(keys, local);
            int rangeStart = index > 0 ? keys[index - 1] + 1 : 0;
            bool exactMatch = index < keys.Count && keys[index] == local;
            int nextIndex = exactMatch ? index + 1 : index;
            int rangeEnd = nextIndex < keys.Count ? keys[nextIndex] : int.MaxValue;

            observations[local] = observation;

            var after1 = GetObservation(local + 1);

            bool scaleChanged = !ScaleEquals(before0, observation) || !ScaleEquals(before1, after1);
            bool poseChanged = !PoseEquals(before0, observation) || !PoseEquals(before1, after1);

            bool changed = scaleChanged || (poseChanged && (BodyType == BodyType.Static || SpawnLocals.GetViewBetween(rangeStart, rangeEnd == int.MaxValue ? int.MaxValue : rangeEnd - 1).Count > 0));
            return changed ? rangeStart : null;
        }

        private static bool ScaleEquals(TrackObservation? a, TrackObservation? b)
        {
            if (a == null || b == null) return a == null && b == null;
            var x = a.Value;
            var y = b.Value;
            return x.Invert == y.Invert && Vector2.DistanceSquared(x.Zoom, y.Zoom) < 1e-10f &&
                   (!x.Invert || Vector2.DistanceSquared(x.CenterPoint, y.CenterPoint) < 1e-6f);
        }

        private static bool PoseEquals(TrackObservation? a, TrackObservation? b)
        {
            if (a == null || b == null) return a == null && b == null;
            return Vector2.DistanceSquared(a.Value.Draw, b.Value.Draw) < 1e-6f && MathF.Abs(a.Value.Rotation - b.Value.Rotation) < 1e-5f;
        }

        #endregion

        #region 形状

        internal ShapeVersion? GetShape(int local)
        {
            int count = shapes.Count;
            if (count == 0) return null;
            if (ShapeUpdateMode == ShapeUpdateMode.Once) return shapes.Values[0];

            var keys = shapes.Keys;
            int index = LowerBound(keys, local);
            if (index < count && keys[index] == local) return shapes.Values[index];
            return index == 0 ? shapes.Values[0] : shapes.Values[index - 1];
        }

        internal ShapeVersion? FindShape(ulong hash)
        {
            return shapesByHash.GetValueOrDefault(hash);
        }

        internal bool HasShape => shapes.Count > 0;

        internal int? RecordShape(int local, ShapeVersion shape, bool replaced)
        {
            if (shapesByHash.TryGetValue(shape.Hash, out var known))
            {
                shape = known;
            }
            else
            {
                if (shapesByHash.Count > 512) shapesByHash.Clear();
                shapesByHash[shape.Hash] = shape;
            }

            if (ShapeUpdateMode == ShapeUpdateMode.Once)
            {
                var existing = shapes.Count > 0 ? shapes.Values[0] : null;
                bool replace = existing == null
                    || replaced
                    || (existing.IsEmpty && !shape.IsEmpty)
                    || (local < onceCapturedLocal && !shape.IsEmpty)
                    || (local == onceCapturedLocal && existing.Hash != shape.Hash);
                if (!replace) return null;

                shapes.Clear();
                shapes[0] = shape;
                onceCapturedLocal = local;
                return existing == null || existing.Hash != shape.Hash ? 0 : null;
            }

            if (shapes.TryGetValue(local, out var sameFrame) && sameFrame.Hash != shape.Hash)
            {
                shapes.Clear();
                shapes[local] = shape;
                return 0;
            }

            var before = GetShape(local);
            var keys = shapes.Keys;
            int index = LowerBound(keys, local);
            bool hasPrevious = index > 0;

            shapes[local] = shape;

            if (before != null && before.Hash == shape.Hash) return null;
            return hasPrevious ? local : 0;
        }

        internal void ClearShapes()
        {
            shapes.Clear();
            onceCapturedLocal = int.MaxValue;
        }

        #endregion

        #region ジョイント

        internal bool SetJoint(object key, in JointDefinition definition, Func<JointDefinition>? provider)
        {
            if (joints.TryGetValue(key, out var entry))
            {
                entry.Serial = UpdateSerial;
                entry.Provider = provider;
                if (entry.Definition == definition) return false;

                entry.Definition = definition;
                return true;
            }

            entry = new JointEntry(key, definition, UpdateSerial, nextJointOrder++) { Provider = provider };
            joints[key] = entry;
            orderedJoints.Add(entry);
            return true;
        }

        internal bool PruneJoints()
        {
            if (joints.Count == 0) return false;

            bool removed = false;
            foreach (var (key, entry) in joints.ToList())
            {
                if (entry.Serial < UpdateSerial - 1)
                {
                    joints.Remove(key);
                    orderedJoints.Remove(entry);
                    removed = true;
                }
            }
            return removed;
        }

        #endregion

        #region パラメーター

        internal BodyParameters GetParameters(int local, int fps)
        {
            if (parameterCache.TryGetValue(local, out var cached)) return cached;

            var parameters = ParameterProvider?.Invoke(local, Length, fps) ?? new BodyParameters(20f, 0.5f, 0.5f, 0.05f, 3f);
            parameterCache[local] = parameters;
            return parameters;
        }

        internal bool CheckParameters(int local, in BodyParameters parameters)
        {
            if (parameterCache.TryGetValue(local, out var cached) && cached != parameters)
            {
                parameterCache.Clear();
                parameterCache[local] = parameters;
                return true;
            }

            parameterCache[local] = parameters;
            return false;
        }

        internal void RequestParameterRefresh()
        {
            Volatile.Write(ref parameterRefreshRequested, 1);
        }

        internal bool ConsumeParameterRefresh()
        {
            return Interlocked.Exchange(ref parameterRefreshRequested, 0) != 0;
        }

        internal void ClearParameterCache()
        {
            parameterCache.Clear();
            areaParameterCache.Clear();
        }

        internal AreaParameters GetAreaParameters(int local, int fps)
        {
            if (areaParameterCache.TryGetValue(local, out var cached)) return cached;

            var parameters = AreaParameterProvider?.Invoke(local, Length, fps) ?? default;
            areaParameterCache[local] = parameters;
            return parameters;
        }

        internal bool CheckAreaParameters(int local, in AreaParameters parameters)
        {
            if (areaParameterCache.TryGetValue(local, out var cached) && cached != parameters)
            {
                areaParameterCache.Clear();
                areaParameterCache[local] = parameters;
                return true;
            }

            areaParameterCache[local] = parameters;
            return false;
        }

        #endregion

        private static int LowerBound(IList<int> keys, int value)
        {
            int low = 0;
            int high = keys.Count;
            while (low < high)
            {
                int mid = (low + high) >>> 1;
                if (keys[mid] < value) low = mid + 1;
                else high = mid;
            }
            return low;
        }
    }
}
