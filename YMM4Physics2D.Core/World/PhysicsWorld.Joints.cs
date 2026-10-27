using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Geometry;

namespace YMM4Physics2D.Core.World
{
    public sealed partial class PhysicsWorld
    {
        private const float JointPositionCorrection = 0.8f;
        private const int JointPositionIterations = 4;

        private struct ActiveJoint
        {
            public object Key;
            public JointDefinition Definition;
            public BodyTrack TrackA;
            public RigidBody BodyA;
            public Vector2 LocalA;
            public RigidBody? BodyB;
            public Vector2 LocalB;
            public Vector2 WorldPoint;
            public float RestLength;
            public float ReferenceAngle;
        }

        private readonly List<ActiveJoint> activeJoints = [];
        private readonly HashSet<(long, long)> nonCollidingPairs = [];
        private string[] bodyNames = [];
        private string[] triggerNames = [];

        public void CollectBodyNames(ISet<string> names)
        {
            foreach (var name in Volatile.Read(ref bodyNames)) names.Add(name);
        }

        public void CollectTriggerNames(ISet<string> names)
        {
            foreach (var name in Volatile.Read(ref triggerNames)) names.Add(name);
        }

        private void RefreshNames()
        {
            var bodies = new List<string>();
            var triggers = new List<string>();
            foreach (var track in tracks)
            {
                if (track.IsRemoved || string.IsNullOrEmpty(track.Name)) continue;
                if (!track.IsArea) bodies.Add(track.Name);
                else if (track.Area is { Kind: AreaKind.Trigger }) triggers.Add(track.Name);
            }
            Volatile.Write(ref bodyNames, [.. bodies]);
            Volatile.Write(ref triggerNames, [.. triggers]);
        }

        public BodyTrack? FindTrack(int startFrame, int layer, int groupIndex, int inputIndex, bool wait = true)
        {
            bool entered = false;
            if (wait) Monitor.Enter(syncRoot, ref entered);
            else entered = Monitor.TryEnter(syncRoot);
            if (!entered) return null;

            try
            {
                foreach (var track in tracks)
                {
                    if (!track.IsInitialized || track.IsRemoved) continue;
                    if (track.StartFrame == startFrame && track.Layer == layer &&
                        track.Key.GroupIndex == groupIndex && track.Key.InputIndex == inputIndex)
                    {
                        return track;
                    }
                }
                return null;
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }

        public void UpdateJoint(BodyTrack track, object jointKey, in JointDefinition definition, Func<JointDefinition>? provider = null)
        {
            lock (syncRoot)
            {
                if (track.IsRemoved || !track.IsInitialized) return;

                if (track.SetJoint(jointKey, definition, provider))
                {
                    Invalidate(track.StartFrame);
                }
            }
        }

        private void PrepareJoints()
        {
            activeJoints.Clear();
            nonCollidingPairs.Clear();

            foreach (var track in tracks)
            {
                if (!track.IsSpawned || track.Joints.Count == 0) continue;

                foreach (var entry in track.Joints)
                {
                    var definition = entry.Definition;

                    BodyTrack? target = null;
                    if (definition.TargetMode != JointTargetMode.FixedPoint)
                    {
                        target = FindTarget(track, definition);
                        if (target == null || !target.IsSpawned) continue;
                    }

                    var bodyA = SelectBody(track, definition.AnchorA);
                    if (bodyA == null) continue;

                    var joint = new ActiveJoint
                    {
                        Key = entry.Key,
                        Definition = definition,
                        TrackA = track,
                        BodyA = bodyA,
                        LocalA = Vector2.Transform(definition.AnchorA, track.SpawnedTransform) - bodyA.LocalCentroid,
                    };

                    if (target != null)
                    {
                        var bodyB = SelectBody(target, definition.AnchorB);
                        if (bodyB == null) continue;

                        joint.BodyB = bodyB;
                        joint.LocalB = Vector2.Transform(definition.AnchorB, target.SpawnedTransform) - bodyB.LocalCentroid;

                        if (!definition.CollideConnected) nonCollidingPairs.Add(PairKey(track, target));
                    }
                    else
                    {
                        var placement = track.GetObservation(0);
                        if (placement == null) continue;
                        joint.WorldPoint = GetFixedPoint(placement.Value, definition);
                    }

                    if (!TryGetInitialPlacement(track, target, definition, out joint.RestLength, out joint.ReferenceAngle)) continue;

                    activeJoints.Add(joint);
                }
            }
        }

        private static bool TryGetInitialPlacement(BodyTrack trackA, BodyTrack? trackB, in JointDefinition definition, out float length, out float referenceAngle)
        {
            length = 0f;
            referenceAngle = 0f;

            int activation = Math.Max(trackA.StartFrame, trackB?.StartFrame ?? trackA.StartFrame);

            var observationA = trackA.GetObservation(activation - trackA.StartFrame);
            if (observationA == null) return false;
            Vector2 worldA = PlacedPoint(observationA.Value, definition.AnchorA);

            Vector2 worldB;
            float rotationB;
            if (trackB != null)
            {
                var observationB = trackB.GetObservation(activation - trackB.StartFrame);
                if (observationB == null) return false;
                worldB = PlacedPoint(observationB.Value, definition.AnchorB);
                rotationB = observationB.Value.Rotation;
            }
            else
            {
                worldB = GetFixedPoint(observationA.Value, definition);
                rotationB = 0f;
            }

            length = definition.Length > 0f ? definition.Length : Vector2.Distance(worldA, worldB);
            referenceAngle = rotationB - observationA.Value.Rotation;
            return true;
        }

        private static Vector2 PlacedPoint(in TrackObservation observation, Vector2 imagePoint)
        {
            return observation.Draw + Vector2.Transform(imagePoint, observation.ImageTransform).Rotate(observation.Rotation);
        }

        private static Vector2 GetFixedPoint(in TrackObservation placement, in JointDefinition definition)
        {
            return placement.Draw + definition.AnchorB;
        }

        private BodyTrack? FindTarget(BodyTrack self, in JointDefinition definition)
        {
            return definition.TargetMode switch
            {
                JointTargetMode.Name => FindTrackByName(self, definition.TargetName),
                JointTargetMode.Layer => FindTrackByLayer(self, definition.TargetLayer - 1),
                JointTargetMode.Previous => FindPreviousTrack(self),
                _ => null,
            };
        }

        private BodyTrack? FindTrackByLayer(BodyTrack self, int layer)
        {
            foreach (var track in tracks)
            {
                if (track != self && track.IsSpawned && track.Layer == layer) return track;
            }
            return null;
        }

        private BodyTrack? FindTrackByName(BodyTrack self, string name)
        {
            if (string.IsNullOrEmpty(name)) return null;

            foreach (var track in tracks)
            {
                if (track != self && track.IsSpawned && track.Name == name) return track;
            }
            return null;
        }

        private BodyTrack? FindPreviousTrack(BodyTrack self)
        {
            var key = self.Key;
            if (key.InputIndex > 0)
            {
                var previous = new TrackKey(key.Owner, key.GroupIndex, key.InputIndex - 1);
                if (trackMap.TryGetValue(previous, out var track) && track.IsSpawned) return track;
            }
            if (key.GroupIndex > 0)
            {
                var previous = new TrackKey(key.Owner, key.GroupIndex - 1, key.InputIndex);
                if (trackMap.TryGetValue(previous, out var track) && track.IsSpawned) return track;
            }
            return null;
        }

        private static RigidBody? SelectBody(BodyTrack track, Vector2 imagePoint)
        {
            if (track.Bodies.Count == 0) return null;
            if (track.Bodies.Count == 1 || track.SpawnedShape == null) return track.Bodies[0];
            if (track.IsFluid || !track.IsSeparate) return NearestBody(track, imagePoint);

            var components = track.SpawnedShape.Components;
            for (int i = 0; i < components.Length && i < track.Bodies.Count; i++)
            {
                foreach (var part in components[i].ConvexParts)
                {
                    if (GeometryUtils.IsPointInPolygon(imagePoint, part)) return track.Bodies[i];
                }
            }

            return NearestBody(track, imagePoint);
        }

        private static RigidBody NearestBody(BodyTrack track, Vector2 imagePoint)
        {
            RigidBody nearest = track.Bodies[0];
            float bestDistance = float.MaxValue;
            foreach (var body in track.Bodies)
            {
                float distance = Vector2.DistanceSquared(body.ImageCentroid, imagePoint);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    nearest = body;
                }
            }
            return nearest;
        }

        private static bool IsSpawning(in ActiveJoint joint)
        {
            return joint.BodyA.IsSpawning || (joint.BodyB?.IsSpawning ?? false);
        }

        private static (long, long) PairKey(BodyTrack a, BodyTrack b)
        {
            return a.Sequence < b.Sequence ? (a.Sequence, b.Sequence) : (b.Sequence, a.Sequence);
        }

        private bool IsCollisionDisabled(RigidBody bodyA, RigidBody bodyB)
        {
            if (nonCollidingPairs.Count == 0) return false;
            if (bodyA.Owner is not BodyTrack trackA || bodyB.Owner is not BodyTrack trackB || trackA == trackB) return false;
            return nonCollidingPairs.Contains(PairKey(trackA, trackB));
        }

        private void ApplySprings(float deltaTime)
        {
            foreach (var joint in activeJoints)
            {
                if (joint.Definition.Type != JointType.Spring || IsSpawning(joint)) continue;

                var anchors = GetAnchors(joint);
                Vector2 delta = anchors.PointB - anchors.PointA;
                float length = delta.Length();
                if (length < 1e-4f) continue;

                Vector2 axis = delta / length;
                float effectiveInvMass = EffectiveInvMass(joint, anchors, axis);
                if (effectiveInvMass <= 0f) continue;

                float effectiveMass = 1f / effectiveInvMass;
                float omega = MathF.Min(2f * MathF.PI * MathF.Max(joint.Definition.Frequency, 0f), 0.8f / deltaTime);
                float stiffness = effectiveMass * omega * omega;
                float damping = 2f * effectiveMass * MathF.Max(joint.Definition.DampingRatio, 0f) * omega;

                float stretch = length - joint.RestLength;
                float speed = Vector2.Dot(anchors.VelocityB - anchors.VelocityA, axis);
                float impulse = -(stiffness * stretch + damping * speed) * deltaTime;

                ApplyJointImpulse(joint, anchors, axis * impulse);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void SolveJoints()
        {
            foreach (var joint in activeJoints)
            {
                if (IsSpawning(joint)) continue;

                switch (joint.Definition.Type)
                {
                    case JointType.Hinge:
                    case JointType.Weld:
                        SolvePointConstraint(joint);
                        if (joint.Definition.Type == JointType.Weld) SolveAngleConstraint(joint);
                        break;
                    case JointType.Distance:
                    case JointType.Rope:
                        SolveDistanceConstraint(joint, joint.Definition.Type == JointType.Rope);
                        break;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void SolveJointPositions()
        {
            if (activeJoints.Count == 0) return;

            for (int iteration = 0; iteration < JointPositionIterations; iteration++)
            {
                foreach (var joint in activeJoints)
                {
                    if (IsSpawning(joint)) continue;

                    switch (joint.Definition.Type)
                    {
                        case JointType.Hinge:
                        case JointType.Weld:
                            CorrectPoint(joint);
                            if (joint.Definition.Type == JointType.Weld) CorrectAngle(joint);
                            break;
                        case JointType.Distance:
                        case JointType.Rope:
                            CorrectDistance(joint, joint.Definition.Type == JointType.Rope);
                            break;
                    }
                }
            }

            foreach (var joint in activeJoints)
            {
                joint.BodyA.UpdateWorldAABB();
                joint.BodyB?.UpdateWorldAABB();
            }
        }

        private static void CorrectPoint(in ActiveJoint joint)
        {
            var anchors = GetAnchors(joint);
            if (!TrySolvePointMatrix(joint, anchors, -(anchors.PointB - anchors.PointA) * JointPositionCorrection, out var impulse)) return;
            ApplyPositionImpulse(joint, anchors, impulse);
        }

        private static void CorrectAngle(in ActiveJoint joint)
        {
            var bodyA = joint.BodyA;
            var bodyB = joint.BodyB;
            float inertiaSum = bodyA.InvInertia + (bodyB?.InvInertia ?? 0f);
            if (inertiaSum <= 0f) return;

            float error = (bodyB?.Rotation ?? 0f) - bodyA.Rotation - joint.ReferenceAngle;
            float impulse = -error * JointPositionCorrection / inertiaSum;
            bodyA.Rotation -= bodyA.InvInertia * impulse;
            if (bodyB != null) bodyB.Rotation += bodyB.InvInertia * impulse;
        }

        private static void CorrectDistance(in ActiveJoint joint, bool isRope)
        {
            var anchors = GetAnchors(joint);
            Vector2 delta = anchors.PointB - anchors.PointA;
            float length = delta.Length();
            if (length < 1e-4f) return;

            float error = length - joint.RestLength;
            if (isRope && error <= 0f) return;

            Vector2 axis = delta / length;
            float effectiveInvMass = EffectiveInvMass(joint, anchors, axis);
            if (effectiveInvMass <= 0f) return;

            ApplyPositionImpulse(joint, anchors, axis * (-error * JointPositionCorrection / effectiveInvMass));
        }

        private static void ApplyPositionImpulse(in ActiveJoint joint, in JointAnchors anchors, Vector2 impulse)
        {
            var bodyA = joint.BodyA;
            bodyA.Position -= impulse * bodyA.InvMass;
            bodyA.Rotation -= anchors.OffsetA.Cross(impulse) * bodyA.InvInertia;

            if (joint.BodyB is { } bodyB)
            {
                bodyB.Position += impulse * bodyB.InvMass;
                bodyB.Rotation += anchors.OffsetB.Cross(impulse) * bodyB.InvInertia;
            }
        }

        private static bool TrySolvePointMatrix(in ActiveJoint joint, in JointAnchors anchors, Vector2 target, out Vector2 impulse)
        {
            var bodyA = joint.BodyA;
            var bodyB = joint.BodyB;

            float massSum = bodyA.InvMass + (bodyB?.InvMass ?? 0f);
            float inertiaA = bodyA.InvInertia;
            float inertiaB = bodyB?.InvInertia ?? 0f;
            Vector2 rA = anchors.OffsetA;
            Vector2 rB = anchors.OffsetB;

            float k11 = massSum + inertiaA * rA.Y * rA.Y + inertiaB * rB.Y * rB.Y;
            float k12 = -inertiaA * rA.X * rA.Y - inertiaB * rB.X * rB.Y;
            float k22 = massSum + inertiaA * rA.X * rA.X + inertiaB * rB.X * rB.X;
            float determinant = k11 * k22 - k12 * k12;
            if (MathF.Abs(determinant) < 1e-12f)
            {
                impulse = Vector2.Zero;
                return false;
            }

            impulse = new Vector2(
                (k22 * target.X - k12 * target.Y) / determinant,
                (k11 * target.Y - k12 * target.X) / determinant);
            return true;
        }

        private static void SolvePointConstraint(in ActiveJoint joint)
        {
            var anchors = GetAnchors(joint);
            Vector2 target = anchors.VelocityA - anchors.VelocityB;
            if (!TrySolvePointMatrix(joint, anchors, target, out var impulse)) return;

            ApplyJointImpulse(joint, anchors, impulse);
        }

        private static void SolveAngleConstraint(in ActiveJoint joint)
        {
            var bodyA = joint.BodyA;
            var bodyB = joint.BodyB;

            float inertiaSum = bodyA.InvInertia + (bodyB?.InvInertia ?? 0f);
            if (inertiaSum <= 0f) return;

            float speed = (bodyB?.AngularVelocity ?? 0f) - bodyA.AngularVelocity;
            float impulse = -speed / inertiaSum;

            bodyA.AngularVelocity -= bodyA.InvInertia * impulse;
            if (bodyB != null) bodyB.AngularVelocity += bodyB.InvInertia * impulse;
        }

        private static void SolveDistanceConstraint(in ActiveJoint joint, bool isRope)
        {
            var anchors = GetAnchors(joint);
            Vector2 delta = anchors.PointB - anchors.PointA;
            float length = delta.Length();
            if (length < 1e-4f) return;

            float error = length - joint.RestLength;
            if (isRope && error <= 0f) return;

            Vector2 axis = delta / length;
            float effectiveInvMass = EffectiveInvMass(joint, anchors, axis);
            if (effectiveInvMass <= 0f) return;

            float speed = Vector2.Dot(anchors.VelocityB - anchors.VelocityA, axis);
            float impulse = -speed / effectiveInvMass;

            if (isRope) impulse = MathF.Min(impulse, 0f);

            ApplyJointImpulse(joint, anchors, axis * impulse);
        }

        private readonly record struct JointAnchors(Vector2 OffsetA, Vector2 PointA, Vector2 VelocityA, Vector2 OffsetB, Vector2 PointB, Vector2 VelocityB);

        private static JointAnchors GetAnchors(in ActiveJoint joint)
        {
            var bodyA = joint.BodyA;
            Vector2 offsetA = joint.LocalA.Rotate(bodyA.Rotation);
            Vector2 velocityA = bodyA.LinearVelocity + Vector2Extensions.Cross(bodyA.AngularVelocity, offsetA);

            if (joint.BodyB is { } bodyB)
            {
                Vector2 offsetB = joint.LocalB.Rotate(bodyB.Rotation);
                Vector2 velocityB = bodyB.LinearVelocity + Vector2Extensions.Cross(bodyB.AngularVelocity, offsetB);
                return new JointAnchors(offsetA, bodyA.Position + offsetA, velocityA, offsetB, bodyB.Position + offsetB, velocityB);
            }

            return new JointAnchors(offsetA, bodyA.Position + offsetA, velocityA, Vector2.Zero, joint.WorldPoint, Vector2.Zero);
        }

        private static float EffectiveInvMass(in ActiveJoint joint, in JointAnchors anchors, Vector2 axis)
        {
            float crossA = anchors.OffsetA.Cross(axis);
            float result = joint.BodyA.InvMass + joint.BodyA.InvInertia * crossA * crossA;

            if (joint.BodyB is { } bodyB)
            {
                float crossB = anchors.OffsetB.Cross(axis);
                result += bodyB.InvMass + bodyB.InvInertia * crossB * crossB;
            }
            return result;
        }

        private static void ApplyJointImpulse(in ActiveJoint joint, in JointAnchors anchors, Vector2 impulse)
        {
            var bodyA = joint.BodyA;
            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyA.AngularVelocity -= anchors.OffsetA.Cross(impulse) * bodyA.InvInertia;

            if (joint.BodyB is { } bodyB)
            {
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += anchors.OffsetB.Cross(impulse) * bodyB.InvInertia;
            }
        }

        private JointFrame[] CollectJointFrames(BodyTrack track, JointFrame[]? reuse)
        {
            int count = 0;
            foreach (var joint in activeJoints)
            {
                if (joint.TrackA == track) count++;
            }
            if (count == 0) return [];

            var frames = reuse != null && reuse.Length == count ? reuse : new JointFrame[count];
            int index = 0;
            foreach (var joint in activeJoints)
            {
                if (joint.TrackA != track) continue;

                var anchors = GetAnchors(joint);
                var toAnchorA = InverseLinear(track.SpawnedTransform * Matrix3x2.CreateRotation(joint.BodyA.Rotation));
                var toAnchorB = joint.BodyB is { } bodyB && bodyB.Owner is BodyTrack target
                    ? InverseLinear(target.SpawnedTransform * Matrix3x2.CreateRotation(bodyB.Rotation))
                    : Matrix3x2.Identity;

                frames[index++] = new JointFrame(joint.Key, anchors.PointA, anchors.PointB, true, toAnchorA, toAnchorB, joint.Definition.LineWidth, joint.Definition.LineColor);
            }
            return frames;
        }

        private static Matrix3x2 InverseLinear(Matrix3x2 matrix)
        {
            matrix.Translation = Vector2.Zero;
            return Matrix3x2.Invert(matrix, out var inverse) ? inverse : Matrix3x2.Identity;
        }

        public bool TryGetJointFrame(BodyTrack track, object jointKey, int frame, bool wait, out JointFrame jointFrame)
        {
            return TryGetJointFrame(track, jointKey, frame, wait, wait, out jointFrame);
        }

        public bool TryGetJointFrame(BodyTrack track, object jointKey, int frame, bool wait, bool simulate, out JointFrame jointFrame)
        {
            jointFrame = default;

            bool entered = false;
            if (wait) Monitor.Enter(syncRoot, ref entered);
            else entered = Monitor.TryEnter(syncRoot);
            if (!entered) return false;

            try
            {
                if (track.IsRemoved || !track.IsInitialized) return false;
                if (simulate) PullSettings();

                int local = frame - track.StartFrame;
                bool hasRecord = local >= 0 && local < track.Results.Count && track.Results[local].IsWritten;
                if (!hasRecord && (!simulate || !TryEnsureRecord(track, frame, out _))) return false;

                if (local < 0 || local >= track.Results.Count) return false;

                var record = track.Results[local];
                if (record.JointFrames != null)
                {
                    foreach (var joint in record.JointFrames)
                    {
                        if (ReferenceEquals(joint.Key, jointKey))
                        {
                            jointFrame = joint;
                            return true;
                        }
                    }
                }

                var entry = track.Joints.FirstOrDefault(j => ReferenceEquals(j.Key, jointKey));
                if (entry == null || record.Poses.Length == 0 || record.Shape == null) return false;

                var pose = record.Poses[0];
                var linear = record.Transform * Matrix3x2.CreateRotation(pose.Rotation);
                linear.Translation = Vector2.Zero;
                Vector2 pointA = pose.Position + Vector2.Transform(Vector2.Transform(entry.Definition.AnchorA, record.Transform) - record.Centroids[0], Matrix3x2.CreateRotation(pose.Rotation));

                Vector2 pointB = pointA;
                if (entry.Definition.TargetMode == JointTargetMode.FixedPoint && track.GetObservation(0) is { } placement)
                {
                    pointB = GetFixedPoint(placement, entry.Definition);
                }

                jointFrame = new JointFrame(jointKey, pointA, pointB, false, InverseLinear(linear), Matrix3x2.Identity, 0f, 0);
                return true;
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }

    }
}
