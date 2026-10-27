using System.Numerics;
using YMM4Physics2D.Core.Bodies;

namespace YMM4Physics2D.Core.World
{
    public sealed partial class PhysicsWorld
    {
        private sealed class ActiveArea
        {
            public required BodyTrack Track;
            public required AreaDefinition Definition;
            public required AreaParameters Parameters;
            public required Matrix3x2 ToLocal;
            public required AreaBounds Bounds;
            public required Vector2 Center;
            public Vector2 CornerMin;
            public Vector2 CornerMax;
            public Vector2 Axis0;
            public Vector2 Range0;
            public Vector2 Axis1;
            public Vector2 Range1;
            public Vector2 Down;
            public Vector2 Side;
            public Vector2 DownRange;
            public Vector2 SideRange;
            public int Fired;
        }

        private readonly List<ActiveArea> activeAreas = [];
        private float simulationTime;

        private void CollectArea(BodyTrack track, int local, in TrackObservation observation)
        {
            var bounds = track.AreaBounds;
            if (bounds.IsEmpty || track.Area is not { } definition) return;

            var toWorld = observation.ImageTransform * Matrix3x2.CreateRotation(observation.Rotation) * Matrix3x2.CreateTranslation(observation.Draw);
            if (!Matrix3x2.Invert(toWorld, out var toLocal)) return;

            ReadOnlySpan<Vector2> corners =
            [
                Vector2.Transform(new Vector2(bounds.Left, bounds.Top), toWorld),
                Vector2.Transform(new Vector2(bounds.Right, bounds.Top), toWorld),
                Vector2.Transform(new Vector2(bounds.Right, bounds.Bottom), toWorld),
                Vector2.Transform(new Vector2(bounds.Left, bounds.Bottom), toWorld),
            ];

            Vector2 gravity = parameters.Gravity;
            Vector2 down = gravity.LengthSquared() > 1e-6f ? Vector2.Normalize(gravity) : Vector2.UnitY;
            Vector2 side = new(-down.Y, down.X);
            Vector2 axis0 = EdgeAxis(corners[0], corners[1]);
            Vector2 axis1 = EdgeAxis(corners[1], corners[2]);

            Vector2 cornerMin = new(float.MaxValue), cornerMax = new(float.MinValue);
            foreach (var corner in corners)
            {
                cornerMin = Vector2.Min(cornerMin, corner);
                cornerMax = Vector2.Max(cornerMax, corner);
            }

            activeAreas.Add(new ActiveArea
            {
                Track = track,
                Definition = definition,
                Parameters = track.GetAreaParameters(local, parameters.Fps),
                ToLocal = toLocal,
                Bounds = bounds,
                Center = Vector2.Transform(bounds.Center, toWorld),
                CornerMin = cornerMin,
                CornerMax = cornerMax,
                Axis0 = axis0,
                Range0 = Project(corners, axis0),
                Axis1 = axis1,
                Range1 = Project(corners, axis1),
                Down = down,
                Side = side,
                DownRange = Project(corners, down),
                SideRange = Project(corners, side),
            });
        }

        private static Vector2 EdgeAxis(Vector2 from, Vector2 to)
        {
            Vector2 edge = to - from;
            Vector2 axis = new(-edge.Y, edge.X);
            return axis.LengthSquared() < 1e-8f ? Vector2.Zero : axis;
        }

        private static Vector2 Project(ReadOnlySpan<Vector2> points, Vector2 axis)
        {
            float min = float.MaxValue, max = float.MinValue;
            foreach (var point in points)
            {
                float d = Vector2.Dot(point, axis);
                min = MathF.Min(min, d);
                max = MathF.Max(max, d);
            }
            return new Vector2(min, max);
        }

        private static Vector2 Project(in Colliders.AABB box, Vector2 axis)
        {
            Vector2 center = (box.Min + box.Max) * 0.5f;
            Vector2 half = (box.Max - box.Min) * 0.5f;
            float c = Vector2.Dot(center, axis);
            float r = MathF.Abs(axis.X) * half.X + MathF.Abs(axis.Y) * half.Y;
            return new Vector2(c - r, c + r);
        }

        private static bool IsTarget(in AreaDefinition definition, BodyTrack owner)
        {
            string target = definition.TargetName;
            if (string.IsNullOrEmpty(target)) return true;

            string name = owner.Name;
            if (name == target) return true;
            if (target.EndsWith('/')) return name.StartsWith(target, StringComparison.Ordinal);
            return name.StartsWith(target + "/", StringComparison.Ordinal);
        }

        private static bool Contains(ActiveArea area, Vector2 point)
        {
            var local = Vector2.Transform(point, area.ToLocal);
            return local.X >= area.Bounds.Left && local.X <= area.Bounds.Right &&
                   local.Y >= area.Bounds.Top && local.Y <= area.Bounds.Bottom;
        }

        private void ApplyAreaForces(BodyTrack owner, RigidBody body, float deltaTime)
        {
            foreach (var area in activeAreas)
            {
                if (area.Definition.Kind == AreaKind.Trigger || !IsTarget(area.Definition, owner)) continue;

                if (area.Definition.Kind == AreaKind.Wind)
                {
                    if (Contains(area, body.Position)) ApplyWind(area, body, deltaTime);
                }
                else
                {
                    ApplyWater(area, body, deltaTime);
                }
            }
        }

        private void ApplyWind(ActiveArea area, RigidBody body, float deltaTime)
        {
            var p = area.Parameters;
            float turbulence = Math.Clamp(p.Turbulence, 0f, 1f);
            float gust = Noise(simulationTime, body.Position, 0f);
            float swirl = Noise(simulationTime, body.Position, 7.3f);

            float angle = p.FlowAngle * MathF.PI / 180f + turbulence * swirl * 0.7f;
            float speed = p.FlowSpeed * (1f + turbulence * gust * 0.8f);
            var direction = new Vector2(MathF.Cos(angle), MathF.Sin(angle));

            float along = Vector2.Dot(body.LinearVelocity, direction);
            float blend = 1f - MathF.Exp(-MathF.Max(p.Strength, 0f) * deltaTime);
            body.LinearVelocity += direction * ((speed - along) * blend);
        }

        private void ApplyWater(ActiveArea area, RigidBody body, float deltaTime)
        {
            var p = area.Parameters;

            float center = Vector2.Dot(body.Position, area.Side);
            if (center < area.SideRange.X || center > area.SideRange.Y) return;

            Vector2 bodyRange = Project(body.WorldAABB, area.Down);
            float height = MathF.Max(bodyRange.Y - bodyRange.X, 1f);
            float submerged = Math.Clamp((MathF.Min(bodyRange.Y, area.DownRange.Y) - MathF.Max(bodyRange.X, area.DownRange.X)) / height, 0f, 1f);
            if (submerged <= 0f) return;

            body.LinearVelocity -= parameters.Gravity * (MathF.Max(p.Buoyancy, 0f) * submerged * deltaTime);

            float angle = p.FlowAngle * MathF.PI / 180f;
            var flow = new Vector2(MathF.Cos(angle), MathF.Sin(angle)) * p.FlowSpeed;
            float decay = MathF.Exp(-MathF.Max(p.Drag, 0f) * submerged * deltaTime);
            body.LinearVelocity = flow + (body.LinearVelocity - flow) * decay;
            body.AngularVelocity *= MathF.Sqrt(decay);
        }

        private static float Noise(float time, Vector2 position, float seed)
        {
            return 0.6f * MathF.Sin(time * 2.3f + position.X * 0.011f + position.Y * 0.007f + seed)
                 + 0.4f * MathF.Sin(time * 5.1f - position.X * 0.023f + position.Y * 0.017f + 1.3f + seed * 1.7f);
        }

        private void ProcessTriggers()
        {
            foreach (var track in tracks)
            {
                if (!track.IsSpawned) continue;
                foreach (var body in track.Bodies)
                {
                    if (!body.IsGhost || body.Opacity <= 0f) continue;
                    body.Opacity = MathF.Max(0f, body.Opacity - body.FadeRate);
                }
            }

            foreach (var area in activeAreas)
            {
                if (area.Definition.Kind != AreaKind.Trigger) continue;

                long id = area.Track.Sequence;
                foreach (var track in tracks)
                {
                    if (!track.IsSpawned || track.IsArea || !IsTarget(area.Definition, track)) continue;

                    if (track.IsFluid && area.Definition.Action is TriggerAction.Vanish or TriggerAction.Warp)
                    {
                        if (track.Bodies.Count > 0 && FireFluidTrigger(area, track)) area.Fired++;
                        continue;
                    }

                    foreach (var body in track.Bodies)
                    {
                        if (body.IsSpawning) continue;

                        body.UpdateWorldAABB();
                        bool overlapping = !body.IsGhost && Overlaps(area, body.WorldAABB);
                        int index = body.InsideTriggers.IndexOf(id);

                        if (overlapping && index < 0)
                        {
                            body.InsideTriggers.Add(id);
                            Fire(area, body);
                            area.Fired++;
                        }
                        else if (!overlapping && index >= 0)
                        {
                            body.InsideTriggers.RemoveAt(index);
                        }
                    }
                }
            }
        }

        private void Fire(ActiveArea area, RigidBody body)
        {
            var p = area.Parameters;
            switch (area.Definition.Action)
            {
                case TriggerAction.Vanish:
                    body.IsGhost = true;
                    float frames = MathF.Max(p.FadeSeconds, 0f) * parameters.Fps;
                    body.FadeRate = frames >= 1f ? 1f / frames : 1f;
                    break;

                case TriggerAction.Freeze:
                    body.IsFrozen = true;
                    body.Type = BodyType.Static;
                    body.LinearVelocity = Vector2.Zero;
                    body.AngularVelocity = 0f;
                    break;

                case TriggerAction.Launch:
                    float angle = p.LaunchAngle * MathF.PI / 180f;
                    var velocity = new Vector2(MathF.Cos(angle), MathF.Sin(angle)) * p.LaunchSpeed;
                    body.LinearVelocity = area.Definition.KeepVelocity ? body.LinearVelocity + velocity : velocity;
                    break;

                case TriggerAction.Warp:
                    body.Position = area.Center + p.WarpOffset;
                    if (!area.Definition.KeepVelocity)
                    {
                        body.LinearVelocity = Vector2.Zero;
                        body.AngularVelocity = 0f;
                    }
                    body.UpdateWorldAABB();
                    break;
            }
        }

        private static bool Overlaps(ActiveArea area, in Colliders.AABB box)
        {
            if (area.CornerMax.X < box.Min.X || area.CornerMin.X > box.Max.X || area.CornerMax.Y < box.Min.Y || area.CornerMin.Y > box.Max.Y) return false;
            return OverlapsOnAxis(area.Axis0, area.Range0, box) && OverlapsOnAxis(area.Axis1, area.Range1, box);
        }

        private static bool OverlapsOnAxis(Vector2 axis, Vector2 range, in Colliders.AABB box)
        {
            if (axis == Vector2.Zero) return true;
            Vector2 boxRange = Project(box, axis);
            return range.Y >= boxRange.X && boxRange.Y >= range.X;
        }

        public bool TryGetTriggerFrame(string name, int frame, bool wait, out int fireFrame)
        {
            fireFrame = 0;
            if (string.IsNullOrEmpty(name)) return false;

            bool entered = false;
            if (wait) Monitor.Enter(syncRoot, ref entered);
            else entered = Monitor.TryEnter(syncRoot);
            if (!entered) return false;

            try
            {
                if (wait) PullSettings();
                if (wait && hasParameters && frame > validUntil) EnsureSimulated(frame, false);

                bool found = false;
                foreach (var track in tracks)
                {
                    if (track.Area is not { Kind: AreaKind.Trigger } || track.Name != name || !track.IsInitialized) continue;

                    int first = track.FirstTriggerLocal;
                    if (first == int.MaxValue || first > frame - track.StartFrame) continue;

                    int absolute = track.StartFrame + first;
                    if (!found || absolute < fireFrame)
                    {
                        fireFrame = absolute;
                        found = true;
                    }
                }
                return found;
            }
            finally
            {
                Monitor.Exit(syncRoot);
            }
        }
    }
}
