using System.Numerics;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Shapes;

namespace YMM4Physics2D.Core.World
{
    internal struct BodyState(RigidBody body)
    {
        public Vector2 Position = body.Position;
        public float Rotation = body.Rotation;
        public Vector2 LinearVelocity = body.LinearVelocity;
        public float AngularVelocity = body.AngularVelocity;
        public float Opacity = body.Opacity;
        public float Presence = body.Presence;
        public float FadeRate = body.FadeRate;
        public bool IsGhost = body.IsGhost;
        public bool IsFrozen = body.IsFrozen;
        public long[] InsideTriggers = body.InsideTriggers.Count == 0 ? [] : [.. body.InsideTriggers];

        public readonly void ApplyTo(RigidBody body)
        {
            body.Position = Position;
            body.Rotation = Rotation;
            body.LinearVelocity = LinearVelocity;
            body.AngularVelocity = AngularVelocity;
            body.Opacity = Opacity;
            body.Presence = Presence;
            body.FadeRate = FadeRate;
            body.IsGhost = IsGhost;
            body.IsFrozen = IsFrozen;
            body.InsideTriggers.Clear();
            body.InsideTriggers.AddRange(InsideTriggers);
            body.ClearForces();
        }
    }

    internal sealed class TrackState
    {
        public ShapeVersion Shape { get; set; } = ShapeVersion.Empty;
        public Matrix3x2 Transform { get; set; }
        public BodyState[] Bodies { get; set; } = [];
        public FluidShape? FluidShape { get; set; }
    }

    internal sealed class WorldSnapshot
    {
        public int Frame { get; set; }
        public Dictionary<BodyTrack, TrackState> Tracks { get; } = [];
    }
}
