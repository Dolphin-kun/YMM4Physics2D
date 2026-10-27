using System.Numerics;

namespace YMM4Physics2D.Core.World
{
    public class WorldConfig
    {
        public int Id { get; set; }

        public Vector2 Gravity { get; set; } = new Vector2(0f, 980.0f);

        public float GravityAngle { get; set; } = 0f;
        public int SnapshotInterval { get; set; } = 30;
        public int Iterations { get; set; } = 4;

        public float MinVelocityForRestitution { get; set; } = 1.0f;
        public float PositionCorrectionPercent { get; set; } = 0.4f;
        public float PositionCorrectionSlop { get; set; } = 0.01f;

        public bool EnableWalls { get; set; } = false;

        public WorldConfig Clone() => (WorldConfig)MemberwiseClone();
    }
}
