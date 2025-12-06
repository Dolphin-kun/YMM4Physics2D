using System.Drawing;
using System.Numerics;

namespace YMM4Physics2D.Core.World
{
    public class WorldConfig
    {
        public int Id { get; set; }

        public Vector2 Gravity { get; set; } = new Vector2(0f, 980.0f);
        public float LinearDamping { get; set; } = 0.05f;
        public float AngularDamping { get; set; } = 3.0f;
        
        public int SnapshotInterval { get; set; } = 60;
        public int Iterations { get; set; } = 4;

        public float MinVelocityForRestitution { get; set; } = 1.0f;
        public float PositionCorrectionPercent { get; set; } = 0.4f;
        public float PositionCorrectionSlop { get; set; } = 0.01f;

        public bool EnableWalls { get; set; } = false;
        public Size ScreenSize { get; set; }
    }
}
