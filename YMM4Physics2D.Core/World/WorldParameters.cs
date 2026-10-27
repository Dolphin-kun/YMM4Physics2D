using System.Drawing;
using System.Numerics;

namespace YMM4Physics2D.Core.World
{
    public readonly record struct WorldParameters(
        Vector2 Gravity,
        int Iterations,
        int SnapshotInterval,
        float MinVelocityForRestitution,
        float PositionCorrectionPercent,
        float PositionCorrectionSlop,
        bool EnableWalls,
        Size ScreenSize,
        int Fps)
    {
        public static WorldParameters Create(WorldConfig? config, Size screenSize, int fps)
        {
            config ??= new WorldConfig();
            return new WorldParameters(
                config.Gravity,
                Math.Max(1, config.Iterations),
                Math.Max(1, config.SnapshotInterval),
                config.MinVelocityForRestitution,
                config.PositionCorrectionPercent,
                config.PositionCorrectionSlop,
                config.EnableWalls,
                screenSize,
                Math.Max(1, fps));
        }
    }
}
