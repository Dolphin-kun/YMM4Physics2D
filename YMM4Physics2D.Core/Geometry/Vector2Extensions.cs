using System.Numerics;

namespace YMM4Physics2D.Core.Geometry
{
    public static class Vector2Extensions
    {
        public static Vector2 Rotate(this Vector2 v, float radians)
        {
            var (sin, cos) = MathF.SinCos(radians);

            return new Vector2(
                v.X * cos - v.Y * sin,
                v.X * sin + v.Y * cos
            );
        }

        public static float Cross(this Vector2 a, Vector2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        public static Vector2 Cross(float s, Vector2 a)
        {
            return new Vector2(-s * a.Y, s * a.X);
        }
    }
}
