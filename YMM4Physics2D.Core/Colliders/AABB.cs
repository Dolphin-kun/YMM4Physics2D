using System.Numerics;

namespace YMM4Physics2D.Core.Colliders
{
    public struct AABB(Vector2 min, Vector2 max)
    {
        public Vector2 Min = min;
        public Vector2 Max = max;

        public readonly bool Overlaps(in AABB other)
        {
            bool noOverlap = Max.X < other.Min.X || Min.X > other.Max.X ||
                             Max.Y < other.Min.Y || Min.Y > other.Max.Y;

            return !noOverlap;
        }
    }
}
