using System.Numerics;

namespace YMM4Physics2D.Core.Colliders
{
    public struct AABB
    {
        public Vector2 Min;
        public Vector2 Max;

        public AABB(Vector2 min, Vector2 max)
        {
            Min = min;
            Max = max;
        }

        public readonly float Width => Max.X - Min.X;
        public readonly float Height => Max.Y - Min.Y;
        public readonly Vector2 Center => (Min + Max) * 0.5f;

        public readonly bool Overlaps(in AABB other)
        {
            bool noOverlap = Max.X < other.Min.X || Min.X > other.Max.X ||
                             Max.Y < other.Min.Y || Min.Y > other.Max.Y;

            return !noOverlap;
        }

        public readonly bool Contains(Vector2 point)
        {
            return point.X >= Min.X && point.X <= Max.X &&
                   point.Y >= Min.Y && point.Y <= Max.Y;
        }
    }
}
