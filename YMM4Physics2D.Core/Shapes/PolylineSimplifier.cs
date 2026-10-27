using System.Numerics;

namespace YMM4Physics2D.Core.Shapes
{
    internal static class PolylineSimplifier
    {
        public static List<Vector2> SimplifyClosed(List<Vector2> points, float tolerance)
        {
            int count = points.Count;
            if (count < 4 || tolerance <= 0f) return RemoveCollinear(points);

            int farthest = 0;
            float farthestDistance = -1f;
            for (int i = 1; i < count; i++)
            {
                float distance = Vector2.DistanceSquared(points[0], points[i]);
                if (distance > farthestDistance)
                {
                    farthestDistance = distance;
                    farthest = i;
                }
            }

            var keep = new bool[count];
            keep[0] = true;
            keep[farthest] = true;

            float toleranceSquared = tolerance * tolerance;
            var stack = new Stack<(int First, int Last)>();
            stack.Push((0, farthest));
            stack.Push((farthest, count));

            while (stack.Count > 0)
            {
                var (first, last) = stack.Pop();
                if (last - first < 2) continue;

                Vector2 start = points[first % count];
                Vector2 end = points[last % count];

                int index = -1;
                float maxDistance = toleranceSquared;
                for (int i = first + 1; i < last; i++)
                {
                    float distance = DistanceToSegmentSquared(points[i % count], start, end);
                    if (distance > maxDistance)
                    {
                        maxDistance = distance;
                        index = i;
                    }
                }

                if (index < 0) continue;

                keep[index % count] = true;
                stack.Push((first, index));
                stack.Push((index, last));
            }

            var result = new List<Vector2>();
            for (int i = 0; i < count; i++)
            {
                if (keep[i]) result.Add(points[i]);
            }

            return RemoveCollinear(result);
        }

        public static List<Vector2> RemoveCollinear(List<Vector2> points)
        {
            var result = new List<Vector2>(points);
            bool removed = true;
            while (removed && result.Count >= 3)
            {
                removed = false;
                for (int i = 0; i < result.Count && result.Count >= 3; i++)
                {
                    Vector2 previous = result[(i - 1 + result.Count) % result.Count];
                    Vector2 current = result[i];
                    Vector2 next = result[(i + 1) % result.Count];

                    float cross = (current.X - previous.X) * (next.Y - current.Y) - (current.Y - previous.Y) * (next.X - current.X);
                    if (current == previous || MathF.Abs(cross) < 1e-6f)
                    {
                        result.RemoveAt(i);
                        removed = true;
                        i--;
                    }
                }
            }
            return result;
        }

        private static float DistanceToSegmentSquared(Vector2 point, Vector2 start, Vector2 end)
        {
            Vector2 segment = end - start;
            float lengthSquared = segment.LengthSquared();
            if (lengthSquared < 1e-12f) return Vector2.DistanceSquared(point, start);

            float t = Math.Clamp(Vector2.Dot(point - start, segment) / lengthSquared, 0f, 1f);
            return Vector2.DistanceSquared(point, start + segment * t);
        }
    }
}
