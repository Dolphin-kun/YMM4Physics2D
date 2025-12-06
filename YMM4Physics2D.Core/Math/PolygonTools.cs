using System.Numerics;

namespace YMM4Physics2D.Core.Math
{
    public static class PolygonTools
    {
        public static List<Vector2> CreateConvexHull(List<Vector2> vertices)
        {
            if (vertices.Count < 3) return [.. vertices];

            var sorted = vertices.OrderBy(v => v.X).ThenBy(v => v.Y).ToList();

            List<Vector2> upper = [];
            List<Vector2> lower = [];

            foreach (var p in sorted)
            {
                while (lower.Count >= 2 && Cross(lower[^2], lower[^1], p) <= 0)
                {
                    lower.RemoveAt(lower.Count - 1);
                }
                lower.Add(p);
            }

            for (int i = sorted.Count - 1; i >= 0; i--)
            {
                var p = sorted[i];
                while (upper.Count >= 2 && Cross(upper[^2], upper[^1], p) <= 0)
                {
                    upper.RemoveAt(upper.Count - 1);
                }
                upper.Add(p);
            }

            lower.RemoveAt(lower.Count - 1);
            upper.RemoveAt(upper.Count - 1);

            return [.. lower, .. upper];
        }

        private static float Cross(Vector2 o, Vector2 a, Vector2 b)
        {
            return (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
        }

        public static List<List<Vector2>> DecomposeConcave(List<Vector2> vertices)
        {
            return BayazitDecomposer.Decompose(vertices);
        }

        public static List<Vector2> Simplify(List<Vector2> vertices, float tolerance = 1.0f)
        {
            if (vertices.Count < 3) return vertices;

            List<Vector2> simplified = [];
            simplified.Add(vertices[0]);

            for (int i = 1; i < vertices.Count; i++)
            {
                if (Vector2.DistanceSquared(vertices[i], simplified.Last()) > tolerance * tolerance)
                {
                    simplified.Add(vertices[i]);
                }
            }

            return simplified;
        }
    }
}
