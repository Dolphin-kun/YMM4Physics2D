using System.Numerics;

namespace YMM4Physics2D.Core.Math
{
    public static class GeometryUtils
    {
        public static float CalculateSignedArea(IList<Vector2> vertices)
        {
            if (vertices.Count < 3) return 0f;
            float area = 0f;
            int count = vertices.Count;
            for (int i = 0; i < count; i++)
            {
                int j = (i + 1) % count;
                area += (vertices[i].X * vertices[j].Y) - (vertices[j].X * vertices[i].Y);
            }
            return area * 0.5f;
        }

        public static float CalculateArea(IList<Vector2> vertices)
            => System.Math.Abs(CalculateSignedArea(vertices));

        public static bool IsCounterClockwise(IList<Vector2> vertices)
            => CalculateSignedArea(vertices) > 0;

        public static bool IsPointInPolygon(Vector2 point, IList<Vector2> polygon)
        {
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                if (((polygon[i].Y > point.Y) != (polygon[j].Y > point.Y)) &&
                    (point.X < (polygon[j].X - polygon[i].X) * (point.Y - polygon[i].Y) / (polygon[j].Y - polygon[i].Y) + polygon[i].X))
                {
                    inside = !inside;
                }
            }
            return inside;
        }

        public static Vector2 GetPolygonCenter(Vector2[] vertices)
        {
            Vector2 center = Vector2.Zero;

            for (int i = 0; i < vertices.Length; i++)
            {
                center += vertices[i];
            }

            return center / vertices.Length;
        }

        public static Vector2 CalculateCentroid(Vector2[] vs)
        {
            Vector2 c = Vector2.Zero;
            float area = 0f;
            float inv3 = 1.0f / 3.0f;

            for (int i = 0; i < vs.Length; i++)
            {
                Vector2 p1 = vs[i];
                Vector2 p2 = vs[(i + 1) % vs.Length];

                float d = p1.X * p2.Y - p1.Y * p2.X;

                float triangleArea = 0.5f * d;
                area += triangleArea;

                c += (p1 + p2) * inv3 * triangleArea;
            }

            return MathF.Abs(area) > 1e-6f ? c / area : Vector2.Zero;
        }
    }
}
