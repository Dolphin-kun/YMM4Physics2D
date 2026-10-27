using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Geometry;

namespace YMM4Physics2D.Core.Colliders
{
    public class PolygonCollider : Collider
    {
        private Vector2[] worldVertices = [];
        private Vector2[] worldNormals = [];
        private Vector2[] localNormals = [];

        public Vector2[] LocalVertices { get; private set; } = [];

        public Vector2[] WorldVertices
        {
            get
            {
                EnsureUpdated();
                return worldVertices;
            }
        }

        public Vector2[] WorldNormals
        {
            get
            {
                EnsureUpdated();
                return worldNormals;
            }
        }

        public PolygonCollider(Vector2[] vertices, bool autoCenter = true)
        {
            SetVertices(vertices, autoCenter);
        }

        public void SetVertices(Vector2[] vertices, bool autoCenter = true)
        {
            var points = new List<Vector2>(vertices.Length);
            foreach (var vertex in vertices)
            {
                if (points.Count == 0 || Vector2.DistanceSquared(points[^1], vertex) > 1e-8f)
                {
                    points.Add(vertex);
                }
            }
            if (points.Count > 1 && Vector2.DistanceSquared(points[0], points[^1]) <= 1e-8f)
            {
                points.RemoveAt(points.Count - 1);
            }
            if (points.Count < 3) return;

            if (autoCenter)
            {
                Vector2 centroid = GeometryUtils.CalculateCentroid([.. points]);
                for (int i = 0; i < points.Count; i++)
                {
                    points[i] -= centroid;
                }
            }

            LocalVertices = [.. points];

            if (GeometryUtils.CalculateSignedArea(LocalVertices) < 0)
            {
                Array.Reverse(LocalVertices);
            }

            int count = LocalVertices.Length;
            worldVertices = new Vector2[count];
            localNormals = new Vector2[count];
            worldNormals = new Vector2[count];

            for (int i = 0; i < count; i++)
            {
                Vector2 edge = LocalVertices[(i + 1) % count] - LocalVertices[i];
                localNormals[i] = Vector2.Normalize(new Vector2(edge.Y, -edge.X));
            }

            MarkDirty();
        }

        public override float MinWidth
        {
            get
            {
                float width = float.MaxValue;
                for (int i = 0; i < localNormals.Length; i++)
                {
                    float offset = Vector2.Dot(localNormals[i], LocalVertices[i]);
                    float depth = 0f;
                    foreach (var vertex in LocalVertices)
                    {
                        depth = MathF.Max(depth, offset - Vector2.Dot(localNormals[i], vertex));
                    }
                    width = MathF.Min(width, depth);
                }
                return width == float.MaxValue ? 0f : width;
            }
        }

        public override float CalculateArea()
        {
            return MathF.Abs(GeometryUtils.CalculateSignedArea(LocalVertices));
        }

        public override float CalculateInertia(float mass)
        {
            if (LocalVertices.Length == 0) return 0f;

            float numerator = 0f;
            float denominator = 0f;

            for (int i = 0; i < LocalVertices.Length; i++)
            {
                Vector2 p1 = LocalVertices[i];
                Vector2 p2 = LocalVertices[(i + 1) % LocalVertices.Length];

                float cross = p1.X * p2.Y - p1.Y * p2.X;

                float dotSum = (p1.X * p1.X + p1.Y * p1.Y) +
                               (p1.X * p2.X + p1.Y * p2.Y) +
                               (p2.X * p2.X + p2.Y * p2.Y);

                numerator += cross * dotSum;
                denominator += cross;
            }

            return denominator > 1e-6f ? (mass / 6.0f) * (numerator / denominator) : 0f;
        }

        protected override AABB ComputeLocalBounds()
        {
            if (LocalVertices.Length == 0) return new AABB(Offset, Offset);

            Vector2 min = LocalVertices[0] + Offset;
            Vector2 max = min;
            for (int i = 1; i < LocalVertices.Length; i++)
            {
                Vector2 v = LocalVertices[i] + Offset;
                min = Vector2.Min(min, v);
                max = Vector2.Max(max, v);
            }
            return new AABB(min, max);
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        protected override AABB UpdateWorld(in Matrix3x2 transform)
        {
            if (LocalVertices.Length == 0) return new AABB(transform.Translation, transform.Translation);

            Vector2 min = new(float.MaxValue);
            Vector2 max = new(float.MinValue);
            for (int i = 0; i < LocalVertices.Length; i++)
            {
                Vector2 v = Vector2.Transform(LocalVertices[i] + Offset, transform);
                worldVertices[i] = v;
                worldNormals[i] = Vector2.TransformNormal(localNormals[i], transform);
                min = Vector2.Min(min, v);
                max = Vector2.Max(max, v);
            }
            return new AABB(min, max);
        }
    }
}
