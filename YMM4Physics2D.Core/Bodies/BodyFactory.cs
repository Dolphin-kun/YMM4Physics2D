using System.Numerics;
using YMM4Physics2D.Core.Colliders;
using YMM4Physics2D.Core.Geometry;
using YMM4Physics2D.Core.Shapes;

namespace YMM4Physics2D.Core.Bodies
{
    public static class BodyFactory
    {
        private const float CircleIsoperimetricQuotient = 0.975f;

        private const int CircleMinVertices = 10;

        private const float MinCircleRadius = 0.5f;

        private const float MaxMiter = 2f;

        public static List<RigidBody> CreateBodies(ShapeVersion shape, Matrix3x2 imageTransform, bool isSeparate, BodyType bodyType, float mass, float margin = 0f)
        {
            var bodies = new List<RigidBody>();
            if (shape.IsEmpty) return bodies;

            if (isSeparate)
            {
                float massPerBody = mass / shape.Components.Length;
                foreach (var component in shape.Components)
                {
                    bodies.Add(CreateBody([component], imageTransform, bodyType, massPerBody, margin));
                }
            }
            else
            {
                bodies.Add(CreateBody(shape.Components, imageTransform, bodyType, mass, margin));
            }

            return bodies;
        }

        private static RigidBody CreateBody(IReadOnlyList<ShapeComponent> components, Matrix3x2 imageTransform, BodyType bodyType, float mass, float margin)
        {
            float totalArea = 0f;
            Vector2 weighted = Vector2.Zero;
            foreach (var component in components)
            {
                totalArea += component.Area;
                weighted += component.Centroid * component.Area;
            }

            Vector2 imageCentroid = totalArea > 0f ? weighted / totalArea : Vector2.Zero;
            Vector2 localCentroid = Vector2.Transform(imageCentroid, imageTransform);
            bool mirrored = imageTransform.GetDeterminant() < 0f;

            var body = new RigidBody(Vector2.Zero, mass, bodyType)
            {
                ImageCentroid = imageCentroid,
                LocalCentroid = localCentroid,
            };

            var colliders = new List<Collider>();
            foreach (var component in components)
            {
                foreach (var part in component.ConvexParts)
                {
                    var vertices = new Vector2[part.Length];
                    for (int i = 0; i < part.Length; i++)
                    {
                        vertices[i] = Vector2.Transform(part[i], imageTransform) - localCentroid;
                    }
                    if (mirrored) Array.Reverse(vertices);

                    if (TryGetCircleParams(vertices, out float radius, out Vector2 offset))
                    {
                        colliders.Add(new CircleCollider(MathF.Max(MinCircleRadius, radius + margin), offset));
                    }
                    else
                    {
                        var polygon = new PolygonCollider(OffsetConvex(vertices, margin), false);
                        if (polygon.LocalVertices.Length >= 3) colliders.Add(polygon);
                    }
                }
            }

            body.AddColliders(colliders);
            return body;
        }

        private static Vector2[] OffsetConvex(Vector2[] vertices, float margin)
        {
            int count = vertices.Length;
            if (margin == 0f || count < 3) return vertices;

            float area = GeometryUtils.CalculateSignedArea(vertices);
            if (MathF.Abs(area) < 1e-4f) return vertices;
            float orientation = area > 0f ? 1f : -1f;

            var result = new Vector2[count];
            for (int i = 0; i < count; i++)
            {
                Vector2 previous = vertices[(i + count - 1) % count];
                Vector2 current = vertices[i];
                Vector2 next = vertices[(i + 1) % count];
                Vector2 normalA = EdgeNormal(previous, current, orientation);
                Vector2 normalB = EdgeNormal(current, next, orientation);
                if (normalA == Vector2.Zero) normalA = normalB;
                if (normalB == Vector2.Zero) normalB = normalA;

                float denominator = 1f + Vector2.Dot(normalA, normalB);
                Vector2 miter = denominator > 1e-3f ? (normalA + normalB) / denominator : normalA;
                float length = miter.Length();
                if (length > MaxMiter) miter *= MaxMiter / length;
                result[i] = current + miter * margin;
            }

            if (margin < 0f)
            {
                for (int i = 0; i < count; i++)
                {
                    Vector2 before = vertices[(i + 1) % count] - vertices[i];
                    Vector2 after = result[(i + 1) % count] - result[i];
                    if (Vector2.Dot(before, after) <= 0f) return vertices;
                }
                if (GeometryUtils.CalculateSignedArea(result) * orientation <= 1e-4f) return vertices;
            }

            return result;
        }

        private static Vector2 EdgeNormal(Vector2 from, Vector2 to, float orientation)
        {
            Vector2 edge = to - from;
            float length = edge.Length();
            return length < 1e-6f ? Vector2.Zero : new Vector2(edge.Y, -edge.X) * (orientation / length);
        }

        private static bool TryGetCircleParams(Vector2[] vertices, out float radius, out Vector2 offset)
        {
            radius = 0f;
            offset = Vector2.Zero;

            if (vertices.Length < CircleMinVertices) return false;

            float area = GeometryUtils.CalculateArea(vertices);
            if (area <= 0f) return false;

            float perimeter = 0f;
            for (int i = 0; i < vertices.Length; i++)
            {
                perimeter += Vector2.Distance(vertices[i], vertices[(i + 1) % vertices.Length]);
            }

            float quotient = 4f * MathF.PI * area / (perimeter * perimeter);
            if (quotient < CircleIsoperimetricQuotient) return false;

            radius = MathF.Sqrt(area / MathF.PI);
            offset = GeometryUtils.CalculateCentroid(vertices);
            return true;
        }
    }
}
