using System.Numerics;
using YMM4Physics2D.Core.Colliders;

namespace YMM4Physics2D.Core.Bodies
{
    public static class BodyFactory
    {
        private const float CircleTolerance = 0.15f;

        public static List<RigidBody> CreateBodies(List<List<Vector2[]>> groupedParts, Vector2 position, float mass, bool isSeparate, BodyType bodyType = BodyType.Dynamic)
        {
            var bodies = new List<RigidBody>();

            if (groupedParts == null || groupedParts.Count == 0) return bodies;

            if (isSeparate)
            {
                float massPerBody = mass / groupedParts.Count;

                foreach (var shapeParts in groupedParts)
                {
                    var allVerticesInShape = shapeParts.SelectMany(v => v).ToArray();
                    Vector2 shapeCentroid = CalculateCentroid(allVerticesInShape);

                    Vector2 bodyWorldPosition = position + shapeCentroid;

                    RigidBody body = new(bodyWorldPosition, massPerBody, bodyType)
                    {
                        VisualOffset = shapeCentroid
                    };

                    var collidersToAdd = new List<Collider>(shapeParts.Count);
                    foreach (var vertices in shapeParts)
                    {
                        Vector2[] centeredVertices = new Vector2[vertices.Length];
                        for (int i = 0; i < vertices.Length; i++)
                        {
                            centeredVertices[i] = vertices[i] - shapeCentroid;
                        }

                        if (TryGetCircleParams(centeredVertices, out float radius, out Vector2 offset))
                        {
                            collidersToAdd.Add(new CircleCollider(radius, offset));
                        }
                        else
                        {
                            collidersToAdd.Add(new PolygonCollider(centeredVertices, false));
                        }
                    }

                    body.AddColliders(collidersToAdd);
                    bodies.Add(body);
                }
            }
            else
            {
                var allVertices = groupedParts.SelectMany(g => g).SelectMany(v => v).ToArray();
                Vector2 commonCenter = CalculateCentroid(allVertices);

                var body = new RigidBody(position, mass, bodyType)
                {
                    VisualOffset = commonCenter
                };

                int totalParts = groupedParts.Sum(g => g.Count);
                var collidersToAdd = new List<Collider>(totalParts);
                foreach (var shapeParts in groupedParts)
                {
                    foreach (var vertices in shapeParts)
                    {
                        Vector2[] centeredVertices = new Vector2[vertices.Length];
                        for (int i = 0; i < vertices.Length; i++)
                        {
                            centeredVertices[i] = vertices[i] - commonCenter;
                        }

                        if (TryGetCircleParams(centeredVertices, out float radius, out Vector2 offset))
                        {
                            collidersToAdd.Add(new CircleCollider(radius, offset));
                        }
                        else
                        {
                            collidersToAdd.Add(new PolygonCollider(centeredVertices, false));
                        }
                    }
                }

                body.AddColliders(collidersToAdd);
                bodies.Add(body);
            }

            return bodies;
        }

        private static Vector2 CalculateCentroid(Vector2[] vertices)
        {
            if (vertices == null || vertices.Length == 0) return Vector2.Zero;

            Vector2 sum = Vector2.Zero;
            foreach(var v in vertices)
            {
                sum += v;
            }
            return sum / vertices.Length;
        }

        private static bool TryGetCircleParams(Vector2[] vertices, out float radius, out Vector2 offset)
        {
            radius = 0f;
            offset = Vector2.Zero;

            if (vertices.Length < 5) return false;

            Vector2 center = CalculateCentroid(vertices);

            float minR = float.MaxValue;
            float maxR = float.MinValue;
            float sumR = 0f;

            foreach (var v in vertices)
            {
                float d = Vector2.Distance(v, center);
                if (d < minR) minR = d;
                if (d > maxR) maxR = d;
                sumR += d;
            }

            float avgR = sumR / vertices.Length;

            float difference = maxR - minR;
            if (avgR > 0 && difference / avgR < CircleTolerance)
            {
                radius = avgR;
                offset = center;
                return true;
            }

            return false;
        }
    }
}
