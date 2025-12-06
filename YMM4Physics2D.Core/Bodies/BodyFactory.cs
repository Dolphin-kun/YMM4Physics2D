using System.Numerics;
using YMM4Physics2D.Core.Colliders;

namespace YMM4Physics2D.Core.Bodies
{
    public static class BodyFactory
    {
        public static List<RigidBody> CreateBodies(List<Vector2[]> partsVertices, Vector2 position, float mass, bool isSeparate, BodyType bodyType = BodyType.Dynamic)
        {
            var bodies = new List<RigidBody>();

            if (partsVertices == null || partsVertices.Count == 0) return bodies;

            if (isSeparate)
            {
                float massPerBody = mass / partsVertices.Count;

                foreach (var vertices in partsVertices)
                {
                    Vector2 centroid = CalculateCentroid(vertices);
                    Vector2 bodyWorldPosition = position + centroid;
                    RigidBody body = new(bodyWorldPosition, massPerBody, bodyType);

                    body.VisualOffset = centroid;

                    Vector2[] centeredVertices = [.. vertices.Select(v => v - centroid)];

                    body.AddCollider(new PolygonCollider(centeredVertices));
                    bodies.Add(body);
                }
            }
            else
            {
                var body = new RigidBody(position, mass, bodyType);
                body.VisualOffset = Vector2.Zero;

                foreach (var vertices in partsVertices)
                {
                    body.AddCollider(new PolygonCollider(vertices));
                }

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
    }
}
