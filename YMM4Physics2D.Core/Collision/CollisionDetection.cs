using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Colliders;

namespace YMM4Physics2D.Core.Collision
{
    public static class CollisionDetection
    {
        [ThreadStatic]
        private static List<Collider>? candidatesA;

        [ThreadStatic]
        private static List<Collider>? candidatesB;

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public static void Detect(RigidBody bodyA, RigidBody bodyB, List<Manifold> manifolds)
        {
            var listA = candidatesA ??= [];
            var listB = candidatesB ??= [];
            listA.Clear();
            listB.Clear();

            AABB boundsA = bodyA.WorldAABB;
            AABB boundsB = bodyB.WorldAABB;
            foreach (var collider in bodyA.Colliders)
            {
                if (collider.WorldAABB.Overlaps(boundsB)) listA.Add(collider);
            }
            if (listA.Count == 0) return;

            foreach (var collider in bodyB.Colliders)
            {
                if (collider.WorldAABB.Overlaps(boundsA)) listB.Add(collider);
            }

            foreach (var colA in listA)
            {
                foreach (var colB in listB)
                {
                    if (!colA.WorldAABB.Overlaps(colB.WorldAABB)) continue;

                    var manifold = new Manifold(bodyA, bodyB);
                    bool hasCollision;

                    if (colA is CircleCollider circleA)
                    {
                        hasCollision = colB is CircleCollider circleB
                            ? DetectCircleCircle(circleA, circleB, ref manifold)
                            : DetectPolygonCircle(colB, circleA, ref manifold, flipOwner: true);
                    }
                    else if (colB is CircleCollider circleB)
                    {
                        hasCollision = DetectPolygonCircle(colA, circleB, ref manifold, flipOwner: false);
                    }
                    else
                    {
                        hasCollision = DetectConvex(colA, colB, ref manifold);
                    }

                    if (hasCollision)
                    {
                        manifolds.Add(manifold);
                    }
                }
            }
        }

        private static bool TryGetConvex(Collider collider, out Vector2[] vertices, out Vector2[] normals)
        {
            switch (collider)
            {
                case PolygonCollider polygon:
                    vertices = polygon.WorldVertices;
                    normals = polygon.WorldNormals;
                    return vertices.Length >= 3;
                case BoxCollider box:
                    vertices = box.WorldVertices;
                    normals = box.WorldNormals;
                    return true;
                default:
                    vertices = [];
                    normals = [];
                    return false;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static bool DetectConvex(Collider colA, Collider colB, ref Manifold manifold)
        {
            if (!TryGetConvex(colA, out var verticesA, out var normalsA)) return false;
            if (!TryGetConvex(colB, out var verticesB, out var normalsB)) return false;

            float separationA = FindMaxSeparation(verticesA, normalsA, verticesB, out int edgeA);
            if (separationA > 0f) return false;

            float separationB = FindMaxSeparation(verticesB, normalsB, verticesA, out int edgeB);
            if (separationB > 0f) return false;

            Vector2[] referenceVertices, incidentVertices, incidentNormals;
            Vector2 referenceNormal;
            int referenceEdge;
            bool flip;

            if (separationB > separationA + 0.01f)
            {
                referenceVertices = verticesB;
                referenceNormal = normalsB[edgeB];
                referenceEdge = edgeB;
                incidentVertices = verticesA;
                incidentNormals = normalsA;
                flip = true;
            }
            else
            {
                referenceVertices = verticesA;
                referenceNormal = normalsA[edgeA];
                referenceEdge = edgeA;
                incidentVertices = verticesB;
                incidentNormals = normalsB;
                flip = false;
            }

            int incidentEdge = 0;
            float minDot = float.MaxValue;
            for (int i = 0; i < incidentNormals.Length; i++)
            {
                float dot = Vector2.Dot(referenceNormal, incidentNormals[i]);
                if (dot < minDot)
                {
                    minDot = dot;
                    incidentEdge = i;
                }
            }

            Span<Vector2> incident = [incidentVertices[incidentEdge], incidentVertices[(incidentEdge + 1) % incidentVertices.Length]];

            Vector2 v1 = referenceVertices[referenceEdge];
            Vector2 v2 = referenceVertices[(referenceEdge + 1) % referenceVertices.Length];
            Vector2 edge = v2 - v1;
            float edgeLength = edge.Length();
            if (edgeLength < 1e-6f) return false;
            Vector2 tangent = edge / edgeLength;

            Span<Vector2> clipped1 = stackalloc Vector2[2];
            if (ClipSegment(incident, clipped1, -tangent, -Vector2.Dot(tangent, v1)) < 2) return false;

            Span<Vector2> clipped2 = stackalloc Vector2[2];
            if (ClipSegment(clipped1, clipped2, tangent, Vector2.Dot(tangent, v2)) < 2) return false;

            float referenceOffset = Vector2.Dot(referenceNormal, v1);
            float maxDepth = 0f;
            for (int i = 0; i < 2; i++)
            {
                float separation = Vector2.Dot(referenceNormal, clipped2[i]) - referenceOffset;
                if (separation <= 0f)
                {
                    manifold.AddContact(clipped2[i]);
                    maxDepth = MathF.Max(maxDepth, -separation);
                }
            }

            if (manifold.ContactCount == 0) return false;

            manifold.Depth = maxDepth;
            manifold.Normal = flip ? -referenceNormal : referenceNormal;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static float FindMaxSeparation(Vector2[] verticesA, Vector2[] normalsA, Vector2[] verticesB, out int edgeIndex)
        {
            float maxSeparation = float.MinValue;
            edgeIndex = 0;

            for (int i = 0; i < verticesA.Length; i++)
            {
                Vector2 normal = normalsA[i];
                Vector2 vertex = verticesA[i];

                float offset = Vector2.Dot(normal, vertex);
                float minSeparation = float.MaxValue;
                foreach (var vertexB in verticesB)
                {
                    minSeparation = MathF.Min(minSeparation, Vector2.Dot(normal, vertexB) - offset);
                }

                if (minSeparation > 0f)
                {
                    edgeIndex = i;
                    return minSeparation;
                }

                if (minSeparation > maxSeparation)
                {
                    maxSeparation = minSeparation;
                    edgeIndex = i;
                }
            }

            return maxSeparation;
        }

        private static int ClipSegment(ReadOnlySpan<Vector2> input, Span<Vector2> output, Vector2 normal, float offset)
        {
            int count = 0;
            float distance0 = Vector2.Dot(normal, input[0]) - offset;
            float distance1 = Vector2.Dot(normal, input[1]) - offset;

            if (distance0 <= 0f) output[count++] = input[0];
            if (distance1 <= 0f) output[count++] = input[1];

            if (distance0 * distance1 < 0f && count < 2)
            {
                float t = distance0 / (distance0 - distance1);
                output[count++] = input[0] + t * (input[1] - input[0]);
            }

            return count;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static bool DetectPolygonCircle(Collider polyCol, CircleCollider circleCol, ref Manifold manifold, bool flipOwner)
        {
            if (!TryGetConvex(polyCol, out var vertices, out var normals)) return false;

            Vector2 center = circleCol.WorldCenter;
            float radius = circleCol.Radius;

            float separation = float.MinValue;
            int edge = 0;
            for (int i = 0; i < vertices.Length; i++)
            {
                float s = Vector2.Dot(normals[i], center - vertices[i]);
                if (s > radius) return false;
                if (s > separation)
                {
                    separation = s;
                    edge = i;
                }
            }

            Vector2 v1 = vertices[edge];
            Vector2 v2 = vertices[(edge + 1) % vertices.Length];

            Vector2 normal;
            float depth;
            Vector2 contact;

            if (separation < 1e-6f)
            {
                normal = normals[edge];
                depth = radius - separation;
                contact = center - normal * radius;
            }
            else
            {
                float u1 = Vector2.Dot(center - v1, v2 - v1);
                float u2 = Vector2.Dot(center - v2, v1 - v2);

                Vector2 closest;
                if (u1 <= 0f) closest = v1;
                else if (u2 <= 0f) closest = v2;
                else closest = center - normals[edge] * separation;

                Vector2 delta = center - closest;
                float distanceSquared = delta.LengthSquared();
                if (distanceSquared > radius * radius) return false;

                float distance = MathF.Sqrt(distanceSquared);
                normal = distance > 1e-6f ? delta / distance : normals[edge];
                depth = radius - distance;
                contact = closest;
            }

            manifold.Normal = flipOwner ? -normal : normal;
            manifold.Depth = depth;
            manifold.AddContact(contact);
            return true;
        }

        private static bool DetectCircleCircle(CircleCollider a, CircleCollider b, ref Manifold manifold)
        {
            Vector2 delta = b.WorldCenter - a.WorldCenter;
            float distSq = delta.LengthSquared();
            float radiusSum = a.Radius + b.Radius;

            if (distSq > radiusSum * radiusSum) return false;

            float distance = MathF.Sqrt(distSq);

            if (distance == 0)
            {
                manifold.Depth = radiusSum;
                manifold.Normal = Vector2.UnitX;
                manifold.AddContact(a.WorldCenter);
            }
            else
            {
                manifold.Depth = radiusSum - distance;
                manifold.Normal = delta / distance;
                manifold.AddContact(a.WorldCenter + manifold.Normal * a.Radius);
            }
            return true;
        }
    }
}
