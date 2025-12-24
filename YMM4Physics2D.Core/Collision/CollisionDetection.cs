using System.Numerics;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Colliders;

namespace YMM4Physics2D.Core.Collision
{
    public static class CollisionDetection
    {
        public static List<Manifold> Detect(RigidBody bodyA, RigidBody bodyB)
        {
            var manifolds = new List<Manifold>();

            foreach (var colA in bodyA.Colliders)
            {
                foreach (var colB in bodyB.Colliders)
                {
                    if (!colA.WorldAABB.Overlaps(colB.WorldAABB)) continue;

                    var manifold = new Manifold(bodyA, bodyB);
                    bool hasCollision;

                    if (colA is CircleCollider circleA && colB is CircleCollider circleB)
                    {
                        hasCollision = DetectCircleCircle(circleA, circleB, ref manifold);
                    }
                    else if (colA is CircleCollider cA && (colB is BoxCollider || colB is PolygonCollider))
                    {
                        hasCollision = DetectPolygonCircle(colB, cA, ref manifold, flipOwner: true);
                    }
                    else if ((colA is BoxCollider || colA is PolygonCollider) && colB is CircleCollider cB)
                    {
                        hasCollision = DetectPolygonCircle(colA, cB, ref manifold, flipOwner: false);
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

            return manifolds;
        }

        private static bool DetectConvex(Collider colA, Collider colB, ref Manifold manifold)
        {
            Span<Vector2> boxAxesA = stackalloc Vector2[2];
            Span<Vector2> boxAxesB = stackalloc Vector2[2];

            ReadOnlySpan<Vector2> vertsA;
            if (colA is PolygonCollider pA) vertsA = pA.WorldVertices;
            else if (colA is BoxCollider bA) vertsA = bA.WorldVertices;
            else return false;

            ReadOnlySpan<Vector2> vertsB;
            if (colB is PolygonCollider pB) vertsB = pB.WorldVertices;
            else if (colB is BoxCollider bB) vertsB = bB.WorldVertices;
            else return false;

            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.Zero;
            bool axisOwnerIsA = true;

            if (colA is PolygonCollider polyA)
            {
                if (!FindSmallestAxis(polyA.WorldAxes, vertsA, vertsB, ref minOverlap, ref smallestAxis, true, ref axisOwnerIsA)) return false;
            }
            else if (colA is BoxCollider boxA)
            {
                boxAxesA[0] = Vector2.Normalize(vertsA[1] - vertsA[0]);
                boxAxesA[1] = Vector2.Normalize(vertsA[2] - vertsA[1]);
                if (!FindSmallestAxis(boxAxesA, vertsA, vertsB, ref minOverlap, ref smallestAxis, true, ref axisOwnerIsA)) return false;
            }

            if (colB is PolygonCollider polyB)
            {
                if (!FindSmallestAxis(polyB.WorldAxes, vertsA, vertsB, ref minOverlap, ref smallestAxis, false, ref axisOwnerIsA)) return false;
            }
            else if (colB is BoxCollider boxB)
            {
                boxAxesB[0] = Vector2.Normalize(vertsB[1] - vertsB[0]);
                boxAxesB[1] = Vector2.Normalize(vertsB[2] - vertsB[1]);
                if (!FindSmallestAxis(boxAxesB, vertsA, vertsB, ref minOverlap, ref smallestAxis, false, ref axisOwnerIsA)) return false;
            }

            manifold.HasCollision = true;
            manifold.Depth = minOverlap;

            Vector2 centerA = colA.WorldAABB.Center;
            Vector2 centerB = colB.WorldAABB.Center;

            if (Vector2.Dot(centerB - centerA, smallestAxis) < 0)
            {
                smallestAxis = -smallestAxis;
            }
            manifold.Normal = smallestAxis;

            if (axisOwnerIsA)
            {
                manifold.AddContact(GetSupportPoint(vertsB, -manifold.Normal));
            }
            else
            {
                manifold.AddContact(GetSupportPoint(vertsA, manifold.Normal));
            }

            return true;
        }

        private static bool DetectPolygonCircle(Collider polyCol, CircleCollider circleCol, ref Manifold manifold, bool flipOwner)
        {
            Vector2 circleCenter = circleCol.WorldCenter;
            float radius = circleCol.Radius;
            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.Zero;

            ReadOnlySpan<Vector2> vertices;
            if (polyCol is PolygonCollider poly)
            {
                vertices = poly.WorldVertices;
                if (!CheckCircleAxes(poly.WorldAxes, vertices, circleCenter, radius, ref minOverlap, ref smallestAxis))
                    return false;
            }
            else if (polyCol is BoxCollider box)
            {
                vertices = box.WorldVertices;

                Span<Vector2> boxAxes = [Vector2.Normalize(vertices[1] - vertices[0]), Vector2.Normalize(vertices[2] - vertices[1])];
                if (!CheckCircleAxes(boxAxes, vertices, circleCenter, radius, ref minOverlap, ref smallestAxis))
                    return false;
            }
            else return false;

            Vector2 closestVertex = vertices[0];
            float minDistSq = Vector2.DistanceSquared(circleCenter, closestVertex);

            for (int i = 1; i < vertices.Length; i++)
            {
                float d = Vector2.DistanceSquared(circleCenter, vertices[i]);
                if (d < minDistSq)
                {
                    minDistSq = d;
                    closestVertex = vertices[i];
                }
            }

            Vector2 axisToCenter = circleCenter - closestVertex;
            if (axisToCenter.LengthSquared() > 1e-6f)
            {
                Span<Vector2> vertexAxis = [Vector2.Normalize(axisToCenter)];
                if (!CheckCircleAxes(vertexAxis, vertices, circleCenter, radius, ref minOverlap, ref smallestAxis))
                    return false;
            }

            manifold.HasCollision = true;
            manifold.Depth = minOverlap;

            if (Vector2.Dot(circleCenter - polyCol.WorldAABB.Center, smallestAxis) < 0)
            {
                smallestAxis = -smallestAxis;
            }

            if (flipOwner) manifold.Normal = -smallestAxis;
            else manifold.Normal = smallestAxis;

            manifold.AddContact(circleCenter - smallestAxis * radius);
            return true;
        }

        private static bool DetectCircleCircle(CircleCollider a, CircleCollider b, ref Manifold manifold)
        {
            Vector2 delta = b.WorldCenter - a.WorldCenter;
            float distSq = delta.LengthSquared();
            float radiusSum = a.Radius + b.Radius;

            if (distSq > radiusSum * radiusSum) return false;

            float distance = MathF.Sqrt(distSq);
            manifold.HasCollision = true;

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

        private static bool FindSmallestAxis(ReadOnlySpan<Vector2> axes, ReadOnlySpan<Vector2> verticesA, ReadOnlySpan<Vector2> verticesB, ref float minOverlap, ref Vector2 smallestAxis, bool isCheckingAxisA, ref bool axisOwnerIsA)
        {
            foreach (var axis in axes)
            {
                ProjectVertices(verticesA, axis, out float minA, out float maxA);
                ProjectVertices(verticesB, axis, out float minB, out float maxB);

                float overlap = System.Math.Min(maxA, maxB) - System.Math.Max(minA, minB);

                if (overlap <= 0) return false;

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    smallestAxis = axis;
                    axisOwnerIsA = isCheckingAxisA;
                }
            }

            return true;
        }

        private static bool CheckCircleAxes(ReadOnlySpan<Vector2> axes, ReadOnlySpan<Vector2> vertices, Vector2 circleCenter, float radius, ref float minOverlap, ref Vector2 smallestAxis)
        {
            foreach (var axis in axes)
            {
                ProjectVertices(vertices, axis, out float minP, out float maxP);
                float projC = Vector2.Dot(circleCenter, axis);
                float minC = projC - radius;
                float maxC = projC + radius;

                float overlap = System.Math.Min(maxP, maxC) - System.Math.Max(minP, minC);
                if (overlap <= 0) return false;

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    smallestAxis = axis;
                }
            }
            return true;
        }

        private static Vector2 GetSupportPoint(ReadOnlySpan<Vector2> vertices, Vector2 dir)
        {
            Vector2 bestVertex = vertices[0];
            float maxDot = Vector2.Dot(vertices[0], dir);

            for (int i = 1; i < vertices.Length; i++)
            {
                float dot = Vector2.Dot(vertices[i], dir);
                if (dot > maxDot)
                {
                    maxDot = dot;
                    bestVertex = vertices[i];
                }
            }

            return bestVertex;
        }

        private static void ProjectVertices(ReadOnlySpan<Vector2> vertices, Vector2 axis, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;

            foreach (var v in vertices)
            {
                float proj = Vector2.Dot(v, axis);
                if (proj < min) min = proj;
                if (proj > max) max = proj;
            }
        }
    }
}