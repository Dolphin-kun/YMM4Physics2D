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

                    bool isAConvex = colA.ShapeType == ShapeType.Polygon || colA.ShapeType == ShapeType.Box;
                    bool isBConvex = colB.ShapeType == ShapeType.Polygon || colB.ShapeType == ShapeType.Box;

                    if (isAConvex && isBConvex)
                    {
                        var manifold = new Manifold(bodyA, bodyB);

                        if (TryGetConvexData(colA, out var vertsA, out var axesA) &&
                            TryGetConvexData(colB, out var vertsB, out var axesB))
                        {
                            DetectConvex(vertsA, axesA, vertsB, axesB, ref manifold);

                            if (manifold.HasCollision)
                            {
                                manifolds.Add(manifold);
                            }
                        }
                    }
                    else if (colA.ShapeType == ShapeType.Circle && colB.ShapeType == ShapeType.Circle)
                    {
                        // 拡張用: 円同士の処理
                    }
                }
            }

            return manifolds;
        }

        
        private static bool FindSmallestAxis(Vector2[] axes, Vector2[] verticesA, Vector2[] verticesB, ref float minOverlap, ref Vector2 smallestAxis, bool isCheckingAxisA, ref bool axisOwnerIsA)
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

        private static bool TryGetConvexData(Collider col, out Vector2[] vertices, out Vector2[] axes)
        {
            vertices = null;
            axes = null;

            if (col is PolygonCollider poly)
            {
                vertices = poly.WorldVertices;
                axes = poly.WorldAxes;
                return true;
            }
            else if (col is BoxCollider box)
            {
                vertices = box.WorldVertices;

                axes = GetBoxAxes(vertices);
                return true;
            }

            return false;
        }

        private static void DetectConvex(Vector2[] verticesA, Vector2[] axesA, Vector2[] verticesB, Vector2[] axesB, ref Manifold manifold)
        {
            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.Zero;
            bool axisOwnerIsA = true;

            // Aの軸で判定
            if (!FindSmallestAxis(axesA, verticesA, verticesB, ref minOverlap, ref smallestAxis, true, ref axisOwnerIsA))
            {
                return;
            }

            // Bの軸で判定
            if (!FindSmallestAxis(axesB, verticesA, verticesB, ref minOverlap, ref smallestAxis, false, ref axisOwnerIsA))
            {
                return;
            }

            manifold.HasCollision = true;

            Vector2 centerA = PolygonCollider.GetPolygonCenter(verticesA);
            Vector2 centerB = PolygonCollider.GetPolygonCenter(verticesB);
            Vector2 direction = centerB - centerA;

            if (Vector2.Dot(direction, smallestAxis) < 0)
            {
                smallestAxis = -smallestAxis;
            }

            manifold.Normal = Vector2.Normalize(smallestAxis);
            manifold.Depth = minOverlap;

            if (axisOwnerIsA)
            {
                Vector2 point = GetSupportPoint(verticesB, -manifold.Normal);
                manifold.AddContact(point);
            }
            else
            {
                Vector2 point = GetSupportPoint(verticesA, manifold.Normal);
                manifold.AddContact(point);
            }
        }

        private static Vector2[] GetBoxAxes(Vector2[] vertices)
        {
            Vector2 edge1 = vertices[1] - vertices[0];
            Vector2 normal1 = Vector2.Normalize(new Vector2(-edge1.Y, edge1.X));

            Vector2 edge2 = vertices[2] - vertices[1];
            Vector2 normal2 = Vector2.Normalize(new Vector2(-edge2.Y, edge2.X));

            return [normal1, normal2];
        }

        private static Vector2 GetSupportPoint(Vector2[] vertices, Vector2 dir)
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

        private static void ProjectVertices(Vector2[] vertices, Vector2 axis, out float min, out float max)
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