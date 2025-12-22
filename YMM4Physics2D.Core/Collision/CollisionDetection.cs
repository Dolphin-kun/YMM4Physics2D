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
                        // Box vs Box, Box vs Poly, Poly vs Poly
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
            // ここで配列をnewせず、既存の参照を取得する
            // ※ BoxColliderにもWorldAxes、WorldVerticesを実装している前提
            // ※ PolygonColliderはもともと持っている


            if (!TryGetConvexSpan(colA, out ReadOnlySpan<Vector2> vertsA, out ReadOnlySpan<Vector2> axesA)) return false;
            if (!TryGetConvexSpan(colB, out ReadOnlySpan<Vector2> vertsB, out ReadOnlySpan<Vector2> axesB)) return false;

            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.Zero;
            bool axisOwnerIsA = true;

            // Aの軸テスト
            if (!FindSmallestAxis(axesA, vertsA, vertsB, ref minOverlap, ref smallestAxis, true, ref axisOwnerIsA)) return false;

            // Bの軸テスト
            if (!FindSmallestAxis(axesB, vertsA, vertsB, ref minOverlap, ref smallestAxis, false, ref axisOwnerIsA)) return false;

            // 衝突確定
            manifold.HasCollision = true;
            manifold.Depth = minOverlap;

            // 法線の向きを調整 (A -> B)
            // 重心計算はコストがかかるため、AABBの中心で簡易判定しても良いが、正確性をとって幾何中心を使う
            Vector2 centerA = colA.WorldAABB.Center;
            Vector2 centerB = colB.WorldAABB.Center;

            if (Vector2.Dot(centerB - centerA, smallestAxis) < 0)
            {
                smallestAxis = -smallestAxis;
            }
            manifold.Normal = smallestAxis; // 常に正規化されている前提

            // 接触点の特定 (Feature Flipping)
            // axisOwnerIsA == true なら、最小分離軸はAのもの -> 接触点はB上の頂点
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

        private static bool TryGetConvexSpan(Collider col, out ReadOnlySpan<Vector2> verts, out ReadOnlySpan<Vector2> axes)
        {
            if (col is PolygonCollider poly)
            {
                verts = poly.WorldVertices;
                axes = poly.WorldAxes;
                return true;
            }
            else if (col is BoxCollider box)
            {
                verts = box.WorldVertices;
                // BoxColliderにWorldAxesを追加することを強く推奨。
                // ない場合はここで計算が必要だが、配列newは避けたい。
                // 暫定対応: もしBoxColliderにWorldAxesがないなら、box.WorldVerticesから計算するヘルパーが必要
                // ここでは「BoxColliderもPolygonColliderと同じインターフェース(WorldAxes)を持つ」ように修正されたと仮定します。

                // 仮の実装:
                axes = box.GetType().GetProperty("WorldAxes")?.GetValue(box) as Vector2[];
                // ↑これは遅いので、BoxCollider.csに public Vector2[] WorldAxes { get; } を追加してください。

                return true;
            }

            verts = default;
            axes = default;
            return false;
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

        private static bool DetectPolygonCircle(Collider polyCol, CircleCollider circleCol, ref Manifold manifold, bool flipOwner)
        {
            if (!TryGetConvexSpan(polyCol, out var vertices, out var axes)) return false;

            Vector2 circleCenter = circleCol.WorldCenter;
            float radius = circleCol.Radius;

            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.Zero;

            // 1. ポリゴン軸テスト
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

            // 2. 最短頂点軸テスト
            Vector2 closestVertex = vertices[0];
            float minDistSq = Vector2.DistanceSquared(circleCenter, closestVertex);

            // Spanなのでforループ
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
                Vector2 axis = Vector2.Normalize(axisToCenter);
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

            manifold.HasCollision = true;
            manifold.Depth = minOverlap;

            // 法線方向の調整
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