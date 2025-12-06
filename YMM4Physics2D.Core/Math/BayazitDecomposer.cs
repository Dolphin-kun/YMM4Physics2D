using System.Numerics;

namespace YMM4Physics2D.Core.Math
{
    public static class BayazitDecomposer
    {
        private const int MaxRecursionDepth = 50;
        private const float Epsilon = 1e-4f;

        public static List<List<Vector2>> Decompose(List<Vector2> vertices)
        {
            return DecomposeRecursive(vertices, 0);
        }

        private static List<List<Vector2>> DecomposeRecursive(List<Vector2> vertices, int depth)
        {
            List<List<Vector2>> list = [];

            if (depth > MaxRecursionDepth || vertices.Count < 3)
            {
                list.Add(vertices);
                return list;
            }

            if (IsConvex(vertices))
            {
                list.Add(vertices);
                return list;
            }

            for (int i = 0; i < vertices.Count; ++i)
            {
                if (IsReflex(i, vertices))
                {
                    float minDist = float.MaxValue;
                    int closestIndex = -1;
                    Vector2 intersection = Vector2.Zero;

                    for (int j = 0; j < vertices.Count; ++j)
                    {
                        Vector2 p1 = At(vertices, i - 1);
                        Vector2 p2 = At(vertices, i);
                        Vector2 p3 = At(vertices, j);
                        Vector2 p4 = At(vertices, j - 1);

                        if (LineIntersect(p1, p2, p3, p4, out Vector2 hit))
                        {
                            float dist = Vector2.DistanceSquared(p2, hit);
                            if (dist < minDist)
                            {
                                minDist = dist;
                                closestIndex = j;
                                intersection = hit;
                            }
                        }
                    }

                    if (closestIndex == -1)
                    {
                        for (int j = 0; j < vertices.Count; ++j)
                        {
                            if (i == j || i == (j - 1 + vertices.Count) % vertices.Count || i == (j + 1) % vertices.Count) continue;

                            if (CanSee(i, j, vertices))
                            {
                                float dist = Vector2.DistanceSquared(At(vertices, i), At(vertices, j));
                                if (dist < minDist)
                                {
                                    minDist = dist;
                                    closestIndex = j;
                                    intersection = At(vertices, j);
                                }
                            }
                        }
                    }

                    if (closestIndex != -1)
                    {
                        List<Vector2> lowerPoly;
                        List<Vector2> upperPoly;

                        if (closestIndex < i)
                        {
                            lowerPoly = Copy(i, closestIndex, vertices);
                            upperPoly = Copy(closestIndex, i, vertices);
                        }
                        else
                        {
                            lowerPoly = Copy(i, closestIndex, vertices);
                            upperPoly = Copy(closestIndex, i, vertices);
                        }

                        if (lowerPoly.Count >= vertices.Count || upperPoly.Count >= vertices.Count)
                        {
                            list.Add(vertices);
                            return list;
                        }

                        list.AddRange(DecomposeRecursive(lowerPoly, depth + 1));
                        list.AddRange(DecomposeRecursive(upperPoly, depth + 1));
                        return list;
                    }
                }
            }

            if (list.Count == 0) list.Add(vertices);
            return list;
        }

        private static Vector2 At(List<Vector2> vertices, int index)
        {
            int c = vertices.Count;
            return vertices[(index % c + c) % c];
        }

        private static bool IsConvex(List<Vector2> vertices)
        {
            for (int i = 0; i < vertices.Count; i++)
            {
                if (IsReflex(i, vertices)) return false;
            }
            return true;
        }

        private static bool IsReflex(int i, List<Vector2> vertices)
        {
            return Cross(At(vertices, i - 1), At(vertices, i), At(vertices, i + 1)) < 0;
        }

        private static float Cross(Vector2 a, Vector2 b, Vector2 c)
        {
            return (b.X - a.X) * (c.Y - b.Y) - (b.Y - a.Y) * (c.X - b.X);
        }

        private static bool CanSee(int i, int j, List<Vector2> vertices)
        {
            if (Cross(At(vertices, i - 1), At(vertices, i), At(vertices, j)) <= 0) return false;
            if (Cross(At(vertices, i), At(vertices, i + 1), At(vertices, j)) <= 0) return false;

            Vector2 start = At(vertices, i);
            Vector2 end = At(vertices, j);

            for (int k = 0; k < vertices.Count; ++k)
            {
                int next = (k + 1) % vertices.Count;
                if (k == i || k == j || next == i || next == j) continue;

                if (LineIntersect(start, end, vertices[k], vertices[next], out _)) return false;
            }
            return true;
        }

        private static bool LineIntersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, out Vector2 intersection)
        {
            intersection = Vector2.Zero;
            float d = (p4.Y - p3.Y) * (p2.X - p1.X) - (p4.X - p3.X) * (p2.Y - p1.Y);

            if (System.Math.Abs(d) < Epsilon) return false;

            float uA = ((p4.X - p3.X) * (p1.Y - p3.Y) - (p4.Y - p3.Y) * (p1.X - p3.X)) / d;
            float uB = ((p2.X - p1.X) * (p1.Y - p3.Y) - (p2.Y - p1.Y) * (p1.X - p3.X)) / d;

            if (uA > Epsilon && uA < 1.0f - Epsilon && uB > Epsilon && uB < 1.0f - Epsilon)
            {
                intersection = new Vector2(p1.X + uA * (p2.X - p1.X), p1.Y + uA * (p2.Y - p1.Y));
                return true;
            }
            return false;
        }

        private static List<Vector2> Copy(int i, int j, List<Vector2> vertices)
        {
            List<Vector2> p = [];
            if (i < j)
            {
                for (int k = i; k <= j; k++) p.Add(At(vertices, k));
            }
            else
            {
                for (int k = i; k < vertices.Count; k++) p.Add(At(vertices, k));
                for (int k = 0; k <= j; k++) p.Add(At(vertices, k));
            }
            return p;
        }
    }
}