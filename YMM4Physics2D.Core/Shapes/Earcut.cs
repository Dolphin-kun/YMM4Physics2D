using System.Runtime.CompilerServices;

namespace YMM4Physics2D.Core.Shapes
{
    internal static class Earcut
    {
        private const long MinStepBudget = 200_000;
        private const long MaxStepBudget = 30_000_000;

        [ThreadStatic]
        private static long remainingSteps;

        private sealed class StepLimitExceededException : Exception;

        private sealed class Node(int index, double x, double y)
        {
            public readonly int I = index;
            public readonly double X = x;
            public readonly double Y = y;
            public Node Prev = null!;
            public Node Next = null!;
            public bool Steiner;
        }

        public static bool TryTriangulate(double[] data, IReadOnlyList<int> holeIndices, out List<int> triangles)
        {
            long vertices = data.Length / 2;
            remainingSteps = Math.Clamp(vertices * vertices * 32, MinStepBudget, MaxStepBudget);
            try
            {
                triangles = Triangulate(data, holeIndices);
                return true;
            }
            catch (StepLimitExceededException)
            {
                triangles = [];
                return false;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Step()
        {
            if (--remainingSteps < 0) throw new StepLimitExceededException();
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static List<int> Triangulate(double[] data, IReadOnlyList<int> holeIndices)
        {
            var triangles = new List<int>();
            bool hasHoles = holeIndices.Count > 0;
            int outerLength = hasHoles ? holeIndices[0] * 2 : data.Length;

            Node? outerNode = LinkedList(data, 0, outerLength, true);
            if (outerNode == null || outerNode.Next == outerNode.Prev) return triangles;

            if (hasHoles) outerNode = EliminateHoles(data, holeIndices, outerNode);

            EarcutLinked(outerNode, triangles, 0);
            return triangles;
        }

        private static Node? LinkedList(double[] data, int start, int end, bool clockwise)
        {
            Node? last = null;
            if (clockwise == (SignedArea(data, start, end) > 0))
            {
                for (int i = start; i < end; i += 2) last = InsertNode(i / 2, data[i], data[i + 1], last);
            }
            else
            {
                for (int i = end - 2; i >= start; i -= 2) last = InsertNode(i / 2, data[i], data[i + 1], last);
            }

            if (last != null && Equals(last, last.Next))
            {
                RemoveNode(last);
                last = last.Next;
            }

            return last;
        }

        private static Node? FilterPoints(Node? start, Node? end = null)
        {
            if (start == null) return start;
            end ??= start;

            Node p = start;
            bool again;
            do
            {
                Step();
                again = false;
                if (!p.Steiner && (Equals(p, p.Next) || Area(p.Prev, p, p.Next) == 0))
                {
                    RemoveNode(p);
                    p = end = p.Prev;
                    if (p == p.Next) break;
                    again = true;
                }
                else
                {
                    p = p.Next;
                }
            }
            while (again || p != end);

            return end;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static void EarcutLinked(Node? ear, List<int> triangles, int pass)
        {
            if (ear == null) return;

            Node stop = ear;
            while (ear.Prev != ear.Next)
            {
                Step();
                Node prev = ear.Prev;
                Node next = ear.Next;

                if (IsEar(ear))
                {
                    triangles.Add(prev.I);
                    triangles.Add(ear.I);
                    triangles.Add(next.I);

                    RemoveNode(ear);

                    ear = next.Next;
                    stop = next.Next;
                    continue;
                }

                ear = next;

                if (ear == stop)
                {
                    if (pass == 0)
                    {
                        EarcutLinked(FilterPoints(ear), triangles, 1);
                    }
                    else if (pass == 1)
                    {
                        Node? cured = CureLocalIntersections(FilterPoints(ear)!, triangles);
                        EarcutLinked(cured, triangles, 2);
                    }
                    else if (pass == 2)
                    {
                        SplitEarcut(ear, triangles);
                    }
                    break;
                }
            }
        }

        private static bool IsEar(Node ear)
        {
            Node a = ear.Prev;
            Node b = ear;
            Node c = ear.Next;

            if (Area(a, b, c) >= 0) return false;

            double x0 = Math.Min(a.X, Math.Min(b.X, c.X));
            double y0 = Math.Min(a.Y, Math.Min(b.Y, c.Y));
            double x1 = Math.Max(a.X, Math.Max(b.X, c.X));
            double y1 = Math.Max(a.Y, Math.Max(b.Y, c.Y));

            Node p = c.Next;
            while (p != a)
            {
                Step();
                if (p.X >= x0 && p.X <= x1 && p.Y >= y0 && p.Y <= y1 &&
                    PointInTriangle(a.X, a.Y, b.X, b.Y, c.X, c.Y, p.X, p.Y) &&
                    Area(p.Prev, p, p.Next) >= 0)
                {
                    return false;
                }
                p = p.Next;
            }

            return true;
        }

        private static Node? CureLocalIntersections(Node start, List<int> triangles)
        {
            Node p = start;
            do
            {
                Step();
                Node a = p.Prev;
                Node b = p.Next.Next;

                if (!Equals(a, b) && Intersects(a, p, p.Next, b) && LocallyInside(a, b) && LocallyInside(b, a))
                {
                    triangles.Add(a.I);
                    triangles.Add(p.I);
                    triangles.Add(b.I);

                    RemoveNode(p);
                    RemoveNode(p.Next);

                    p = start = b;
                }
                p = p.Next;
            }
            while (p != start);

            return FilterPoints(p);
        }

        private static void SplitEarcut(Node start, List<int> triangles)
        {
            Node a = start;
            do
            {
                Node b = a.Next.Next;
                while (b != a.Prev)
                {
                    Step();
                    if (a.I != b.I && IsValidDiagonal(a, b))
                    {
                        Node c = SplitPolygon(a, b);

                        Node? filteredA = FilterPoints(a, a.Next);
                        Node? filteredC = FilterPoints(c, c.Next);

                        EarcutLinked(filteredA, triangles, 0);
                        EarcutLinked(filteredC, triangles, 0);
                        return;
                    }
                    b = b.Next;
                }
                a = a.Next;
            }
            while (a != start);
        }

        private static Node EliminateHoles(double[] data, IReadOnlyList<int> holeIndices, Node outerNode)
        {
            var queue = new List<Node>();
            for (int i = 0; i < holeIndices.Count; i++)
            {
                int start = holeIndices[i] * 2;
                int end = i < holeIndices.Count - 1 ? holeIndices[i + 1] * 2 : data.Length;
                Node? list = LinkedList(data, start, end, false);
                if (list == null) continue;
                if (list == list.Next) list.Steiner = true;
                queue.Add(GetLeftmost(list));
            }

            queue.Sort((a, b) => a.X.CompareTo(b.X));

            foreach (var hole in queue)
            {
                outerNode = EliminateHole(hole, outerNode);
            }

            return outerNode;
        }

        private static Node EliminateHole(Node hole, Node outerNode)
        {
            Node? bridge = FindHoleBridge(hole, outerNode);
            if (bridge == null) return outerNode;

            Node bridgeReverse = SplitPolygon(bridge, hole);

            FilterPoints(bridgeReverse, bridgeReverse.Next);
            return FilterPoints(bridge, bridge.Next)!;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static Node? FindHoleBridge(Node hole, Node outerNode)
        {
            Node p = outerNode;
            double hx = hole.X;
            double hy = hole.Y;
            double qx = double.NegativeInfinity;
            Node? m = null;

            do
            {
                Step();
                if (hy <= p.Y && hy >= p.Next.Y && p.Next.Y != p.Y)
                {
                    double x = p.X + (hy - p.Y) * (p.Next.X - p.X) / (p.Next.Y - p.Y);
                    if (x <= hx && x > qx)
                    {
                        qx = x;
                        m = p.X < p.Next.X ? p : p.Next;
                        if (x == hx) return m;
                    }
                }
                p = p.Next;
            }
            while (p != outerNode);

            if (m == null) return null;

            Node stop = m;
            double mx = m.X;
            double my = m.Y;
            double tanMin = double.PositiveInfinity;

            p = m;
            do
            {
                Step();
                if (hx >= p.X && p.X >= mx && hx != p.X &&
                    PointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p.X, p.Y))
                {
                    double tan = Math.Abs(hy - p.Y) / (hx - p.X);

                    if (LocallyInside(p, hole) &&
                        (tan < tanMin || (tan == tanMin && (p.X > m.X || (p.X == m.X && SectorContainsSector(m, p))))))
                    {
                        m = p;
                        tanMin = tan;
                    }
                }
                p = p.Next;
            }
            while (p != stop);

            return m;
        }

        private static bool SectorContainsSector(Node m, Node p)
        {
            return Area(m.Prev, m, p.Prev) < 0 && Area(p.Next, m, m.Next) < 0;
        }

        private static Node GetLeftmost(Node start)
        {
            Node p = start;
            Node leftmost = start;
            do
            {
                Step();
                if (p.X < leftmost.X || (p.X == leftmost.X && p.Y < leftmost.Y)) leftmost = p;
                p = p.Next;
            }
            while (p != start);
            return leftmost;
        }

        private static bool PointInTriangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py)
        {
            return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
                   (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
                   (bx - px) * (cy - py) >= (cx - px) * (by - py);
        }

        private static bool IsValidDiagonal(Node a, Node b)
        {
            return a.Next.I != b.I && a.Prev.I != b.I && !IntersectsPolygon(a, b) &&
                   (LocallyInside(a, b) && LocallyInside(b, a) && MiddleInside(a, b) &&
                    (Area(a.Prev, a, b.Prev) != 0 || Area(a, b.Prev, b) != 0) ||
                    Equals(a, b) && Area(a.Prev, a, a.Next) > 0 && Area(b.Prev, b, b.Next) > 0);
        }

        private static double Area(Node p, Node q, Node r)
        {
            return (q.Y - p.Y) * (r.X - q.X) - (q.X - p.X) * (r.Y - q.Y);
        }

        private static bool Equals(Node p1, Node p2)
        {
            return p1.X == p2.X && p1.Y == p2.Y;
        }

        private static bool Intersects(Node p1, Node q1, Node p2, Node q2)
        {
            int o1 = Sign(Area(p1, q1, p2));
            int o2 = Sign(Area(p1, q1, q2));
            int o3 = Sign(Area(p2, q2, p1));
            int o4 = Sign(Area(p2, q2, q1));

            if (o1 != o2 && o3 != o4) return true;
            if (o1 == 0 && OnSegment(p1, p2, q1)) return true;
            if (o2 == 0 && OnSegment(p1, q2, q1)) return true;
            if (o3 == 0 && OnSegment(p2, p1, q2)) return true;
            if (o4 == 0 && OnSegment(p2, q1, q2)) return true;
            return false;
        }

        private static bool OnSegment(Node p, Node q, Node r)
        {
            return q.X <= Math.Max(p.X, r.X) && q.X >= Math.Min(p.X, r.X) &&
                   q.Y <= Math.Max(p.Y, r.Y) && q.Y >= Math.Min(p.Y, r.Y);
        }

        private static int Sign(double value)
        {
            return value > 0 ? 1 : value < 0 ? -1 : 0;
        }

        private static bool IntersectsPolygon(Node a, Node b)
        {
            Node p = a;
            do
            {
                Step();
                if (p.I != a.I && p.Next.I != a.I && p.I != b.I && p.Next.I != b.I && Intersects(p, p.Next, a, b)) return true;
                p = p.Next;
            }
            while (p != a);
            return false;
        }

        private static bool LocallyInside(Node a, Node b)
        {
            return Area(a.Prev, a, a.Next) < 0
                ? Area(a, b, a.Next) >= 0 && Area(a, a.Prev, b) >= 0
                : Area(a, b, a.Prev) < 0 || Area(a, a.Next, b) < 0;
        }

        private static bool MiddleInside(Node a, Node b)
        {
            Node p = a;
            bool inside = false;
            double px = (a.X + b.X) / 2;
            double py = (a.Y + b.Y) / 2;
            do
            {
                Step();
                if (((p.Y > py) != (p.Next.Y > py)) && p.Next.Y != p.Y &&
                    (px < (p.Next.X - p.X) * (py - p.Y) / (p.Next.Y - p.Y) + p.X))
                {
                    inside = !inside;
                }
                p = p.Next;
            }
            while (p != a);
            return inside;
        }

        private static Node SplitPolygon(Node a, Node b)
        {
            var a2 = new Node(a.I, a.X, a.Y);
            var b2 = new Node(b.I, b.X, b.Y);
            Node an = a.Next;
            Node bp = b.Prev;

            a.Next = b;
            b.Prev = a;

            a2.Next = an;
            an.Prev = a2;

            b2.Next = a2;
            a2.Prev = b2;

            bp.Next = b2;
            b2.Prev = bp;

            return b2;
        }

        private static Node InsertNode(int index, double x, double y, Node? last)
        {
            var p = new Node(index, x, y);
            if (last == null)
            {
                p.Prev = p;
                p.Next = p;
            }
            else
            {
                p.Next = last.Next;
                p.Prev = last;
                last.Next.Prev = p;
                last.Next = p;
            }
            return p;
        }

        private static void RemoveNode(Node p)
        {
            p.Next.Prev = p.Prev;
            p.Prev.Next = p.Next;
        }

        private static double SignedArea(double[] data, int start, int end)
        {
            double sum = 0;
            for (int i = start, j = end - 2; i < end; i += 2)
            {
                sum += (data[j] - data[i]) * (data[i + 1] + data[j + 1]);
                j = i;
            }
            return sum;
        }
    }
}
