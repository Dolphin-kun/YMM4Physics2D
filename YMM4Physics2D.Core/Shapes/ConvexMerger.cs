using System.Numerics;

namespace YMM4Physics2D.Core.Shapes
{
    internal static class ConvexMerger
    {
        private const double ConvexEpsilon = 1e-9;

        public static List<Vector2[]> Merge(double[] data, List<int> triangles)
        {
            var polygons = new List<List<int>?>();
            for (int i = 0; i + 2 < triangles.Count; i += 3)
            {
                int a = triangles[i];
                int b = triangles[i + 1];
                int c = triangles[i + 2];

                double area = Cross(data, a, b, c);
                if (Math.Abs(area) < 1e-9) continue;

                polygons.Add(area > 0 ? [a, b, c] : [a, c, b]);
            }

            var edgeOwners = new Dictionary<(int, int), int>();
            for (int p = 0; p < polygons.Count; p++)
            {
                RegisterEdges(polygons[p]!, p, edgeOwners);
            }

            var diagonals = new List<(int A, int B, double Length)>();
            foreach (var edge in edgeOwners.Keys)
            {
                if (edge.Item1 < edge.Item2 && edgeOwners.ContainsKey((edge.Item2, edge.Item1)))
                {
                    double dx = data[edge.Item1 * 2] - data[edge.Item2 * 2];
                    double dy = data[edge.Item1 * 2 + 1] - data[edge.Item2 * 2 + 1];
                    diagonals.Add((edge.Item1, edge.Item2, dx * dx + dy * dy));
                }
            }
            diagonals.Sort((x, y) => y.Length.CompareTo(x.Length));

            foreach (var (a, b, _) in diagonals)
            {
                if (!edgeOwners.TryGetValue((a, b), out int first)) continue;
                if (!edgeOwners.TryGetValue((b, a), out int second)) continue;
                if (first == second) continue;

                var polygonA = polygons[first]!;
                var polygonB = polygons[second]!;

                var merged = TryMerge(data, polygonA, polygonB, a, b);
                if (merged == null) continue;

                UnregisterEdges(polygonA, edgeOwners);
                UnregisterEdges(polygonB, edgeOwners);
                polygons[first] = merged;
                polygons[second] = null;
                RegisterEdges(merged, first, edgeOwners);
            }

            var result = new List<Vector2[]>();
            foreach (var polygon in polygons)
            {
                if (polygon == null) continue;

                var points = new List<Vector2>(polygon.Count);
                foreach (int index in polygon)
                {
                    points.Add(new Vector2((float)data[index * 2], (float)data[index * 2 + 1]));
                }

                points = PolylineSimplifier.RemoveCollinear(points);
                if (points.Count >= 3) result.Add([.. points]);
            }

            return result;
        }

        private static List<int>? TryMerge(double[] data, List<int> polygonA, List<int> polygonB, int a, int b)
        {
            int indexA = FindEdge(polygonA, a, b);
            int indexB = FindEdge(polygonB, b, a);
            if (indexA < 0 || indexB < 0) return null;

            var merged = new List<int>(polygonA.Count + polygonB.Count - 2);

            for (int i = 1; i <= polygonA.Count; i++)
            {
                merged.Add(polygonA[(indexA + i) % polygonA.Count]);
            }
            for (int i = 2; i < polygonB.Count; i++)
            {
                merged.Add(polygonB[(indexB + i) % polygonB.Count]);
            }

            for (int i = 0; i < merged.Count; i++)
            {
                int previous = merged[(i - 1 + merged.Count) % merged.Count];
                int current = merged[i];
                int next = merged[(i + 1) % merged.Count];
                if (current != a && current != b) continue;
                if (Cross(data, previous, current, next) < -ConvexEpsilon) return null;
            }

            return merged;
        }

        private static int FindEdge(List<int> polygon, int from, int to)
        {
            for (int i = 0; i < polygon.Count; i++)
            {
                if (polygon[i] == from && polygon[(i + 1) % polygon.Count] == to) return i;
            }
            return -1;
        }

        private static void RegisterEdges(List<int> polygon, int owner, Dictionary<(int, int), int> edgeOwners)
        {
            for (int i = 0; i < polygon.Count; i++)
            {
                edgeOwners[(polygon[i], polygon[(i + 1) % polygon.Count])] = owner;
            }
        }

        private static void UnregisterEdges(List<int> polygon, Dictionary<(int, int), int> edgeOwners)
        {
            for (int i = 0; i < polygon.Count; i++)
            {
                edgeOwners.Remove((polygon[i], polygon[(i + 1) % polygon.Count]));
            }
        }

        private static double Cross(double[] data, int a, int b, int c)
        {
            double ax = data[a * 2], ay = data[a * 2 + 1];
            double bx = data[b * 2], by = data[b * 2 + 1];
            double cx = data[c * 2], cy = data[c * 2 + 1];
            return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        }
    }
}
