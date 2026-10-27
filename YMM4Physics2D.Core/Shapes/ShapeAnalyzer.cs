using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Geometry;

namespace YMM4Physics2D.Core.Shapes
{
    public static class ShapeAnalyzer
    {
        private const double MinLoopArea = 4.0;

        private const float OutlineTolerance = 0.5f;

        private sealed class Loop(List<Vector2> points, double area)
        {
            public List<Vector2> Points { get; } = points;
            public double Area { get; } = area;
            public List<Loop> Holes { get; } = [];
            public Vector2 Min { get; } = points.Aggregate(new Vector2(float.MaxValue), Vector2.Min);
            public Vector2 Max { get; } = points.Aggregate(new Vector2(float.MinValue), Vector2.Max);
        }

        public static ShapeVersion Analyze(byte[] mask, int width, int height, Vector2 origin, float pixelSize, float tolerance, ulong hash)
        {
            var outers = new List<Loop>();
            var holes = new List<Loop>();

            foreach (var points in BoundaryTracer.Trace(mask, width, height))
            {
                if (points.Count < 3) continue;

                double area = SignedArea(points);
                if (Math.Abs(area) < MinLoopArea) continue;

                (area > 0 ? outers : holes).Add(new Loop(points, area));
            }

            foreach (var hole in holes)
            {
                Vector2 probe = (hole.Points[0] + hole.Points[1]) * 0.5f;
                Loop? owner = null;

                foreach (var outer in outers)
                {
                    if (outer.Area <= -hole.Area) continue;
                    if (probe.X < outer.Min.X || probe.Y < outer.Min.Y || probe.X > outer.Max.X || probe.Y > outer.Max.Y) continue;
                    if (owner != null && outer.Area >= owner.Area) continue;
                    if (!GeometryUtils.IsPointInPolygon(probe, outer.Points)) continue;

                    owner = outer;
                }

                owner?.Holes.Add(hole);
            }

            float gridTolerance = tolerance / pixelSize;
            var components = new List<ShapeComponent>(outers.Count);

            foreach (var outer in outers)
            {
                var component = CreateComponent(outer, gridTolerance, origin, pixelSize);
                if (component != null) components.Add(component);
            }

            return new ShapeVersion(hash, [.. components]);
        }

        private static ShapeComponent? CreateComponent(Loop outer, float gridTolerance, Vector2 origin, float pixelSize)
        {
            var (outerPoints, holePoints) = SimplifyValidated(outer, gridTolerance);
            if (outerPoints.Count < 3) return null;

            int vertexCount = outerPoints.Count + holePoints.Sum(h => h.Count);
            var data = new double[vertexCount * 2];
            var holeIndices = new List<int>(holePoints.Count);

            int offset = 0;
            AppendPoints(data, ref offset, outerPoints);
            foreach (var hole in holePoints)
            {
                holeIndices.Add(offset / 2);
                AppendPoints(data, ref offset, hole);
            }

            if (!Earcut.TryTriangulate(data, holeIndices, out var triangles))
            {
                data = data[..(outerPoints.Count * 2)];
                if (!Earcut.TryTriangulate(data, [], out triangles)) return null;
            }
            var parts = ConvexMerger.Merge(data, triangles);
            if (parts.Count == 0) return null;

            float totalArea = 0f;
            Vector2 weightedCentroid = Vector2.Zero;

            foreach (var part in parts)
            {
                for (int i = 0; i < part.Length; i++)
                {
                    part[i] = origin + part[i] * pixelSize;
                }

                float area = GeometryUtils.CalculateArea(part);
                totalArea += area;
                weightedCentroid += GeometryUtils.CalculateCentroid(part) * area;
            }

            if (totalArea < 1e-4f) return null;

            var outline = new List<Vector2[]>(1 + outer.Holes.Count)
            {
                ToImageSpace(PolylineSimplifier.SimplifyClosed(outer.Points, OutlineTolerance), origin, pixelSize)
            };
            foreach (var hole in outer.Holes)
            {
                outline.Add(ToImageSpace(PolylineSimplifier.SimplifyClosed(hole.Points, OutlineTolerance), origin, pixelSize));
            }

            return new ShapeComponent([.. parts], [.. outline], totalArea, weightedCentroid / totalArea);
        }

        private static (List<Vector2> Outer, List<List<Vector2>> Holes) SimplifyValidated(Loop outer, float gridTolerance)
        {
            const int Attempts = 4;

            for (int attempt = 0; attempt < Attempts; attempt++)
            {
                float tolerance = attempt == Attempts - 1 ? 0f : gridTolerance * MathF.Pow(0.5f, attempt);

                var outerPoints = PolylineSimplifier.SimplifyClosed(outer.Points, tolerance);
                if (outerPoints.Count < 3 || SignedArea(outerPoints) <= 0) continue;

                var holePoints = new List<List<Vector2>>(outer.Holes.Count);
                bool flipped = false;
                foreach (var hole in outer.Holes)
                {
                    var simplified = PolylineSimplifier.SimplifyClosed(hole.Points, tolerance);
                    if (simplified.Count < 3) continue;

                    double area = SignedArea(simplified);
                    if (area >= 0)
                    {
                        flipped = true;
                        break;
                    }
                    if (-area < MinLoopArea) continue;

                    holePoints.Add(simplified);
                }

                if (flipped) continue;

                if (tolerance == 0f || !HasIntersections(outerPoints, holePoints))
                {
                    return (outerPoints, holePoints);
                }
            }

            return ([], []);
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static bool HasIntersections(List<Vector2> outer, List<List<Vector2>> holes)
        {
            var segments = new List<(Vector2 A, Vector2 B, int Loop, int Index, int Count)>();
            AddSegments(segments, outer, 0);
            for (int i = 0; i < holes.Count; i++)
            {
                AddSegments(segments, holes[i], i + 1);
            }

            for (int i = 0; i < segments.Count; i++)
            {
                var s1 = segments[i];
                float minX1 = MathF.Min(s1.A.X, s1.B.X), maxX1 = MathF.Max(s1.A.X, s1.B.X);
                float minY1 = MathF.Min(s1.A.Y, s1.B.Y), maxY1 = MathF.Max(s1.A.Y, s1.B.Y);

                for (int j = i + 1; j < segments.Count; j++)
                {
                    var s2 = segments[j];

                    if (s1.Loop == s2.Loop)
                    {
                        int difference = s2.Index - s1.Index;
                        if (difference == 1 || difference == s1.Count - 1) continue;
                    }

                    if (MathF.Max(s2.A.X, s2.B.X) < minX1 || MathF.Min(s2.A.X, s2.B.X) > maxX1 ||
                        MathF.Max(s2.A.Y, s2.B.Y) < minY1 || MathF.Min(s2.A.Y, s2.B.Y) > maxY1)
                    {
                        continue;
                    }

                    if (SegmentsIntersect(s1.A, s1.B, s2.A, s2.B)) return true;
                }
            }

            return false;
        }

        private static void AddSegments(List<(Vector2, Vector2, int, int, int)> segments, List<Vector2> loop, int loopIndex)
        {
            for (int i = 0; i < loop.Count; i++)
            {
                segments.Add((loop[i], loop[(i + 1) % loop.Count], loopIndex, i, loop.Count));
            }
        }

        private static bool SegmentsIntersect(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2)
        {
            int o1 = Orientation(p1, q1, p2);
            int o2 = Orientation(p1, q1, q2);
            int o3 = Orientation(p2, q2, p1);
            int o4 = Orientation(p2, q2, q1);

            if (o1 != o2 && o3 != o4) return true;
            if (o1 == 0 && OnSegment(p1, p2, q1)) return true;
            if (o2 == 0 && OnSegment(p1, q2, q1)) return true;
            if (o3 == 0 && OnSegment(p2, p1, q2)) return true;
            if (o4 == 0 && OnSegment(p2, q1, q2)) return true;
            return false;
        }

        private static int Orientation(Vector2 p, Vector2 q, Vector2 r)
        {
            double value = ((double)q.X - p.X) * ((double)r.Y - p.Y) - ((double)q.Y - p.Y) * ((double)r.X - p.X);
            return value > 0 ? 1 : value < 0 ? -1 : 0;
        }

        private static bool OnSegment(Vector2 p, Vector2 q, Vector2 r)
        {
            return q.X <= MathF.Max(p.X, r.X) && q.X >= MathF.Min(p.X, r.X) &&
                   q.Y <= MathF.Max(p.Y, r.Y) && q.Y >= MathF.Min(p.Y, r.Y);
        }

        private static double SignedArea(List<Vector2> points)
        {
            double sum = 0;
            for (int i = 0, j = points.Count - 1; i < points.Count; j = i++)
            {
                sum += (double)points[j].X * points[i].Y - (double)points[i].X * points[j].Y;
            }
            return sum * 0.5;
        }

        private static void AppendPoints(double[] data, ref int offset, List<Vector2> points)
        {
            foreach (var point in points)
            {
                data[offset++] = point.X;
                data[offset++] = point.Y;
            }
        }

        private static Vector2[] ToImageSpace(List<Vector2> points, Vector2 origin, float pixelSize)
        {
            var result = new Vector2[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                result[i] = origin + points[i] * pixelSize;
            }
            return result;
        }
    }
}
