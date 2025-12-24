using System.Buffers;
using System.Numerics;
using YMM4Physics2D.Core.Math;

namespace YMM4Physics2D.Core.Core
{
    public static class ContourTracer
    {
        private static readonly ArrayPool<bool> _visitedPool = ArrayPool<bool>.Shared;

        private static readonly int[] dx = [0, 1, 1, 1, 0, -1, -1, -1];
        private static readonly int[] dy = [-1, -1, 0, 1, 1, 1, 0, -1];

        public static unsafe List<List<Vector2>> TraceAllContours(IntPtr dataPointer, int pitch, int width, int height, byte threshold)
        {
            List<List<Vector2>> allContours = [];
            byte* scan0 = (byte*)dataPointer;

            int length = width * height;
            bool[] globalVisited = _visitedPool.Rent(length);
            Array.Clear(globalVisited, 0, length);

            try
            {

                for (int y = 0; y < height; y++)
                {
                    byte* row = scan0 + (y * pitch);
                    bool insideFlow = false;
                    for (int x = 0; x < width; x++)
                    {
                        byte alpha = row[x * 4 + 3];
                        bool isOpaque = alpha > threshold;

                        if (isOpaque && !insideFlow && !globalVisited[y * width + x])
                        {
                            List<Vector2> contour = TraceFromPoint(scan0, pitch, width, height, threshold, x, y, globalVisited);

                            if (contour.Count >= 3)
                            {
                                float area = GeometryUtils.CalculateArea(contour);
                                if (System.Math.Abs(area) > 2.0f)
                                {
                                    allContours.Add(contour);
                                }
                            }

                            insideFlow = true;
                        }
                        else if (!isOpaque)
                        {
                            insideFlow = false;
                        }
                    }
                }
            }
            finally
            {
                _visitedPool.Return(globalVisited);
            }

            return allContours;
        }

        private static unsafe List<Vector2> TraceFromPoint(byte* scan0, int pitch, int width, int height, byte threshold, int startX, int startY, bool[] globalVisited)
        {
            List<Vector2> points = new(256);

            int x = startX;
            int y = startY;

            int backtrack = 4;

            points.Add(new Vector2(x, y));
            globalVisited[y * width + x] = true;

            int maxSteps = width * height;
            int steps = 0;

            while (steps < maxSteps)
            {
                bool foundNext = false;

                for (int i = 0; i < 8; i++)
                {
                    int idx = (backtrack + i) % 8;
                    int nx = x + dx[idx];
                    int ny = y + dy[idx];

                    if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                    {
                        byte* row = scan0 + (ny * pitch);
                        byte alpha = row[nx * 4 + 3];

                        if (alpha > threshold)
                        {
                            if (nx == startX && ny == startY)
                            {
                                return points;
                            }

                            x = nx;
                            y = ny;
                            points.Add(new Vector2(x, y));

                            globalVisited[y * width + x] = true;

                            backtrack = (idx + 5) % 8;

                            foundNext = true;
                            break;
                        }
                    }
                }

                if (!foundNext) break;
                steps++;
            }

            return points;
        }

        public static List<Vector2> SimplifyContour(List<Vector2> contour, float tolerance)
        {
            if (contour == null || contour.Count < 3) return [];

            bool closed = contour[0] == contour[^1];
            if (!closed) contour.Add(contour[0]);

            List<Vector2> result = [];
            SimplifyRecursive(contour, 0, contour.Count - 1, tolerance * tolerance, result);

            if (result.Count > 1 && result[0] == result[^1])
                result.RemoveAt(result.Count - 1);

            return result;
        }

        private static void SimplifyRecursive(List<Vector2> points, int first, int last, float tolerance, List<Vector2> result)
        {
            float maxDist = 0;
            int index = 0;

            Vector2 pFirst = points[first];
            Vector2 pLast = points[last];

            float lineLenSq = Vector2.DistanceSquared(pFirst, pLast);
            bool isPoint = lineLenSq < 1e-6f;

            for (int i = first + 1; i < last; i++)
            {
                float dist;
                if (isPoint)
                {
                    dist = Vector2.Distance(points[i], pFirst);
                }
                else
                {
                    dist = GetPerpendicularDistance(points[i], pFirst, pLast);
                }

                if (dist > maxDist)
                {
                    maxDist = dist;
                    index = i;
                }
            }

            if (maxDist > tolerance)
            {
                SimplifyRecursive(points, first, index, tolerance, result);
                SimplifyRecursive(points, index, last, tolerance, result);
            }
            else
            {
                if (result.Count == 0 || result[^1] != points[first])
                    result.Add(points[first]);

                if (result.Count == 0 || result[^1] != points[last])
                    result.Add(points[last]);
            }
        }

        private static float GetPerpendicularDistance(Vector2 p, Vector2 lineStart, Vector2 lineEnd)
        {
            float dx = lineEnd.X - lineStart.X;
            float dy = lineEnd.Y - lineStart.Y;

            float area = (dy * p.X - dx * p.Y + lineEnd.X * lineStart.Y - lineEnd.Y * lineStart.X);
            float numerator = area * area;
            float denominator = dx * dx + dy * dy;

            if (denominator == 0) return Vector2.DistanceSquared(p, lineStart);

            return numerator / denominator;
        }
    }
}