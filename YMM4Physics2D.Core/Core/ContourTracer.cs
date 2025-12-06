using System.Numerics;

namespace YMM4Physics2D.Core.Core
{
    public static class ContourTracer
    {
        public static unsafe List<List<Vector2>> TraceAllContours(IntPtr dataPointer, int pitch, int width, int height, byte threshold)
        {
            List<List<Vector2>> allContours = [];
            byte* scan0 = (byte*)dataPointer;

            bool[] globalVisited = new bool[width * height];

            for (int y = 0; y < height; y++)
            {
                byte* row = scan0 + (y * pitch);
                for (int x = 0; x < width; x++)
                {
                    int index = y * width + x;

                    if (!globalVisited[index] && row[x * 4 + 3] > threshold)
                    {
                        List<Vector2> contour = TraceFromPoint(scan0, pitch, width, height, threshold, x, y, globalVisited);

                        if (contour.Count >= 3)
                        {
                            float area = 0;
                            for (int i = 0; i < contour.Count; i++)
                            {
                                Vector2 p1 = contour[i];
                                Vector2 p2 = contour[(i + 1) % contour.Count];
                                area += (p1.X * p2.Y) - (p2.X * p1.Y);
                            }

                            if (System.Math.Abs(area * 0.5f) > 2.0f)
                            {
                                allContours.Add(contour);
                            }
                        }
                    }
                }
            }

            return allContours;
        }

        private static unsafe List<Vector2> TraceFromPoint(byte* scan0, int pitch, int width, int height, byte threshold, int startX, int startY, bool[] globalVisited)
        {
            List<Vector2> points = [];
            Dictionary<int, int> localHistory = [];

            int[] dx = [0, 1, 1, 1, 0, -1, -1, -1];
            int[] dy = [-1, -1, 0, 1, 1, 1, 0, -1];

            int x = startX;
            int y = startY;
            int backtrack = 4;

            points.Add(new Vector2(x, y));
            globalVisited[y * width + x] = true;
            localHistory[y * width + x] = backtrack;

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

                            int key = ny * width + nx;
                            int nextBacktrack = (idx + 5) % 8;

                            if (localHistory.TryGetValue(key, out int prevBacktrack))
                            {
                                if (prevBacktrack == nextBacktrack)
                                {
                                    return points;
                                }
                            }

                            x = nx;
                            y = ny;
                            points.Add(new Vector2(x, y));

                            globalVisited[y * width + x] = true;
                            localHistory[key] = nextBacktrack;
                            backtrack = nextBacktrack;

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
            SimplifyRecursive(contour, 0, contour.Count - 1, tolerance, result);

            if (result.Count > 1 && result[0] == result[^1])
                result.RemoveAt(result.Count - 1);

            return result;
        }

        private static void SimplifyRecursive(List<Vector2> points, int first, int last, float tolerance, List<Vector2> result)
        {
            float maxDist = 0;
            int index = 0;

            for (int i = first + 1; i < last; i++)
            {
                float dist = PerpendicularDistance(points[i], points[first], points[last]);
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

        private static float PerpendicularDistance(Vector2 p, Vector2 lineStart, Vector2 lineEnd)
        {
            float area = System.Math.Abs(0.5f * (lineStart.X * lineEnd.Y + lineEnd.X * p.Y + p.X * lineStart.Y - lineEnd.X * lineStart.Y - p.X * lineEnd.Y - lineStart.X * p.Y));
            float bottom = Vector2.Distance(lineStart, lineEnd);
            return (bottom == 0) ? Vector2.Distance(p, lineStart) : (area / bottom * 2.0f);
        }
    }
}