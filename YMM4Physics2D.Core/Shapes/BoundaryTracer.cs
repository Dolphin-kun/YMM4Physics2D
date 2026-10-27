using System.Numerics;
using System.Runtime.CompilerServices;

namespace YMM4Physics2D.Core.Shapes
{
    internal static class BoundaryTracer
    {
        private const int DirRight = 0;
        private const int DirDown = 1;
        private const int DirLeft = 2;
        private const int DirUp = 3;

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public static List<List<Vector2>> Trace(byte[] mask, int width, int height)
        {
            RemoveSaddles(mask, width, height);

            var loops = new List<List<Vector2>>();
            var visitedHorizontal = new bool[width * (height + 1)];
            var visitedVertical = new bool[(width + 1) * height];

            for (int y = 0; y < height; y++)
            {
                int row = y * width;
                int aboveRow = row - width;
                for (int x = 0; x < width; x++)
                {
                    if (mask[row + x] == 0) continue;
                    if (y > 0 && mask[aboveRow + x] != 0) continue;
                    if (visitedHorizontal[row + x]) continue;

                    loops.Add(TraceLoop(mask, width, height, x, y, visitedHorizontal, visitedVertical));
                }
            }

            return loops;
        }

        private static void RemoveSaddles(byte[] mask, int width, int height)
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                for (int y = 0; y < height - 1; y++)
                {
                    int row = y * width;
                    int nextRow = row + width;
                    for (int x = 0; x < width - 1; x++)
                    {
                        bool a = mask[row + x] != 0;
                        bool b = mask[row + x + 1] != 0;
                        bool c = mask[nextRow + x] != 0;
                        bool d = mask[nextRow + x + 1] != 0;

                        if (a && d && !b && !c)
                        {
                            mask[row + x + 1] = 1;
                            changed = true;
                        }
                        else if (b && c && !a && !d)
                        {
                            mask[row + x] = 1;
                            changed = true;
                        }
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static List<Vector2> TraceLoop(byte[] mask, int width, int height, int startX, int startY, bool[] visitedHorizontal, bool[] visitedVertical)
        {
            var points = new List<Vector2>();
            int x = startX;
            int y = startY;
            int previousDir = -1;
            int firstDir = -1;

            do
            {
                int dir = GetOutgoing(mask, width, height, x, y);
                if (dir < 0) break;

                if (firstDir < 0) firstDir = dir;
                if (dir != previousDir) points.Add(new Vector2(x, y));
                previousDir = dir;

                switch (dir)
                {
                    case DirRight:
                        visitedHorizontal[y * width + x] = true;
                        x++;
                        break;
                    case DirLeft:
                        visitedHorizontal[y * width + x - 1] = true;
                        x--;
                        break;
                    case DirDown:
                        visitedVertical[y * (width + 1) + x] = true;
                        y++;
                        break;
                    default:
                        visitedVertical[(y - 1) * (width + 1) + x] = true;
                        y--;
                        break;
                }
            }
            while (x != startX || y != startY);

            if (points.Count > 0 && previousDir == firstDir)
            {
                points.RemoveAt(0);
            }

            return points;
        }

        private static int GetOutgoing(byte[] mask, int width, int height, int x, int y)
        {
            bool topLeft = IsInside(mask, width, height, x - 1, y - 1);
            bool topRight = IsInside(mask, width, height, x, y - 1);
            bool bottomLeft = IsInside(mask, width, height, x - 1, y);
            bool bottomRight = IsInside(mask, width, height, x, y);

            if (bottomRight && !topRight) return DirRight;
            if (bottomLeft && !bottomRight) return DirDown;
            if (topLeft && !bottomLeft) return DirLeft;
            if (topRight && !topLeft) return DirUp;
            return -1;
        }

        private static bool IsInside(byte[] mask, int width, int height, int x, int y)
        {
            return x >= 0 && y >= 0 && x < width && y < height && mask[y * width + x] != 0;
        }
    }
}
