using System.Numerics;
using System.Windows.Shapes;
using Vortice.DCommon;
using Vortice.Direct2D1;
using Vortice.DXGI;
using Vortice.Mathematics;
using YMM4Physics2D.Core.Core;
using YMM4Physics2D.Core.Math;

namespace YMM4Physics2D.Core.Colliders
{
    public static class ColliderGenerator
    {
        private class ContourNode
        {
            public List<Vector2> Contour { get; set; }
            public List<ContourNode> Children { get; } = [];
            public ContourNode(List<Vector2> contour) => Contour = contour;
        }

        public static List<Vector2[]> GetVerticesFromImage(ID2D1DeviceContext context, ID2D1Image inputImage, Vector2 offset, Vector2 scale, byte alphaThreshold = 20, float simplifyTolerance = 1.5f)
        {
            if (scale == Vector2.Zero) scale = Vector2.One;

            List<List<Vector2>> polygons = GeneratePolygonsInternal(
                context,
                inputImage,
                offset,
                scale,
                alphaThreshold,
                simplifyTolerance);

            return [.. polygons.Select(p => p.ToArray())];
        }

        public static List<List<Vector2>> GeneratePolygonsInternal(ID2D1DeviceContext context, ID2D1Image inputImage, Vector2 offset, Vector2 scale, byte alphaThreshold = 20, float simplifyTolerance = 1.5f)
        {
            if (inputImage == null) return [];

            context.GetDpi(out float dpiX, out float dpiY);

            BitmapProperties1 stagingProps = new(
                new PixelFormat(Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied),
                dpiX, dpiY,
                BitmapOptions.CpuRead | BitmapOptions.CannotDraw
            );

            BitmapProperties1 targetProps = new(
                new PixelFormat(Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied),
                dpiX, dpiY,
                BitmapOptions.Target
            );

            var rect = context.GetImageLocalBounds(inputImage);
            var width = rect.Right - rect.Left;
            var height = rect.Bottom - rect.Top;

            using ID2D1Bitmap1 stagingBitmap = context.CreateBitmap(new SizeI((int)width, (int)height), IntPtr.Zero, 0, stagingProps);
            using ID2D1Bitmap1 targetBitmap = context.CreateBitmap(new SizeI((int)width, (int)height), IntPtr.Zero, 0, targetProps);

            var oldTarget = context.Target;
            context.Target = targetBitmap;
            context.BeginDraw();
            context.Clear(new Color4(0, 0, 0, 0));
            context.DrawImage(inputImage, new Vector2(-offset.X, -offset.Y));
            context.EndDraw();
            context.Target = oldTarget;

            stagingBitmap.CopyFromBitmap(targetBitmap);
            var map = stagingBitmap.Map(MapOptions.Read);

            List<List<Vector2>> allContours;
            try
            {
                allContours = ContourTracer.TraceAllContours(map.Bits, map.Pitch, (int)width, (int)height, alphaThreshold);
            }
            finally
            {
                stagingBitmap.Unmap();
            }

            if (allContours == null || allContours.Count == 0) return [];

            var nodes = allContours
                .Select(c => ContourTracer.SimplifyContour(c, simplifyTolerance))
                .Where(c => c.Count >= 3 && System.Math.Abs(CalculateArea(c)) >= 10.0f)
                .OrderByDescending(c => System.Math.Abs(CalculateArea(c)))
                .Select(c => new ContourNode(c))
                .ToList();

            if (nodes.Count == 0) return [];

            List<ContourNode> roots = [];
            foreach (var node in nodes)
            {
                bool addedToParent = false;
                for (int i = nodes.IndexOf(node) - 1; i >= 0; i--)
                {
                    if (IsPolygonInside(node.Contour, nodes[i].Contour))
                    {
                        nodes[i].Children.Add(node);
                        addedToParent = true;
                        break;
                    }
                }
                if (!addedToParent)
                {
                    roots.Add(node);
                }
            }

            List<List<Vector2>> resultPolygons = [];
            foreach (var root in roots)
            {
                ProcessNode(root, offset, scale, resultPolygons, isHole: false);
            }

            return resultPolygons;
        }

        private static void ProcessNode(ContourNode node, Vector2 offset, Vector2 scale, List<List<Vector2>> polygons, bool isHole)
        {
            if (isHole)
            {
                foreach (var child in node.Children)
                {
                    ProcessNode(child, offset, scale, polygons, isHole: false);
                }
                return;
            }

            var holes = node.Children.Select(c => c.Contour).ToList();

            if (!IsCounterClockwise(node.Contour)) node.Contour.Reverse();
            foreach (var h in holes)
            {
                if (IsCounterClockwise(h)) h.Reverse();
            }

            List<List<Vector2>> decomposed;
            if (holes.Count > 0)
            {
                decomposed = DecomposePolygonWithHoles(node.Contour, holes);
            }
            else
            {
                decomposed = PolygonTools.DecomposeConcave(node.Contour);
            }

            foreach (var poly in decomposed)
            {
                if (poly.Count < 3) continue;

                for (int i = 0; i < poly.Count; i++)
                {
                    poly[i] = poly[i] * scale;
                }

                polygons.Add(poly);
            }

            foreach (var childHole in node.Children)
            {
                foreach (var grandChild in childHole.Children)
                {
                    ProcessNode(grandChild, offset,scale, polygons, false);
                }
            }
        }

        private static List<List<Vector2>> DecomposePolygonWithHoles(List<Vector2> outer, List<List<Vector2>> holes)
        {
            if (holes.Count == 0)
            {
                return PolygonTools.DecomposeConcave(outer);
            }

            var combined = new List<Vector2>(outer);

            foreach (var hole in holes)
            {
                int bridgeOuterIdx = 0;
                int bridgeHoleIdx = 0;
                float minDist = float.MaxValue;

                for (int i = 0; i < combined.Count; i++)
                {
                    for (int j = 0; j < hole.Count; j++)
                    {
                        float dist = Vector2.Distance(combined[i], hole[j]);
                        if (dist < minDist)
                        {
                            minDist = dist;
                            bridgeOuterIdx = i;
                            bridgeHoleIdx = j;
                        }
                    }
                }

                var bridgedPoly = new List<Vector2>();

                for (int i = 0; i <= bridgeOuterIdx; i++)
                {
                    bridgedPoly.Add(combined[i]);
                }

                for (int i = 0; i < hole.Count; i++)
                {
                    int idx = (bridgeHoleIdx + i) % hole.Count;
                    bridgedPoly.Add(hole[idx]);
                }
                bridgedPoly.Add(hole[bridgeHoleIdx]);
                bridgedPoly.Add(combined[bridgeOuterIdx]);

                for (int i = bridgeOuterIdx + 1; i < combined.Count; i++)
                {
                    bridgedPoly.Add(combined[i]);
                }

                combined = bridgedPoly;
            }

            return PolygonTools.DecomposeConcave(combined);
        }

        private static bool IsPointInPolygon(Vector2 point, List<Vector2> polygon)
        {
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                if (((polygon[i].Y > point.Y) != (polygon[j].Y > point.Y)) &&
                    (point.X < (polygon[j].X - polygon[i].X) * (point.Y - polygon[i].Y) / (polygon[j].Y - polygon[i].Y) + polygon[i].X))
                {
                    inside = !inside;
                }
            }
            return inside;
        }

        private static bool IsPolygonInside(List<Vector2> inner, List<Vector2> outer)
        {
            if (inner.Count == 0) return false;
            return IsPointInPolygon(inner[0], outer);
        }

        private static bool IsCounterClockwise(List<Vector2> points)
        {
            return CalculateArea(points) > 0;
        }

        private static float CalculateArea(List<Vector2> points)
        {
            if (points.Count < 3) return 0;

            float area = 0;
            for (int i = 0; i < points.Count; i++)
            {
                Vector2 p1 = points[i];
                Vector2 p2 = points[(i + 1) % points.Count];
                area += (p1.X * p2.Y) - (p2.X * p1.Y);
            }
            return area / 2.0f;
        }
    }
}