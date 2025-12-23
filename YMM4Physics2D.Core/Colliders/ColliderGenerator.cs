using System.Diagnostics;
using System.Numerics;
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

        public static List<List<Vector2[]>> GetVerticesFromImage(ID2D1DeviceContext context, ID2D1Image inputImage, Vector2 offset, Vector2 scale, byte alphaThreshold = 20, float simplifyTolerance = 1.5f)
        {
            if (inputImage == null) return [];
            if (scale == Vector2.Zero) scale = Vector2.One;

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
            int width = (int)(rect.Right - rect.Left);
            int height = (int)(rect.Bottom - rect.Top);

            using ID2D1Bitmap1 stagingBitmap = context.CreateBitmap(new SizeI(width, height), IntPtr.Zero, 0, stagingProps);
            using ID2D1Bitmap1 targetBitmap = context.CreateBitmap(new SizeI(width, height), IntPtr.Zero, 0, targetProps);

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
                allContours = ContourTracer.TraceAllContours(map.Bits, map.Pitch, width, height, alphaThreshold);
            }
            finally
            {
                stagingBitmap.Unmap();
            }

            if (allContours == null || allContours.Count == 0) return [];

            var nodes = allContours
                .Select(c => ContourTracer.SimplifyContour(c, simplifyTolerance))
                .Where(c => c.Count >= 3 && System.Math.Abs(GeometryUtils.CalculateArea(c)) >= 10.0f)
                .OrderByDescending(c => System.Math.Abs(GeometryUtils.CalculateArea(c)))
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

            List<List<Vector2[]>> groupedPolygons = [];
            foreach (var root in roots)
            {
                List<List<Vector2>> currentShapeParts = [];
                ProcessNode(root, offset, scale, currentShapeParts, isHole: false);

                if (currentShapeParts.Count > 0)
                {
                    groupedPolygons.Add([.. currentShapeParts.Select(p => p.ToArray())]);
                }
            }

            return groupedPolygons;
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

            if (!GeometryUtils.IsCounterClockwise(node.Contour)) node.Contour.Reverse();
            foreach (var h in holes)
            {
                if (GeometryUtils.IsCounterClockwise(h)) h.Reverse();
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

            var combined = new List<Vector2>(outer.Count + holes.Sum(h => h.Count) + holes.Count * 2);
            combined.AddRange(outer);

            foreach (var hole in holes)
            {
                int bridgeOuterIdx = 0;
                int bridgeHoleIdx = 0;
                float minSqrDist = float.MaxValue;

                for (int i = 0; i < combined.Count; i++)
                {
                    Vector2 p1 = combined[i];
                    for (int j = 0; j < hole.Count; j++)
                    {
                        Vector2 p2 = hole[j];
                        float sqrDist = Vector2.DistanceSquared(p1, p2);

                        if (sqrDist < minSqrDist)
                        {
                            minSqrDist = sqrDist;
                            bridgeOuterIdx = i;
                            bridgeHoleIdx = j;
                        }
                    }
                }

                if (bridgeOuterIdx == -1) continue;

                var holeVertsToInsert = new List<Vector2>(hole.Count + 2);

                for (int i = 0; i < hole.Count; i++)
                {
                    holeVertsToInsert.Add(hole[(bridgeHoleIdx + i) % hole.Count]);
                }

                holeVertsToInsert.Add(hole[bridgeHoleIdx]);
                holeVertsToInsert.Add(combined[bridgeOuterIdx]);

                combined.InsertRange(bridgeOuterIdx + 1, holeVertsToInsert);
            }

            return PolygonTools.DecomposeConcave(combined);
        }

        private static bool IsPolygonInside(List<Vector2> inner, List<Vector2> outer)
        {
            if (inner.Count == 0) return false;
            return GeometryUtils.IsPointInPolygon(inner[0], outer);
        }
    }
}