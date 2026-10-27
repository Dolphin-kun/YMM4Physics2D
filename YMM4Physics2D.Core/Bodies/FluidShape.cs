using System.Numerics;
using YMM4Physics2D.Core.Shapes;

namespace YMM4Physics2D.Core.Bodies
{
    public sealed class FluidShape
    {
        public required Vector2[] RestLocal { get; init; }
        public required Vector2[] RestImage { get; init; }
        public required float Spacing { get; init; }
        public required float Radius { get; init; }
        public required float DrawRadius { get; init; }
        public required Vector2 ImageMin { get; init; }
        public required Vector2 ImageMax { get; init; }

        public int Count => RestLocal.Length;
    }

    public static class FluidSampler
    {
        public static FluidShape? Build(ShapeVersion shape, Matrix3x2 imageTransform, int targetCount)
        {
            if (shape.IsEmpty || !Matrix3x2.Invert(imageTransform, out var toImage)) return null;

            float scale = MathF.Sqrt(MathF.Abs(imageTransform.GetDeterminant()));
            if (scale < 1e-4f) return null;

            var loops = new List<Vector2[]>();
            float area = 0f;
            foreach (var component in shape.Components)
            {
                area += component.Area;
                foreach (var loop in component.Outline)
                {
                    if (loop.Length < 3) continue;
                    var transformed = new Vector2[loop.Length];
                    for (int i = 0; i < loop.Length; i++) transformed[i] = Vector2.Transform(loop[i], imageTransform);
                    loops.Add(transformed);
                }
            }
            if (loops.Count == 0) return null;

            area *= scale * scale;
            float spacing = Math.Clamp(MathF.Sqrt(area / Math.Max(targetCount, 4)), 3f, 200f);

            var positions = new List<Vector2>();
            var contour = new List<Vector2>();

            foreach (var loop in loops)
            {
                var samples = Resample(loop, spacing);
                if (samples.Count < 3) continue;

                var normals = new Vector2[samples.Count];
                int insideVotes = 0;
                for (int i = 0; i < samples.Count; i++)
                {
                    Vector2 tangent = samples[(i + 1) % samples.Count] - samples[(i + samples.Count - 1) % samples.Count];
                    normals[i] = tangent.LengthSquared() > 1e-10f ? Vector2.Normalize(new Vector2(tangent.Y, -tangent.X)) : Vector2.UnitX;
                    if (IsInside(loops, samples[i] + normals[i] * 0.75f)) insideVotes++;
                }
                if (insideVotes * 2 > samples.Count)
                {
                    for (int i = 0; i < normals.Length; i++) normals[i] = -normals[i];
                }

                for (int i = 0; i < samples.Count; i++)
                {
                    contour.Add(samples[i]);
                    positions.Add(samples[i] - normals[i] * spacing * 0.5f);
                }
            }
            if (positions.Count == 0) return null;

            Vector2 min = new(float.MaxValue), max = new(float.MinValue);
            foreach (var point in contour)
            {
                min = Vector2.Min(min, point);
                max = Vector2.Max(max, point);
            }

            float clearance = spacing * 1.35f;
            float rowHeight = spacing * 0.8660254f;
            int row = 0;
            for (float y = min.Y + rowHeight * 0.5f; y < max.Y; y += rowHeight, row++)
            {
                float shift = (row & 1) == 0 ? 0f : spacing * 0.5f;
                for (float x = min.X + spacing * 0.5f + shift; x < max.X; x += spacing)
                {
                    var point = new Vector2(x, y);
                    if (!IsInside(loops, point) || DistanceToPoints(contour, point) < clearance) continue;
                    positions.Add(point);
                }
            }

            var restLocal = positions.ToArray();
            var restImage = new Vector2[restLocal.Length];
            for (int i = 0; i < restLocal.Length; i++) restImage[i] = Vector2.Transform(restLocal[i], toImage);

            float drawRadius = spacing * 0.6f / scale;
            Vector2 imageMin = new(float.MaxValue), imageMax = new(float.MinValue);
            foreach (var point in restImage)
            {
                imageMin = Vector2.Min(imageMin, point - new Vector2(drawRadius));
                imageMax = Vector2.Max(imageMax, point + new Vector2(drawRadius));
            }

            return new FluidShape
            {
                RestLocal = restLocal,
                RestImage = restImage,
                Spacing = spacing,
                Radius = spacing * 0.45f,
                DrawRadius = drawRadius,
                ImageMin = imageMin,
                ImageMax = imageMax,
            };
        }

        private static List<Vector2> Resample(Vector2[] loop, float spacing)
        {
            float perimeter = 0f;
            for (int i = 0; i < loop.Length; i++) perimeter += Vector2.Distance(loop[i], loop[(i + 1) % loop.Length]);

            var samples = new List<Vector2>();
            int count = (int)MathF.Round(perimeter / spacing);
            if (count < 3) return samples;

            float step = perimeter / count;
            float travelled = 0f;
            float nextSample = 0f;
            for (int i = 0; i < loop.Length && samples.Count < count; i++)
            {
                Vector2 a = loop[i];
                Vector2 b = loop[(i + 1) % loop.Length];
                float length = Vector2.Distance(a, b);
                while (nextSample <= travelled + length && samples.Count < count)
                {
                    float t = length > 0f ? (nextSample - travelled) / length : 0f;
                    samples.Add(Vector2.Lerp(a, b, t));
                    nextSample += step;
                }
                travelled += length;
            }
            return samples;
        }

        private static bool IsInside(List<Vector2[]> loops, Vector2 point)
        {
            bool inside = false;
            foreach (var loop in loops)
            {
                for (int i = 0, j = loop.Length - 1; i < loop.Length; j = i++)
                {
                    Vector2 a = loop[i], b = loop[j];
                    if ((a.Y > point.Y) != (b.Y > point.Y) &&
                        point.X < (b.X - a.X) * (point.Y - a.Y) / (b.Y - a.Y) + a.X)
                    {
                        inside = !inside;
                    }
                }
            }
            return inside;
        }

        private static float DistanceToPoints(List<Vector2> points, Vector2 point)
        {
            float best = float.MaxValue;
            foreach (var p in points) best = MathF.Min(best, Vector2.DistanceSquared(p, point));
            return MathF.Sqrt(best);
        }
    }
}
