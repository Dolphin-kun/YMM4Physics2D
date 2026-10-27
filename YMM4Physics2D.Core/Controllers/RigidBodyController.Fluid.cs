using System.Numerics;
using System.Runtime.CompilerServices;
using Vortice;
using Vortice.Direct2D1;
using Vortice.Direct2D1.Effects;
using YMM4Physics2D.Core.Bodies;

namespace YMM4Physics2D.Core.Controllers
{
    public sealed partial class RigidBodyController
    {
        private const int InputTextureMaxSize = 8192;
        private const int FluidSurfaceMaxSize = 4096;
        private const int FluidSpriteSize = 128;
        private const int FluidSpriteStops = 12;
        private const float FluidRenderScale = 0.5f;
        private const float FluidFieldPeak = 0.4f;
        private const float FluidDropletRadius = 0.55f;
        private const float FluidColorRadius = 1.3f;
        private const float FluidHighlightAzimuth = 225f;
        private const float FluidHighlightElevation = 40f;
        private const float FluidHighlightExponent = 12f;
        private const float FluidHighlightStrength = 1.1f;

        private ID2D1Bitmap1? fluidField;
        private ID2D1Bitmap1? fluidColor;
        private Vortice.Mathematics.SizeI fluidLowSize;
        private ID2D1Bitmap1? fluidSurface;
        private Vortice.Mathematics.SizeI fluidSurfaceSize;
        private ID2D1Bitmap1? fluidSprite;
        private ID2D1SpriteBatch? fluidSpriteBatch;
        private Vortice.Mathematics.Color4[] fluidSpriteColors = [];
        private Matrix3x2[] fluidSpriteTransforms = [];
        private RawRectF[] fluidSpriteTargets = [];
        private RawRect[] fluidSpriteSources = [];
        private AffineTransform2D? fluidFieldScale;
        private GaussianBlur? fluidFieldBlur;
        private ColorMatrix? fluidFieldMask;
        private ColorMatrix? fluidHeight;
        private GaussianBlur? fluidHeightBlur;
        private DistantSpecular? fluidHighlight;
        private AffineTransform2D? fluidShadedScale;
        private GaussianBlur? fluidColorBlur;
        private ColorMatrix? fluidColorFill;
        private Composite? fluidShaded;
        private Composite? fluidClip;
        private ID2D1Image? fluidOutput;
        private RawRectF fluidSurfaceRect;
        private ID2D1Bitmap1? inputTexture;
        private RawRectF inputTextureRect;
        private ID2D1Image? inputTextureSource;
        private ID2D1BitmapBrush1? inputBrush;
        private InterpolationMode inputBrushInterpolation;

        private float fluidSmoothness = 0.6f;
        private float fluidGloss = 1f;
        private float fluidFrameRate = 60f;

        private void DrawFluid(ID2D1DeviceContext context, ID2D1Image input, InterpolationMode interpolationMode, float smoothness)
        {
            var fluid = result.FluidShape!;
            commandList?.Dispose();
            commandList = context.CreateCommandList();

            bool ready = EnsureInputTexture(context, input, fluid.ImageMin, fluid.ImageMax, interpolationMode) && result.FluidPositions.Count == fluid.Count;

            float opacity = result.Poses.Count > 0 ? result.Poses[0].Opacity : 1f;
            bool hasSurface = ready && smoothness > 0f && RenderFluidSurface(context, fluid, smoothness);

            using (var oldTarget = context.Target)
            {
                var oldTransform = context.Transform;
                context.Target = commandList;
                try
                {
                    context.BeginDraw();
                    try
                    {
                        DrawJointLines(context);

                        if (ready && opacity > 0f)
                        {
                            bool faded = opacity < 1f;
                            if (faded)
                            {
                                context.Transform = Matrix3x2.Identity;
                                context.PushLayer(new LayerParameters1
                                {
                                    ContentBounds = new RawRectF(float.MinValue, float.MinValue, float.MaxValue, float.MaxValue),
                                    MaskTransform = Matrix3x2.Identity,
                                    MaskAntialiasMode = AntialiasMode.PerPrimitive,
                                    Opacity = opacity,
                                    LayerOptions = LayerOptions1.None,
                                }, null!);
                            }

                            if (hasSurface)
                            {
                                var rect = fluidSurfaceRect;
                                context.Transform = Matrix3x2.CreateTranslation(rect.Left, rect.Top);
                                context.DrawImage(fluidSurface!, null, new RawRectF(0, 0, rect.Right - rect.Left, rect.Bottom - rect.Top), InterpolationMode.NearestNeighbor, CompositeMode.SourceOver);
                            }
                            else
                            {
                                DrawFluidParticles(context, fluid, fluid.DrawRadius, Matrix3x2.Identity);
                            }

                            if (faded)
                            {
                                context.Transform = Matrix3x2.Identity;
                                context.PopLayer();
                            }
                        }
                    }
                    finally
                    {
                        context.EndDraw();
                    }
                }
                finally
                {
                    context.Target = oldTarget;
                    context.Transform = oldTransform;
                }
            }

            commandList.Close();
        }

        private void DrawFluidParticles(ID2D1DeviceContext context, FluidShape fluid, float radius, Matrix3x2 offset)
        {
            var imageTransform = result.ImageTransform;
            var rest = fluid.RestImage;
            var positions = result.FluidPositions;
            var presences = result.FluidPresences;
            bool hasPresence = presences.Count == rest.Length;
            for (int i = 0; i < rest.Length; i++)
            {
                float particleRadius = hasPresence ? radius * presences[i] : radius;
                if (particleRadius <= 0f) continue;

                var placement = imageTransform;
                placement.Translation = imageTransform.Translation + positions[i] - Vector2.Transform(rest[i], imageTransform);
                context.Transform = placement * offset;
                context.FillEllipse(new Ellipse(rest[i], particleRadius, particleRadius), inputBrush!);
            }
        }

        private bool RenderFluidSurface(ID2D1DeviceContext context, FluidShape fluid, float smoothness)
        {
            float scale = MathF.Sqrt(MathF.Abs(result.ImageTransform.GetDeterminant()));
            float spacing = fluid.Spacing;
            float kernelRadius = spacing * (1.3f + 0.5f * smoothness);
            float blurRadius = spacing * (0.1f + 0.3f * smoothness);
            float margin = kernelRadius * 1.5f + blurRadius * 3f + 4f;

            Vector2 min = new(float.MaxValue), max = new(float.MinValue);
            foreach (var position in result.FluidPositions)
            {
                min = Vector2.Min(min, position);
                max = Vector2.Max(max, position);
            }

            int left = (int)MathF.Floor(min.X - margin);
            int top = (int)MathF.Floor(min.Y - margin);
            int width = (int)MathF.Ceiling(max.X + margin) - left;
            int height = (int)MathF.Ceiling(max.Y + margin) - top;
            if (width <= 0 || height <= 0 || width > FluidSurfaceMaxSize || height > FluidSurfaceMaxSize) return false;

            float renderScale = FluidRenderScale;
            int lowWidth = (int)MathF.Ceiling(width * renderScale);
            int lowHeight = (int)MathF.Ceiling(height * renderScale);

            EnsureFluidSprite(context);
            EnsureFluidSurfaces(context, lowWidth, lowHeight, width, height);
            EnsureFluidEffects(context);

            var offset = Matrix3x2.CreateTranslation(-left, -top) * Matrix3x2.CreateScale(renderScale);
            var positions = result.FluidPositions;
            using var batcher = context.QueryInterfaceOrNull<ID2D1DeviceContext3>();

            using (var oldTarget = context.Target)
            {
                var oldTransform = context.Transform;
                var oldAntialias = context.AntialiasMode;
                try
                {
                    context.Target = fluidField;
                    context.BeginDraw();
                    try
                    {
                        context.Clear(new Vortice.Mathematics.Color4(0, 0, 0, 0));
                        var blend = context.PrimitiveBlend;
                        context.PrimitiveBlend = PrimitiveBlend.Add;
                        int spriteCount = 0;
                        if (batcher != null) EnsureSpriteCapacity(positions.Count);
                        var velocities = result.FluidVelocities;
                        bool hasVelocity = velocities.Count == positions.Count;
                        var presences = result.FluidPresences;
                        bool hasPresence = presences.Count == positions.Count;
                        float frameTime = 1f / MathF.Max(1f, fluidFrameRate);
                        var unit = new Vortice.Mathematics.Rect(-1f, -1f, 2f, 2f);
                        for (int i = 0; i < positions.Count; i++)
                        {
                            float presence = hasPresence ? presences[i] : 1f;
                            if (presence <= 0f) continue;

                            float along = kernelRadius;
                            float across = kernelRadius;
                            var direction = Vector2.UnitX;
                            float weight = 1f;
                            if (hasVelocity)
                            {
                                Vector2 velocity = velocities[i];
                                float speed = velocity.Length();
                                float travel = speed * frameTime;
                                if (travel > spacing * 0.5f)
                                {
                                    direction = velocity / speed;
                                    along = MathF.Min(kernelRadius + travel * 0.6f, kernelRadius * 2.5f);
                                    across = kernelRadius * MathF.Max(0.75f, MathF.Sqrt(kernelRadius / along));
                                    weight = kernelRadius * kernelRadius / (along * across);
                                }
                            }

                            var placement = new Matrix3x2(direction.X * along, direction.Y * along, -direction.Y * across, direction.X * across, positions[i].X, positions[i].Y);
                            float intensity = MathF.Min(1f, MathF.Sqrt(weight)) * presence;
                            if (batcher != null)
                            {
                                fluidSpriteTransforms[spriteCount] = placement * offset;
                                fluidSpriteColors[spriteCount] = new Vortice.Mathematics.Color4(intensity, intensity, intensity, intensity);
                                spriteCount++;
                                continue;
                            }

                            context.Transform = placement * offset;
                            context.DrawBitmap(fluidSprite!, unit, intensity, InterpolationMode.Linear, null, null);
                        }

                        if (batcher != null && spriteCount > 0)
                        {
                            Array.Fill(fluidSpriteTargets, new RawRectF(-1f, -1f, 1f, 1f), 0, spriteCount);
                            DrawSprites(batcher, fluidSprite!, spriteCount, fluidSpriteTargets, null, fluidSpriteColors, fluidSpriteTransforms);
                        }
                        context.PrimitiveBlend = blend;
                    }
                    finally
                    {
                        context.EndDraw();
                    }

                    context.Target = fluidColor;
                    context.BeginDraw();
                    try
                    {
                        context.Clear(new Vortice.Mathematics.Color4(0, 0, 0, 0));
                        float colorRadius = spacing * FluidColorRadius / MathF.Max(scale, 1e-4f);
                        if (batcher != null) DrawFluidColorSprites(batcher, fluid, colorRadius, colorRadius * scale, left, top, renderScale);
                        else DrawFluidParticles(context, fluid, colorRadius, offset);
                    }
                    finally
                    {
                        context.EndDraw();
                    }

                    ConfigureFluidEffects(kernelRadius, blurRadius, spacing, renderScale);

                    context.Target = fluidSurface;
                    var controls = context.RenderingControls;
                    context.RenderingControls = controls with { BufferPrecision = BufferPrecision.PerChannel16Float };
                    context.BeginDraw();
                    try
                    {
                        context.Clear(new Vortice.Mathematics.Color4(0, 0, 0, 0));
                        context.Transform = Matrix3x2.Identity;
                        context.DrawImage(fluidOutput!, InterpolationMode.Linear, CompositeMode.SourceCopy);
                    }
                    finally
                    {
                        context.EndDraw();
                        context.RenderingControls = controls;
                    }
                }
                finally
                {
                    context.Target = oldTarget;
                    context.Transform = oldTransform;
                    context.AntialiasMode = oldAntialias;
                }
            }

            fluidSurfaceRect = new RawRectF(left, top, left + width, top + height);
            return true;
        }

        private void EnsureSpriteCapacity(int count)
        {
            if (fluidSpriteTransforms.Length >= count) return;
            int capacity = Math.Max(count, fluidSpriteTransforms.Length * 2);
            fluidSpriteColors = new Vortice.Mathematics.Color4[capacity];
            fluidSpriteTransforms = new Matrix3x2[capacity];
            fluidSpriteTargets = new RawRectF[capacity];
            fluidSpriteSources = new RawRect[capacity];
        }

        private void DrawSprites(ID2D1DeviceContext3 context, ID2D1Bitmap bitmap, int count, RawRectF[] targets, RawRect[]? sources, Vortice.Mathematics.Color4[]? colors, Matrix3x2[]? transforms)
        {
            fluidSpriteBatch ??= context.CreateSpriteBatch();
            fluidSpriteBatch.Clear();
            fluidSpriteBatch.AddSprites(
                count,
                targets,
                sources!,
                colors!,
                transforms!,
                Unsafe.SizeOf<RawRectF>(),
                sources == null ? 0 : Unsafe.SizeOf<RawRect>(),
                colors == null ? 0 : Unsafe.SizeOf<Vortice.Mathematics.Color4>(),
                transforms == null ? 0 : Unsafe.SizeOf<Matrix3x2>());

            var transform = context.Transform;
            var antialias = context.AntialiasMode;
            context.Transform = Matrix3x2.Identity;
            context.AntialiasMode = AntialiasMode.Aliased;
            context.DrawSpriteBatch(fluidSpriteBatch, 0, count, bitmap, BitmapInterpolationMode.Linear, SpriteOptions.None);
            context.AntialiasMode = antialias;
            context.Transform = transform;
        }

        private void DrawFluidColorSprites(ID2D1DeviceContext3 context, FluidShape fluid, float imageRadius, float worldRadius, int left, int top, float renderScale)
        {
            var rest = fluid.RestImage;
            var positions = result.FluidPositions;
            var presences = result.FluidPresences;
            bool hasPresence = presences.Count == rest.Length;
            EnsureSpriteCapacity(rest.Length);

            float originX = inputTextureRect.Left, originY = inputTextureRect.Top;
            int count = 0;
            for (int i = 0; i < rest.Length; i++)
            {
                float presence = hasPresence ? presences[i] : 1f;
                if (presence <= 0f) continue;

                float imageExtent = imageRadius * presence;
                float worldExtent = worldRadius * presence * renderScale;
                float x = (positions[i].X - left) * renderScale;
                float y = (positions[i].Y - top) * renderScale;
                fluidSpriteTargets[count] = new RawRectF(x - worldExtent, y - worldExtent, x + worldExtent, y + worldExtent);
                fluidSpriteSources[count] = new RawRect(
                    (int)MathF.Floor(rest[i].X - imageExtent - originX),
                    (int)MathF.Floor(rest[i].Y - imageExtent - originY),
                    (int)MathF.Ceiling(rest[i].X + imageExtent - originX),
                    (int)MathF.Ceiling(rest[i].Y + imageExtent - originY));
                count++;
            }

            if (count > 0) DrawSprites(context, inputTexture!, count, fluidSpriteTargets, fluidSpriteSources, null, null);
        }

        private void ConfigureFluidEffects(float kernelRadius, float blurRadius, float spacing, float renderScale)
        {
            var upscale = Matrix3x2.CreateScale(1f / renderScale);
            fluidFieldScale!.TransformMatrix = upscale;
            fluidShadedScale!.TransformMatrix = upscale;

            fluidFieldBlur!.StandardDeviation = MathF.Max(0.5f, blurRadius * renderScale);
            fluidColorBlur!.StandardDeviation = MathF.Max(0.6f, spacing * 0.4f * renderScale);

            float ratio = FluidDropletRadius * spacing / kernelRadius;
            float falloff = 1f - ratio * ratio;
            float level = FluidFieldPeak * falloff * falloff * falloff;
            float slope = kernelRadius / FluidFieldPeak;
            fluidFieldMask!.Matrix = new Vortice.Mathematics.Matrix5x4
            {
                M44 = slope,
                M51 = 1f,
                M52 = 1f,
                M53 = 1f,
                M54 = 0.5f - level * slope,
            };

            float heightGain = 2.5f / FluidFieldPeak;
            fluidHeight!.Matrix = new Vortice.Mathematics.Matrix5x4
            {
                M44 = heightGain,
                M51 = 1f,
                M52 = 1f,
                M53 = 1f,
                M54 = -level * 0.5f * heightGain,
            };
            fluidHeightBlur!.StandardDeviation = MathF.Max(0.8f, kernelRadius * 0.4f * renderScale);
            fluidHighlight!.SetValue(3, FluidHighlightStrength * fluidGloss);
            fluidHighlight.SetValue(4, kernelRadius * 1.4f * renderScale);
        }

        private void EnsureFluidSprite(ID2D1DeviceContext context)
        {
            if (fluidSprite != null) return;

            var format = new Vortice.DCommon.PixelFormat(Vortice.DXGI.Format.R16G16B16A16_Float, Vortice.DCommon.AlphaMode.Premultiplied);
            fluidSprite = context.CreateBitmap(new Vortice.Mathematics.SizeI(FluidSpriteSize, FluidSpriteSize), new BitmapProperties1(format, 96f, 96f, BitmapOptions.Target));

            var stops = new GradientStop[FluidSpriteStops];
            for (int i = 0; i < stops.Length; i++)
            {
                float u = i / (float)(stops.Length - 1);
                float falloff = 1f - u * u;
                float value = FluidFieldPeak * falloff * falloff * falloff;
                stops[i] = new GradientStop { Position = u, Color = new Vortice.Mathematics.Color4(value, value, value, value) };
            }

            using var stopCollection = context.CreateGradientStopCollection(stops, Gamma.Linear, ExtendMode.Clamp);
            float half = FluidSpriteSize / 2f;
            using var brush = context.CreateRadialGradientBrush(new RadialGradientBrushProperties
            {
                Center = new Vector2(half, half),
                RadiusX = half,
                RadiusY = half,
            }, stopCollection);

            using var oldTarget = context.Target;
            var oldTransform = context.Transform;
            context.Target = fluidSprite;
            try
            {
                context.BeginDraw();
                try
                {
                    context.Clear(new Vortice.Mathematics.Color4(0, 0, 0, 0));
                    context.Transform = Matrix3x2.Identity;
                    context.FillEllipse(new Ellipse(new Vector2(half, half), half, half), brush);
                }
                finally
                {
                    context.EndDraw();
                }
            }
            finally
            {
                context.Target = oldTarget;
                context.Transform = oldTransform;
            }
        }

        private void EnsureFluidSurfaces(ID2D1DeviceContext context, int lowWidth, int lowHeight, int width, int height)
        {
            if (fluidField == null || fluidLowSize.Width < lowWidth || fluidLowSize.Height < lowHeight)
            {
                fluidFieldBlur?.SetInput(0, null, true);
                fluidColorBlur?.SetInput(0, null, true);
                fluidField?.Dispose();
                fluidColor?.Dispose();

                fluidLowSize = new Vortice.Mathematics.SizeI(Math.Max(lowWidth, fluidLowSize.Width), Math.Max(lowHeight, fluidLowSize.Height));
                var fieldFormat = new Vortice.DCommon.PixelFormat(Vortice.DXGI.Format.R16G16B16A16_Float, Vortice.DCommon.AlphaMode.Premultiplied);
                fluidField = context.CreateBitmap(fluidLowSize, new BitmapProperties1(fieldFormat, 96f, 96f, BitmapOptions.Target));
                fluidColor = context.CreateBitmap(fluidLowSize, new BitmapProperties1(fieldFormat, 96f, 96f, BitmapOptions.Target));

                fluidFieldBlur?.SetInput(0, fluidField, true);
                fluidColorBlur?.SetInput(0, fluidColor, true);
            }

            if (fluidSurface == null || fluidSurfaceSize.Width < width || fluidSurfaceSize.Height < height)
            {
                fluidSurface?.Dispose();
                fluidSurfaceSize = new Vortice.Mathematics.SizeI(Math.Max(width, fluidSurfaceSize.Width), Math.Max(height, fluidSurfaceSize.Height));
                var format = new Vortice.DCommon.PixelFormat(Vortice.DXGI.Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied);
                fluidSurface = context.CreateBitmap(fluidSurfaceSize, new BitmapProperties1(format, 96f, 96f, BitmapOptions.Target));
            }
        }

        private void EnsureFluidEffects(ID2D1DeviceContext context)
        {
            if (fluidOutput != null) return;

            fluidFieldBlur = new GaussianBlur(context);
            fluidFieldBlur.SetInput(0, fluidField, true);

            fluidFieldScale = new AffineTransform2D(context) { InterPolationMode = AffineTransform2DInterpolationMode.Cubic };
            fluidFieldScale.SetInput(0, fluidFieldBlur.Output, true);

            fluidFieldMask = new ColorMatrix(context);
            fluidFieldMask.SetValue(2, true);
            fluidFieldMask.SetInput(0, fluidFieldScale.Output, true);

            fluidHeight = new ColorMatrix(context);
            fluidHeight.SetValue(2, true);
            fluidHeight.SetInput(0, fluidFieldBlur.Output, true);

            fluidHeightBlur = new GaussianBlur(context);
            fluidHeightBlur.SetInput(0, fluidHeight.Output, true);

            fluidHighlight = new DistantSpecular(context);
            fluidHighlight.SetValue(0, FluidHighlightAzimuth);
            fluidHighlight.SetValue(1, FluidHighlightElevation);
            fluidHighlight.SetValue(2, FluidHighlightExponent);
            fluidHighlight.SetValue(3, FluidHighlightStrength);
            fluidHighlight.SetInput(0, fluidHeightBlur.Output, true);

            fluidColorBlur = new GaussianBlur(context);
            fluidColorBlur.SetInput(0, fluidColor, true);

            fluidColorFill = new ColorMatrix(context);
            fluidColorFill.SetValue(2, true);
            fluidColorFill.Matrix = new Vortice.Mathematics.Matrix5x4
            {
                M11 = 1f,
                M22 = 1f,
                M33 = 1f,
                M44 = 1000f,
            };
            fluidColorFill.SetInput(0, fluidColorBlur.Output, true);

            using (var highlightOutput = fluidHighlight.Output)
            {
                fluidShaded = new Composite(context) { Mode = CompositeMode.Plus };
                fluidShaded.SetInput(0, fluidColorFill.Output, true);
                fluidShaded.SetInput(1, highlightOutput, true);
            }

            fluidShadedScale = new AffineTransform2D(context) { InterPolationMode = AffineTransform2DInterpolationMode.Linear };
            fluidShadedScale.SetInput(0, fluidShaded.Output, true);

            fluidClip = new Composite(context) { Mode = CompositeMode.DestinationIn };
            fluidClip.SetInput(0, fluidShadedScale.Output, true);
            fluidClip.SetInput(1, fluidFieldMask.Output, true);
            fluidOutput = fluidClip.Output;
        }

        private bool EnsureInputTexture(ID2D1DeviceContext context, ID2D1Image input, Vector2 imageMin, Vector2 imageMax, InterpolationMode interpolationMode)
        {
            int left = (int)MathF.Floor(imageMin.X) - 1;
            int top = (int)MathF.Floor(imageMin.Y) - 1;
            int width = (int)MathF.Ceiling(imageMax.X) + 1 - left;
            int height = (int)MathF.Ceiling(imageMax.Y) + 1 - top;
            if (width <= 0 || height <= 0 || width > InputTextureMaxSize || height > InputTextureMaxSize) return false;

            bool sizeChanged = inputTexture == null || inputTextureRect.Left != left || inputTextureRect.Top != top ||
                               inputTextureRect.Right != left + width || inputTextureRect.Bottom != top + height;
            if (!sizeChanged && ReferenceEquals(inputTextureSource, input) && IsInputCacheable(input) && inputBrushInterpolation == interpolationMode)
            {
                return true;
            }

            if (sizeChanged)
            {
                inputBrush?.Dispose();
                inputBrush = null;
                inputTexture?.Dispose();
                var format = new Vortice.DCommon.PixelFormat(Vortice.DXGI.Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied);
                inputTexture = context.CreateBitmap(new Vortice.Mathematics.SizeI(width, height), new BitmapProperties1(format, 96f, 96f, BitmapOptions.Target));
                inputTextureRect = new RawRectF(left, top, left + width, top + height);
            }

            using (var oldTarget = context.Target)
            {
                var oldTransform = context.Transform;
                context.Target = inputTexture;
                try
                {
                    context.BeginDraw();
                    try
                    {
                        context.Clear(new Vortice.Mathematics.Color4(0, 0, 0, 0));
                        context.Transform = Matrix3x2.CreateTranslation(-left, -top);
                        context.DrawImage(input, InterpolationMode.NearestNeighbor, CompositeMode.SourceOver);
                    }
                    finally
                    {
                        context.EndDraw();
                    }
                }
                finally
                {
                    context.Target = oldTarget;
                    context.Transform = oldTransform;
                }
            }

            if (inputBrush == null || inputBrushInterpolation != interpolationMode)
            {
                inputBrush?.Dispose();
                inputBrush = context.CreateBitmapBrush(inputTexture, new BitmapBrushProperties1
                {
                    ExtendModeX = ExtendMode.Clamp,
                    ExtendModeY = ExtendMode.Clamp,
                    InterpolationMode = interpolationMode,
                });
                inputBrushInterpolation = interpolationMode;
            }
            inputBrush.Transform = Matrix3x2.CreateTranslation(left, top);
            inputTextureSource = input;
            return true;
        }

        private void DisposeFluid()
        {
            fluidClip?.SetInput(0, null, true);
            fluidClip?.SetInput(1, null, true);
            fluidShadedScale?.SetInput(0, null, true);
            fluidShaded?.SetInput(0, null, true);
            fluidShaded?.SetInput(1, null, true);
            fluidColorFill?.SetInput(0, null, true);
            fluidColorBlur?.SetInput(0, null, true);
            fluidHighlight?.SetInput(0, null, true);
            fluidHeightBlur?.SetInput(0, null, true);
            fluidHeight?.SetInput(0, null, true);
            fluidFieldMask?.SetInput(0, null, true);
            fluidFieldBlur?.SetInput(0, null, true);
            fluidFieldScale?.SetInput(0, null, true);
            fluidOutput?.Dispose();
            fluidOutput = null;
            fluidClip?.Dispose();
            fluidClip = null;
            fluidShaded?.Dispose();
            fluidShaded = null;
            fluidColorFill?.Dispose();
            fluidColorFill = null;
            fluidColorBlur?.Dispose();
            fluidColorBlur = null;
            fluidShadedScale?.Dispose();
            fluidShadedScale = null;
            fluidHighlight?.Dispose();
            fluidHighlight = null;
            fluidHeightBlur?.Dispose();
            fluidHeightBlur = null;
            fluidHeight?.Dispose();
            fluidHeight = null;
            fluidFieldMask?.Dispose();
            fluidFieldMask = null;
            fluidFieldBlur?.Dispose();
            fluidFieldBlur = null;
            fluidFieldScale?.Dispose();
            fluidFieldScale = null;
            fluidSprite?.Dispose();
            fluidSprite = null;
            fluidSpriteBatch?.Dispose();
            fluidSpriteBatch = null;
            fluidSurface?.Dispose();
            fluidSurface = null;
            fluidField?.Dispose();
            fluidField = null;
            fluidColor?.Dispose();
            fluidColor = null;
            inputBrush?.Dispose();
            inputBrush = null;
            inputTexture?.Dispose();
            inputTexture = null;
            inputTextureSource = null;
        }
    }
}
