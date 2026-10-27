using System.Numerics;
using Vortice;
using Vortice.Direct2D1;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Shapes;
using YMM4Physics2D.Core.World;

namespace YMM4Physics2D.Core.Controllers
{
    public sealed class ControllerInput
    {
        public Guid SceneId { get; init; }
        public int WorldId { get; init; }
        public WorldParameters WorldParameters { get; init; }
        public int TimelineFrame { get; init; }
        public int LocalFrame { get; init; }
        public int Length { get; init; }
        public int Layer { get; init; }
        public int GroupIndex { get; init; }
        public int InputIndex { get; init; }
        public int InputCount { get; init; } = 1;
        public bool IsPaused { get; init; }
        public BodyType BodyType { get; init; }
        public bool IsSeparate { get; init; }
        public float CollisionMargin { get; init; }
        public int FluidResolution { get; init; }
        public ShapeUpdateMode ShapeUpdateMode { get; init; }
        public float SimplifyTolerance { get; init; }
        public TrackObservation Observation { get; init; }
        public BodyParameters Parameters { get; init; }
        public required BodyParameterProvider ParameterProvider { get; init; }
        public Func<TrackSettings>? SettingsProvider { get; init; }
        public float FluidSmoothness { get; init; } = 0.6f;
        public float FluidGloss { get; init; } = 1f;
        public InterpolationMode InterpolationMode { get; init; } = InterpolationMode.Linear;
        public string Name { get; init; } = string.Empty;
    }

    public sealed partial class RigidBodyController(object owner) : IDisposable
    {
        private const int SecondaryMaxSteps = 2;
        private static readonly long ObserverRefreshTicks = System.Diagnostics.Stopwatch.Frequency / 5;

        [ThreadStatic]
        private static List<RigidBodyController>? deferredControllers;

        private readonly object owner = owner;

        private const byte AlphaThreshold = 20;

        private const float MaskMargin = 2f;

        private const int AtlasWidth = 4096;
        private const int AtlasMaxHeight = 8192;
        private const int AtlasPadding = 2;

        private readonly ShapeReader reader = new();
        private readonly TrackFrameResult result = new();

        private PhysicsWorld? world;
        private BodyTrack? track;
        private bool isObserver;
        private int observedFrame = int.MinValue;
        private ID2D1Image? observedInput;
        private long observedTimestamp;
        private (Guid SceneId, int WorldId) worldKey;
        private (int GroupIndex, int InputIndex) trackIndex;

        private ID2D1Image? lastInput;
        private RawRectF lastBounds;
        private bool shapeDirty = true;

        private ShapeVersion? maskShape;
        private ID2D1Geometry?[] masks = [];
        private ID2D1GeometryRealization?[] maskRealizations = [];
        private float maskRealizationScale;
        private RawRectF[] partBounds = [];

        private ID2D1Bitmap1? atlas;
        private RawRectF[] atlasCells = [];
        private ShapeVersion? atlasShape;
        private ID2D1Image? atlasInput;
        private bool atlasUnavailable;

        private ID2D1ImageBrush? brush;
        private ID2D1Image? brushInput;
        private InterpolationMode brushInterpolationMode;

        private ID2D1StrokeStyle? lineStrokeStyle;
        private readonly Dictionary<uint, ID2D1SolidColorBrush> lineBrushes = [];

        private ID2D1Image? cacheCheckedInput;
        private bool isInputCacheable;

        private ID2D1CommandList? commandList;
        private ID2D1Effect? outputEffect;
        private Vortice.Direct2D1.Effects.AffineTransform2D? poseEffect;
        private ID2D1Image? poseOutput;
        private ID2D1Image? output;
        private bool hasOutput;
        private bool disposed;
        private volatile bool disposeRequested;
        private readonly Lock gate = new();

        private readonly Lock deferredLock = new();
        private ID2D1DeviceContext? deferredContext;
        private ID2D1Image? deferredInput;
        private int deferredFrame;
        private InterpolationMode deferredInterpolationMode;

        public ID2D1Image? Output => hasOutput ? output : null;

        public void NotifySettingsChanged()
        {
            shapeDirty = true;
            track?.RequestParameterRefresh();
            PhysicsManager.NotifySettingsChanged();
        }

        public bool Update(ID2D1DeviceContext context, ID2D1Image input, ControllerInput data)
        {
            try
            {
                lock (gate)
                {
                    if (disposeRequested) return false;
                    return UpdateCore(context, input, data);
                }
            }
            finally
            {
                TryDisposePending();
            }
        }

        private bool UpdateCore(ID2D1DeviceContext context, ID2D1Image input, ControllerInput data)
        {
            bool isLast = data.InputIndex >= data.InputCount - 1;

            if (world == null && PhysicsManager.IsUiThread) isObserver = true;
            if (isObserver) return UpdateObserver(context, input, data);

            fluidSmoothness = data.FluidSmoothness;
            fluidGloss = data.FluidGloss;
            fluidFrameRate = data.WorldParameters.Fps;
            var (currentWorld, currentTrack) = EnsureTrack(data);
            currentWorld.SetParameters(data.WorldParameters);
            PhysicsManager.SweepIfDue();

            if (currentWorld.IsSecondary(currentTrack, this))
            {
                PhysicsManager.SetLastUpdate(data.SceneId, currentWorld, currentTrack, true);
                bool updated = UpdateSecondary(context, input, data, currentWorld, currentTrack);
                if (isLast) FlushDeferred(currentWorld, data.GroupIndex, data.TimelineFrame);
                return updated;
            }
            PhysicsManager.SetLastUpdate(data.SceneId, currentWorld, currentTrack, false);

            var shape = CaptureShape(context, input, data, currentWorld, currentTrack, out bool replaced);

            currentWorld.UpdateTrack(currentTrack, new TrackUpdate
            {
                StartFrame = data.TimelineFrame - data.LocalFrame,
                Length = data.Length,
                Layer = data.Layer,
                LocalFrame = data.LocalFrame,
                BodyType = data.BodyType,
                IsSeparate = data.IsSeparate,
                CollisionMargin = data.CollisionMargin,
                FluidResolution = data.FluidResolution,
                ShapeUpdateMode = data.ShapeUpdateMode,
                Observation = data.Observation,
                Parameters = data.Parameters,
                ParameterProvider = data.ParameterProvider,
                SettingsProvider = data.SettingsProvider,
                Shape = shape,
                ShapeReplaced = replaced,
                Name = data.Name,
            }, this);

            if (!isLast && currentWorld.NeedsSimulation(currentTrack, this, data.TimelineFrame))
            {
                Defer(context, input, data);
                return true;
            }

            bool rendered = Render(context, input, data.TimelineFrame, data.InterpolationMode);
            if (isLast) FlushDeferred(currentWorld, data.GroupIndex, data.TimelineFrame);
            return rendered;
        }

        private bool UpdateSecondary(ID2D1DeviceContext context, ID2D1Image input, ControllerInput data, PhysicsWorld currentWorld, BodyTrack currentTrack)
        {
            currentWorld.TryUpdateSecondary(currentTrack, new TrackUpdate
            {
                StartFrame = data.TimelineFrame - data.LocalFrame,
                Length = data.Length,
                Layer = data.Layer,
                LocalFrame = data.LocalFrame,
                BodyType = data.BodyType,
                IsSeparate = data.IsSeparate,
                CollisionMargin = data.CollisionMargin,
                FluidResolution = data.FluidResolution,
                ShapeUpdateMode = data.ShapeUpdateMode,
                Observation = data.Observation,
                Parameters = data.Parameters,
                ParameterProvider = data.ParameterProvider,
                Name = data.Name,
            });

            if (!currentWorld.TryGetLatestResult(currentTrack, data.TimelineFrame, result, SecondaryMaxSteps)) return hasOutput;

            SetOutput(context, Compose(context, input, data.InterpolationMode));
            hasOutput = true;
            return true;
        }

        private bool UpdateObserver(ID2D1DeviceContext context, ID2D1Image input, ControllerInput data)
        {
            var newWorldKey = (data.SceneId, data.WorldId);
            var newTrackIndex = (data.GroupIndex, data.InputIndex);
            if (world == null || track == null || worldKey != newWorldKey || trackIndex != newTrackIndex)
            {
                if (world != null && track != null) world.Detach(track, this, remove: false);

                world = null;
                track = null;
                worldKey = newWorldKey;
                trackIndex = newTrackIndex;
                observedFrame = int.MinValue;

                if (PhysicsManager.TryAttachObserver(data.SceneId, data.WorldId, new TrackKey(owner, data.GroupIndex, data.InputIndex), this) is not { } attached) return false;
                (world, track) = attached;
            }

            PhysicsManager.SetLastUpdate(data.SceneId, world, track, true);

            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (hasOutput && observedFrame == data.TimelineFrame &&
                (ReferenceEquals(observedInput, input) || now - observedTimestamp < ObserverRefreshTicks))
            {
                return true;
            }

            if (!world.TryGetLatestResult(track, data.TimelineFrame, result, 0)) return hasOutput;

            SetOutput(context, Compose(context, input, data.InterpolationMode));
            hasOutput = true;
            observedFrame = data.TimelineFrame;
            observedInput = input;
            observedTimestamp = now;
            return true;
        }

        public void Skip(ControllerInput data)
        {
            try
            {
                lock (gate)
                {
                    if (disposeRequested) return;
                    if (data.InputIndex >= data.InputCount - 1 && world != null) FlushDeferred(world, data.GroupIndex, data.TimelineFrame);
                }
            }
            finally
            {
                TryDisposePending();
            }
        }

        private void Defer(ID2D1DeviceContext context, ID2D1Image input, ControllerInput data)
        {
            ReleaseDeferred();
            lock (deferredLock)
            {
                deferredContext = context;
                deferredInput = input.QueryInterface<ID2D1Image>();
            }
            deferredFrame = data.TimelineFrame;
            deferredInterpolationMode = data.InterpolationMode;

            if (!hasOutput)
            {
                SetOutput(context, input);
                hasOutput = true;
            }

            var list = deferredControllers ??= [];
            if (!list.Contains(this)) list.Add(this);
        }

        private void FlushDeferred(PhysicsWorld currentWorld, int groupIndex, int frame)
        {
            var list = deferredControllers;
            if (list == null || list.Count == 0) return;

            for (int i = list.Count - 1; i >= 0; i--)
            {
                var controller = list[i];
                if (controller.disposed)
                {
                    list.RemoveAt(i);
                    continue;
                }
                if (controller.world != currentWorld || !ReferenceEquals(controller.owner, owner) || controller.trackIndex.GroupIndex != groupIndex) continue;

                list.RemoveAt(i);
                if (controller.deferredFrame == frame) controller.RenderDeferred();
                else controller.ReleaseDeferred();
            }
        }

        private void RenderDeferred()
        {
            if (!gate.TryEnter())
            {
                ReleaseDeferred();
                return;
            }
            try
            {
                RenderDeferredCore();
            }
            finally
            {
                gate.Exit();
            }
            TryDisposePending();
        }

        private void RenderDeferredCore()
        {
            ID2D1DeviceContext? context;
            ID2D1Image? input;
            lock (deferredLock)
            {
                context = deferredContext;
                input = deferredInput;
                deferredContext = null;
                deferredInput = null;
            }
            if (disposeRequested || context == null || input == null)
            {
                input?.Dispose();
                return;
            }

            try
            {
                Render(context, input, deferredFrame, deferredInterpolationMode);
            }
            finally
            {
                input.Dispose();
            }
        }

        private void ReleaseDeferred()
        {
            ID2D1Image? input;
            lock (deferredLock)
            {
                input = deferredInput;
                deferredInput = null;
                deferredContext = null;
            }
            input?.Dispose();
        }

        private bool Render(ID2D1DeviceContext context, ID2D1Image input, int frame, InterpolationMode interpolationMode)
        {
            var currentWorld = world;
            var currentTrack = track;
            if (currentWorld == null || currentTrack == null || !currentWorld.TryGetResult(currentTrack, frame, result))
            {
                SetOutput(context, input);
                hasOutput = false;
                return false;
            }

            SetOutput(context, Compose(context, input, interpolationMode));
            hasOutput = true;
            return true;
        }

        private void SetOutput(ID2D1DeviceContext context, ID2D1Image image)
        {
            if (outputEffect == null)
            {
                outputEffect = new Vortice.Direct2D1.Effects.Crop(context);
                output = outputEffect.Output;
            }
            outputEffect.SetInput(0, image, true);
        }

        private (PhysicsWorld World, BodyTrack Track) EnsureTrack(ControllerInput data)
        {
            var newWorldKey = (data.SceneId, data.WorldId);
            var newTrackIndex = (data.GroupIndex, data.InputIndex);

            if (world != null && track != null && worldKey == newWorldKey && trackIndex == newTrackIndex)
            {
                return (world, track);
            }

            if (world != null && track != null)
            {
                world.Detach(track, this, remove: worldKey != newWorldKey);
            }

            var (newWorld, newTrack) = PhysicsManager.Attach(data.SceneId, data.WorldId, new TrackKey(owner, data.GroupIndex, data.InputIndex), this);
            world = newWorld;
            track = newTrack;
            worldKey = newWorldKey;
            trackIndex = newTrackIndex;
            shapeDirty = true;

            return (newWorld, newTrack);
        }

        private ShapeVersion? CaptureShape(ID2D1DeviceContext context, ID2D1Image input, ControllerInput data, PhysicsWorld currentWorld, BodyTrack currentTrack, out bool replaced)
        {
            replaced = false;

            bool everyFrame = data.ShapeUpdateMode == ShapeUpdateMode.EveryFrame;
            bool hasShape = currentWorld.HasShape(currentTrack);
            if (!everyFrame && !data.IsPaused && hasShape && !shapeDirty && data.LocalFrame != 0) return null;

            var bounds = context.GetImageLocalBounds(input);
            bool inputChanged = !ReferenceEquals(input, lastInput);
            bool boundsChanged = !bounds.Equals(lastBounds);

            bool needed = shapeDirty
                || inputChanged
                || boundsChanged
                || data.LocalFrame == 0
                || !hasShape
                || (everyFrame && !IsInputCacheable(input));
            if (!needed) return null;

            replaced = shapeDirty || (data.IsPaused && (inputChanged || boundsChanged));

            lastInput = input;
            lastBounds = bounds;
            shapeDirty = false;

            if (!reader.Read(context, input, AlphaThreshold)) return ShapeVersion.Empty;

            ulong hash = reader.Hash ^ ((ulong)BitConverter.SingleToUInt32Bits(data.SimplifyTolerance) * 0xC2B2AE3D27D4EB4FUL);

            return currentWorld.FindShape(currentTrack, hash)
                ?? ShapeAnalyzer.Analyze(reader.Mask, reader.Width, reader.Height, reader.Origin, reader.PixelSize, data.SimplifyTolerance, hash);
        }

        private ID2D1Image Compose(ID2D1DeviceContext context, ID2D1Image input, InterpolationMode interpolationMode)
        {
            if (result.FluidShape != null)
            {
                DrawFluid(context, input, interpolationMode, fluidSmoothness);
                return commandList!;
            }


            if (result.IsSeparate || result.Poses.Count != 1 || result.JointLines.Count > 0 || result.Poses[0].Opacity < 1f)
            {
                Draw(context, input, interpolationMode);
                return commandList!;
            }

            if (poseEffect == null)
            {
                poseEffect = new Vortice.Direct2D1.Effects.AffineTransform2D(context);
                poseOutput = poseEffect.Output;
            }

            poseEffect.SetInput(0, input, true);
            poseEffect.TransformMatrix = GetPoseTransform(0);
            poseEffect.InterPolationMode = (AffineTransform2DInterpolationMode)interpolationMode;
            return poseOutput!;
        }

        private Matrix3x2 GetPoseTransform(int index)
        {
            var pose = result.Poses[index];
            return result.ImageTransform
                * Matrix3x2.CreateTranslation(-result.LocalCentroids[index])
                * Matrix3x2.CreateRotation(pose.Rotation)
                * Matrix3x2.CreateTranslation(pose.Position);
        }

        private void Draw(ID2D1DeviceContext context, ID2D1Image input, InterpolationMode interpolationMode)
        {
            commandList?.Dispose();
            commandList = context.CreateCommandList();

            bool useAtlas = false;
            using var realizer = result.IsSeparate ? context.QueryInterfaceOrNull<ID2D1DeviceContext1>() : null;
            if (result.IsSeparate)
            {
                EnsureMasks(context, result.Shape);
                useAtlas = IsInputCacheable(input) && EnsureAtlas(context, input, result.Shape);
                if (!useAtlas)
                {
                    EnsureBrush(context, input, interpolationMode);
                    if (realizer != null) EnsureMaskRealizations(realizer, MathF.Sqrt(MathF.Abs(result.ImageTransform.GetDeterminant())));
                }
            }

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

                        for (int i = 0; i < result.Poses.Count; i++)
                        {
                            var pose = result.Poses[i];
                            if (pose.Opacity <= 0f) continue;
                            if (result.IsSeparate && i >= masks.Length) break;

                            bool faded = pose.Opacity < 1f;
                            if (faded)
                            {
                                context.Transform = Matrix3x2.Identity;
                                context.PushLayer(new LayerParameters1
                                {
                                    ContentBounds = new RawRectF(float.MinValue, float.MinValue, float.MaxValue, float.MaxValue),
                                    MaskTransform = Matrix3x2.Identity,
                                    MaskAntialiasMode = AntialiasMode.PerPrimitive,
                                    Opacity = pose.Opacity,
                                    LayerOptions = LayerOptions1.None,
                                }, null!);
                            }

                            context.Transform = GetPoseTransform(i);

                            if (!result.IsSeparate)
                            {
                                context.DrawImage(input, interpolationMode, CompositeMode.SourceOver);
                                if (faded) context.PopLayer();
                                continue;
                            }

                            if (masks[i] is { } mask)
                            {
                                var bounds = partBounds[i];
                                if (useAtlas)
                                {
                                    context.Transform = Matrix3x2.CreateTranslation(bounds.Left, bounds.Top) * context.Transform;
                                    context.DrawImage(atlas!, null, atlasCells[i], interpolationMode, CompositeMode.SourceOver);
                                }
                                else
                                {
                                    brush!.SourceRectangle = bounds;
                                    brush.Transform = Matrix3x2.CreateTranslation(bounds.Left, bounds.Top);
                                    if (realizer != null && i < maskRealizations.Length && maskRealizations[i] is { } realization)
                                    {
                                        realizer.DrawGeometryRealization(realization, brush);
                                    }
                                    else
                                    {
                                        context.FillGeometry(mask, brush);
                                    }
                                }
                            }

                            if (faded) context.PopLayer();
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

        private void DrawJointLines(ID2D1DeviceContext context)
        {
            if (result.JointLines.Count == 0) return;

            context.Transform = Matrix3x2.Identity;

            if (lineStrokeStyle == null)
            {
                using var factory = context.Factory;
                lineStrokeStyle = factory.CreateStrokeStyle(new StrokeStyleProperties
                {
                    StartCap = CapStyle.Round,
                    EndCap = CapStyle.Round,
                    LineJoin = LineJoin.Round,
                });
            }

            foreach (var line in result.JointLines)
            {
                if (!lineBrushes.TryGetValue(line.Color, out var brush))
                {
                    var color = new Vortice.Mathematics.Color4(
                        ((line.Color >> 16) & 0xFF) / 255f,
                        ((line.Color >> 8) & 0xFF) / 255f,
                        (line.Color & 0xFF) / 255f,
                        ((line.Color >> 24) & 0xFF) / 255f);
                    brush = context.CreateSolidColorBrush(color);
                    lineBrushes[line.Color] = brush;
                }

                context.DrawLine(line.Start, line.End, brush, line.Width, lineStrokeStyle);
            }
        }

        private void EnsureMasks(ID2D1DeviceContext context, ShapeVersion shape)
        {
            if (ReferenceEquals(maskShape, shape)) return;

            DisposeMasks();

            using var factory = context.Factory;
            masks = new ID2D1Geometry?[shape.Components.Length];
            partBounds = new RawRectF[shape.Components.Length];
            for (int i = 0; i < masks.Length; i++)
            {
                masks[i] = CreateMask(factory, shape.Components[i]);
                partBounds[i] = GetBounds(shape.Components[i]);
            }
            maskShape = shape;
        }

        private static RawRectF GetBounds(ShapeComponent component)
        {
            Vector2 min = new(float.MaxValue);
            Vector2 max = new(float.MinValue);
            foreach (var loop in component.Outline)
            {
                foreach (var point in loop)
                {
                    min = Vector2.Min(min, point);
                    max = Vector2.Max(max, point);
                }
            }

            float margin = MaskMargin + 1f;
            return new RawRectF(MathF.Floor(min.X - margin), MathF.Floor(min.Y - margin), MathF.Ceiling(max.X + margin), MathF.Ceiling(max.Y + margin));
        }

        private bool IsInputCacheable(ID2D1Image input)
        {
            if (!ReferenceEquals(cacheCheckedInput, input))
            {
                using var commandListInput = input.QueryInterfaceOrNull<ID2D1CommandList>();
                isInputCacheable = commandListInput != null;
                cacheCheckedInput = input;
            }
            return isInputCacheable;
        }

        private void EnsureBrush(ID2D1DeviceContext context, ID2D1Image input, InterpolationMode interpolationMode)
        {
            if (brush != null && ReferenceEquals(brushInput, input) && brushInterpolationMode == interpolationMode) return;

            brush?.Dispose();
            brush = context.CreateImageBrush(input, new ImageBrushProperties
            {
                SourceRectangle = new RawRectF(0, 0, 1, 1),
                ExtendModeX = ExtendMode.Clamp,
                ExtendModeY = ExtendMode.Clamp,
                InterpolationMode = interpolationMode,
            });
            brushInput = input;
            brushInterpolationMode = interpolationMode;
        }

        private bool EnsureAtlas(ID2D1DeviceContext context, ID2D1Image input, ShapeVersion shape)
        {
            if (ReferenceEquals(atlasShape, shape) && ReferenceEquals(atlasInput, input))
            {
                return atlas != null && !atlasUnavailable;
            }

            atlas?.Dispose();
            atlas = null;
            atlasShape = shape;
            atlasInput = input;
            atlasUnavailable = true;

            var cells = new RawRectF[partBounds.Length];
            int x = 0, y = 0, rowHeight = 0;
            for (int i = 0; i < partBounds.Length; i++)
            {
                int width = (int)(partBounds[i].Right - partBounds[i].Left);
                int height = (int)(partBounds[i].Bottom - partBounds[i].Top);
                if (width > AtlasWidth) return false;

                if (x + width > AtlasWidth)
                {
                    x = 0;
                    y += rowHeight + AtlasPadding;
                    rowHeight = 0;
                }

                cells[i] = new RawRectF(x, y, x + width, y + height);
                x += width + AtlasPadding;
                rowHeight = Math.Max(rowHeight, height);
            }

            int atlasHeight = y + rowHeight;
            if (cells.Length == 0 || atlasHeight <= 0 || atlasHeight > AtlasMaxHeight) return false;

            int atlasWidth = Math.Min(AtlasWidth, (int)cells.Max(c => c.Right));
            var format = new Vortice.DCommon.PixelFormat(Vortice.DXGI.Format.B8G8R8A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied);
            atlas = context.CreateBitmap(new Vortice.Mathematics.SizeI(atlasWidth, atlasHeight), new BitmapProperties1(format, 96f, 96f, BitmapOptions.Target));
            atlasCells = cells;

            EnsureBrush(context, input, InterpolationMode.NearestNeighbor);

            using (var oldTarget = context.Target)
            {
                var oldTransform = context.Transform;
                context.Target = atlas;
                try
                {
                    context.BeginDraw();
                    try
                    {
                        context.Clear(new Vortice.Mathematics.Color4(0, 0, 0, 0));
                        for (int i = 0; i < cells.Length; i++)
                        {
                            var mask = masks[i];
                            if (mask == null) continue;

                            var bounds = partBounds[i];
                            context.Transform = Matrix3x2.CreateTranslation(cells[i].Left - bounds.Left, cells[i].Top - bounds.Top);
                            brush!.SourceRectangle = bounds;
                            brush.Transform = Matrix3x2.CreateTranslation(bounds.Left, bounds.Top);
                            context.FillGeometry(mask, brush);
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

            atlasUnavailable = false;
            return true;
        }

        private static ID2D1Geometry? CreateMask(ID2D1Factory factory, ShapeComponent component)
        {
            var loops = component.Outline.Where(loop => loop.Length >= 3).ToArray();
            if (loops.Length == 0) return null;

            using var fill = factory.CreatePathGeometry();
            using (var sink = fill.Open())
            {
                sink.SetFillMode(FillMode.Alternate);
                foreach (var loop in loops)
                {
                    sink.BeginFigure(loop[0], FigureBegin.Filled);
                    sink.AddLines(loop[1..]);
                    sink.EndFigure(FigureEnd.Closed);
                }
                sink.Close();
            }

            using var widened = factory.CreatePathGeometry();
            using (var sink = widened.Open())
            {
                fill.Widen(MaskMargin * 2f, sink);
                sink.Close();
            }

            var mask = factory.CreatePathGeometry();
            using (var sink = mask.Open())
            {
                fill.CombineWithGeometry(widened, CombineMode.Union, sink);
                sink.Close();
            }

            return mask;
        }

        private void EnsureMaskRealizations(ID2D1DeviceContext1 context, float scale)
        {
            scale = MathF.Max(scale, 1e-3f);
            if (maskRealizations.Length == masks.Length && maskRealizationScale > 0f && MathF.Abs(scale / maskRealizationScale - 1f) < 0.25f) return;

            DisposeMaskRealizations();
            maskRealizations = new ID2D1GeometryRealization?[masks.Length];
            for (int i = 0; i < masks.Length; i++)
            {
                if (masks[i] is { } mask) maskRealizations[i] = context.CreateFilledGeometryRealization(mask, 0.25f / scale);
            }
            maskRealizationScale = scale;
        }

        private void DisposeMaskRealizations()
        {
            foreach (var realization in maskRealizations) realization?.Dispose();
            maskRealizations = [];
            maskRealizationScale = 0f;
        }

        private void DisposeMasks()
        {
            DisposeMaskRealizations();
            foreach (var mask in masks) mask?.Dispose();
            masks = [];
            maskShape = null;
        }

        public void Dispose()
        {
            disposeRequested = true;
            TryDisposePending();
        }

        private void TryDisposePending()
        {
            if (!disposeRequested || disposed || !gate.TryEnter()) return;
            try
            {
                DisposeCore();
            }
            finally
            {
                gate.Exit();
            }
        }

        private void DisposeCore()
        {
            if (disposed) return;
            disposed = true;

            if (world != null && track != null)
            {
                world.Detach(track, this, remove: false);
            }
            world = null;
            track = null;

            DisposeMasks();
            DisposeFluid();
            atlas?.Dispose();
            atlas = null;
            brush?.Dispose();
            brush = null;
            lineStrokeStyle?.Dispose();
            lineStrokeStyle = null;
            foreach (var lineBrush in lineBrushes.Values) lineBrush.Dispose();
            lineBrushes.Clear();
            reader.Dispose();

            deferredControllers?.Remove(this);
            ReleaseDeferred();

            poseOutput?.Dispose();
            poseOutput = null;
            poseEffect?.Dispose();
            poseEffect = null;
            output?.Dispose();
            output = null;
            outputEffect?.Dispose();
            outputEffect = null;

            commandList?.Dispose();
            commandList = null;
        }
    }
}
