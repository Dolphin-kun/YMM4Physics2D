using System.ComponentModel;
using System.Numerics;
using Vortice.Direct2D1;
using YMM4Physics2D.Core.Controllers;
using YMM4Physics2D.Core.World;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Player.Video;

namespace YMM4Physics2D
{
    internal class PhysicsEffectProcessor : IVideoEffectProcessor
    {
        private readonly IGraphicsDevicesAndContext devices;
        private readonly PhysicsEffect item;
        private readonly RigidBodyController controller;
        private readonly BodyParameterProvider parameterProvider;
        private readonly Func<TrackSettings> settingsProvider;

        private ID2D1Image? input;
        private WorldConfig? worldConfig;
        private int worldConfigVersion = -1;
        private int worldConfigId = -1;
        private bool isPhysicsOutput;
        private bool errorLogged;

        public ID2D1Image Output => (isPhysicsOutput ? controller.Output : null) ?? input ?? throw new InvalidOperationException("入力画像が設定されていません");

        public PhysicsEffectProcessor(IGraphicsDevicesAndContext devices, PhysicsEffect item)
        {
            this.devices = devices;
            this.item = item;

            controller = new RigidBodyController(item);
            parameterProvider = GetParameters;
            settingsProvider = () => new TrackSettings(item.BodyType, item.IsSeparate, item.BodyName, null, (int)item.FluidResolution, (float)item.CollisionMargin);

            item.PropertyChanged += Item_PropertyChanged;
        }

        public DrawDescription Update(EffectDescription effectDescription)
        {
            try
            {
                return UpdateCore(effectDescription);
            }
            catch (Exception exception) when (exception is not OutOfMemoryException)
            {
                isPhysicsOutput = false;
                if (!errorLogged)
                {
                    errorLogged = true;
                    Log.Default.Write("物理演算エフェクトの処理中にエラーが発生しました", exception);
                }
                return effectDescription.DrawDescription;
            }
        }

        private DrawDescription UpdateCore(EffectDescription effectDescription)
        {
            var drawDesc = effectDescription.DrawDescription;

            if (input == null)
            {
                isPhysicsOutput = false;
                controller.Skip(new ControllerInput
                {
                    InputIndex = effectDescription.InputIndex,
                    InputCount = effectDescription.InputCount,
                    GroupIndex = effectDescription.GroupIndex,
                    ParameterProvider = parameterProvider,
                });
                return drawDesc;
            }

            int itemFrame = effectDescription.ItemPosition.Frame;
            int length = effectDescription.ItemDuration.Frame;
            int fps = effectDescription.FPS;
            int worldId = (int)item.WorldId;

            var settings = WorldSettings.WorldSettings.Default;
            int settingsVersion = settings.Version;
            if (settingsVersion != worldConfigVersion || worldId != worldConfigId)
            {
                worldConfig = settings.GetConfigSnapshot(worldId);
                worldConfigVersion = settingsVersion;
                worldConfigId = worldId;
            }

            var data = new ControllerInput
            {
                SceneId = effectDescription.SceneId,
                WorldId = worldId,
                WorldParameters = WorldParameters.Create(worldConfig, effectDescription.ScreenSize, fps),
                TimelineFrame = effectDescription.TimelinePosition.Frame,
                LocalFrame = itemFrame,
                Length = length,
                Layer = effectDescription.Layer,
                GroupIndex = effectDescription.GroupIndex,
                InputIndex = effectDescription.InputIndex,
                InputCount = effectDescription.InputCount,
                IsPaused = effectDescription.Usage == TimelineSourceUsage.Paused,
                BodyType = item.BodyType,
                IsSeparate = item.IsSeparate,
                CollisionMargin = (float)item.CollisionMargin,
                FluidResolution = (int)item.FluidResolution,
                FluidSmoothness = (float)item.FluidSmoothness.GetValue(itemFrame, length, fps) / 100f,
                FluidGloss = (float)item.FluidGloss.GetValue(itemFrame, length, fps) / 100f,
                ShapeUpdateMode = item.ShapeUpdateMode,
                SimplifyTolerance = (float)item.SimplifyTolerance.GetValue(itemFrame, length, fps),
                Observation = new TrackObservation(
                    new Vector2(drawDesc.Draw.X, drawDesc.Draw.Y),
                    drawDesc.Rotation.Z * MathF.PI / 180f,
                    drawDesc.Zoom,
                    drawDesc.Invert,
                    drawDesc.CenterPoint),
                Parameters = GetParameters(itemFrame, length, fps),
                ParameterProvider = parameterProvider,
                SettingsProvider = settingsProvider,
                InterpolationMode = drawDesc.ZoomInterpolationMode,
                Name = item.BodyName,
            };

            isPhysicsOutput = controller.Update(devices.DeviceContext, input, data);
            if (!isPhysicsOutput) return drawDesc;

            return drawDesc with
            {
                Draw = new Vector3(0, 0, drawDesc.Draw.Z),
                Zoom = Vector2.One,
                Rotation = Vector3.Zero,
                Invert = false,
            };
        }

        private BodyParameters GetParameters(int frame, int length, int fps)
        {
            bool attraction = item.IsAttractionEnabled;
            return new BodyParameters(
                (float)item.Mass.GetValue(frame, length, fps),
                (float)item.Restitution.GetValue(frame, length, fps),
                (float)item.Friction.GetValue(frame, length, fps),
                (float)item.LinearDamping.GetValue(frame, length, fps),
                (float)item.AngularDamping.GetValue(frame, length, fps),
                attraction ? (float)item.Attraction.GetValue(frame, length, fps) * 100f : 0f,
                attraction ? (float)item.AttractionRange.GetValue(frame, length, fps) : 0f,
                (float)item.FluidViscosity.GetValue(frame, length, fps) / 100f,
                (float)item.FluidCohesion.GetValue(frame, length, fps) / 100f,
                (float)item.FluidDrying.GetValue(frame, length, fps) / 100f);
        }

        private void Item_PropertyChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(PhysicsEffect.IsOpen)) return;
            controller.NotifySettingsChanged();
        }

        public void SetInput(ID2D1Image? input)
        {
            this.input = input;
        }

        public void ClearInput()
        {
            input = null;
        }

        public void Dispose()
        {
            item.PropertyChanged -= Item_PropertyChanged;
            controller.Dispose();
        }
    }
}
