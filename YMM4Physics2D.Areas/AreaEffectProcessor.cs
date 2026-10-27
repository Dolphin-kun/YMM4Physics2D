using System.ComponentModel;
using System.Numerics;
using Vortice.Direct2D1;
using YMM4Physics2D.Core;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Controllers;
using YMM4Physics2D.Core.World;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Player.Video;

namespace YMM4Physics2D.Areas
{
    internal interface IAreaEffect : INotifyPropertyChanged
    {
        Guid? SceneId { get; set; }
        double WorldId { get; }
        string AreaName { get; }
        bool ShowArea { get; }
        AreaDefinition CreateDefinition();
        AreaParameters GetParameters(int frame, int length, int fps);
    }

    internal sealed class AreaEffectProcessor : IVideoEffectProcessor
    {
        private readonly IGraphicsDevicesAndContext devices;
        private readonly IAreaEffect item;
        private readonly AreaController controller;
        private readonly AreaParameterProvider parameterProvider;
        private readonly Func<TrackSettings> settingsProvider;
        private ID2D1Image? input;
        private bool errorLogged;

        public ID2D1Image Output => input ?? throw new InvalidOperationException("入力画像が設定されていません");

        public AreaEffectProcessor(IGraphicsDevicesAndContext devices, IAreaEffect item)
        {
            this.devices = devices;
            this.item = item;
            controller = new AreaController(item);
            parameterProvider = item.GetParameters;
            settingsProvider = () => new TrackSettings(BodyType.Static, false, item.AreaName, item.CreateDefinition());
            item.PropertyChanged += Item_PropertyChanged;
        }

        private void Item_PropertyChanged(object? sender, PropertyChangedEventArgs e)
        {
            PhysicsManager.NotifySettingsChanged();
        }

        public DrawDescription Update(EffectDescription effectDescription)
        {
            var drawDescription = effectDescription.DrawDescription;
            try
            {
                if (input != null) Register(effectDescription, drawDescription, input);
            }
            catch (Exception exception) when (exception is not OutOfMemoryException)
            {
                if (!errorLogged)
                {
                    errorLogged = true;
                    Log.Default.Write("物理エリアの処理中にエラーが発生しました", exception);
                }
            }

            return item.ShowArea ? drawDescription : drawDescription with { Opacity = 0 };
        }

        private void Register(EffectDescription effectDescription, DrawDescription drawDescription, ID2D1Image image)
        {
            int itemFrame = effectDescription.ItemPosition.Frame;
            int length = effectDescription.ItemDuration.Frame;
            int fps = effectDescription.FPS;
            item.SceneId = effectDescription.SceneId;
            controller.Update(devices.DeviceContext, image, new AreaInput
            {
                SceneId = effectDescription.SceneId,
                WorldId = (int)item.WorldId,
                TimelineFrame = effectDescription.TimelinePosition.Frame,
                LocalFrame = itemFrame,
                Length = length,
                Layer = effectDescription.Layer,
                GroupIndex = effectDescription.GroupIndex,
                InputIndex = effectDescription.InputIndex,
                Observation = new TrackObservation(
                    new Vector2(drawDescription.Draw.X, drawDescription.Draw.Y),
                    drawDescription.Rotation.Z * MathF.PI / 180f,
                    drawDescription.Zoom,
                    drawDescription.Invert,
                    drawDescription.CenterPoint),
                Name = item.AreaName,
                Definition = item.CreateDefinition(),
                Parameters = item.GetParameters(itemFrame, length, fps),
                ParameterProvider = parameterProvider,
                SettingsProvider = settingsProvider,
            });
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
