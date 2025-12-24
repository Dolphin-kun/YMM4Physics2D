using System.Numerics;
using Vortice.Direct2D1;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Controllers;
using YMM4Physics2D.Core.Core;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Player.Video;

namespace YMM4Physics2D.Sample
{
    internal class SampleEffectProcessor : IVideoEffectProcessor
    {
        private readonly IGraphicsDevicesAndContext devices;
        private readonly SampleEffect item;
        private ID2D1Image? input;
        private ID2D1CommandList? _outputCommandList;

        private RigidBodyController? _controller;

        public ID2D1Image Output => _outputCommandList ?? input ?? throw new NullReferenceException(nameof(input) + " is null");

        public SampleEffectProcessor(IGraphicsDevicesAndContext devices, SampleEffect item)
        {
            this.devices = devices;
            this.item = item;
        }

        public DrawDescription Update(EffectDescription effectDescription)
        {
            var drawDesc = effectDescription.DrawDescription;
            var currentZoom = drawDesc.Zoom;

            var itemFrame = effectDescription.ItemPosition.Frame;
            var timelineFrame = effectDescription.TimelinePosition.Frame;
            var length = effectDescription.ItemDuration.Frame;
            var fps = effectDescription.FPS;

            var worldId = (int)item.WorldId;
            var type = item.BodyType;
            var mass = (float)item.Mass.GetValue(itemFrame, length, fps);
            var restitution = (float)item.Restitution.GetValue(itemFrame, length, fps);
            var friction = (float)item.Friction.GetValue(itemFrame, length, fps);
            var linearDamping = (float)item.LinearDamping.GetValue(itemFrame, length, fps);
            var angularDamping = (float)item.AngularDamping.GetValue(itemFrame, length, fps);
            var simplifyTolerance = (float)item.SimplifyTolerance.GetValue(itemFrame, length, fps);
            var isSeparate = item.IsSeparate;

            var config = WorldSettings.WorldSettings.Default.Configs.FirstOrDefault(c => c.Id == worldId);
            if (config != null) PhysicsManager.UpdateWorldSettings(worldId, config, effectDescription.ScreenSize);

            var targetPos = new Vector2(drawDesc.Draw.X, drawDesc.Draw.Y);
            var targetRot = drawDesc.Rotation.Z;

            if (_controller == null && input != null)
            {
                _controller = new RigidBodyController(worldId, config, targetPos);
            }

            if (_controller == null) return drawDesc;

            _controller.SetWorld(worldId, config);
            _controller.UpdateBodyProperties(mass, restitution, friction, linearDamping, angularDamping);
            _controller.SyncShape(devices.DeviceContext, input, currentZoom, simplifyTolerance: simplifyTolerance, separateParts: isSeparate);

            if (type == BodyType.Static)
            {
                float dt = 1.0f / fps;

                foreach (var body in _controller.Bodies)
                {
                    Vector2 positionDiff = targetPos - body.Position;

                    if (dt > 0)
                    {
                        body.LinearVelocity = positionDiff / dt;

                        float rotationDiff = targetRot - body.Rotation;
                        body.AngularVelocity = rotationDiff / dt;
                    }

                    body.Position = targetPos;
                    body.Rotation = targetRot;
                }
            }

            _controller.Step(timelineFrame, itemFrame, fps, targetPos, targetRot, type);

            _outputCommandList?.Dispose();
            _outputCommandList = devices.DeviceContext.CreateCommandList();

            using (var recorder = devices.D2D.Device.CreateDeviceContext(DeviceContextOptions.None))
            {
                recorder.Target = _outputCommandList;
                recorder.BeginDraw();
                recorder.Clear(null);

                _controller.Draw(recorder, input);

                recorder.EndDraw();
            }
            _outputCommandList.Close();

            if (_controller.Bodies.Count == 0) return drawDesc;

            return drawDesc with
            {
                Draw = new Vector3(0, 0, drawDesc.Draw.Z),
                Rotation = new Vector3(0, 0, 0)
            };
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
            _controller?.Dispose();
            _controller = null;

            _outputCommandList?.Dispose();
            _outputCommandList = null;
        }
    }
}