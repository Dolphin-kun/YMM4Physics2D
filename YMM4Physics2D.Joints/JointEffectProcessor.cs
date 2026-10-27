using System.Collections.Immutable;
using System.Numerics;
using Vortice.Direct2D1;
using YMM4Physics2D.Core;
using YMM4Physics2D.Core.World;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Player.Video;

namespace YMM4Physics2D.Joints
{
    internal class JointEffectProcessor : IVideoEffectProcessor
    {
        private readonly JointEffect item;
        private readonly Func<JointDefinition> definitionProvider;
        private ID2D1Image? input;
        private bool errorLogged;

        public JointEffectProcessor(JointEffect item)
        {
            this.item = item;
            definitionProvider = item.CreateDefinition;
            item.PropertyChanged += Item_PropertyChanged;
        }

        private void Item_PropertyChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            PhysicsManager.NotifySettingsChanged();
        }

        public ID2D1Image Output => input ?? throw new InvalidOperationException("入力画像が設定されていません");

        public DrawDescription Update(EffectDescription effectDescription)
        {
            try
            {
                return UpdateCore(effectDescription);
            }
            catch (Exception exception) when (exception is not OutOfMemoryException)
            {
                if (!errorLogged)
                {
                    errorLogged = true;
                    Log.Default.Write("物理ジョイントの処理中にエラーが発生しました", exception);
                }
                return effectDescription.DrawDescription;
            }
        }

        private DrawDescription UpdateCore(EffectDescription effectDescription)
        {
            var drawDescription = effectDescription.DrawDescription;
            item.SceneId = effectDescription.SceneId;
            int startFrame = effectDescription.TimelinePosition.Frame - effectDescription.ItemPosition.Frame;

            if (!PhysicsManager.TryFindTrack(effectDescription.SceneId, startFrame, effectDescription.Layer, effectDescription.GroupIndex, effectDescription.InputIndex, out var world, out var track, out bool isSecondary))
            {
                return drawDescription;
            }

            if (!isSecondary) world.UpdateJoint(track, item, item.CreateDefinition(), definitionProvider);

            if (!world.TryGetJointFrame(track, item, effectDescription.TimelinePosition.Frame, !isSecondary, !isSecondary && effectDescription.InputCount <= 1, out var joint))
            {
                return drawDescription;
            }

            return drawDescription with
            {
                Controllers = (drawDescription.Controllers ?? []).Add(CreateController(joint)),
            };
        }

        private VideoEffectController CreateController(JointFrame joint)
        {
            var toAnchorA = joint.ToAnchorA;
            var points = new List<ControllerPoint>
            {
                new(new Vector3(joint.PointA, 0f), args =>
                {
                    var delta = Vector2.TransformNormal(new Vector2(args.Delta.X, args.Delta.Y), toAnchorA);
                    item.AnchorX += delta.X;
                    item.AnchorY += delta.Y;
                })
                {
                    Shape = VideoControllerPointShape.Circle,
                },
            };

            if (joint.IsConnected || item.TargetMode == JointTargetMode.FixedPoint)
            {
                var toAnchorB = joint.ToAnchorB;
                points.Add(new(new Vector3(joint.PointB, 0f), args =>
                {
                    var delta = Vector2.TransformNormal(new Vector2(args.Delta.X, args.Delta.Y), toAnchorB);
                    item.TargetAnchorX += delta.X;
                    item.TargetAnchorY += delta.Y;
                })
                {
                    Shape = VideoControllerPointShape.Square,
                });
            }

            return new VideoEffectController(item, points)
            {
                Connection = VideoControllerPointConnection.Line,
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
            item.PropertyChanged -= Item_PropertyChanged;
        }
    }
}
