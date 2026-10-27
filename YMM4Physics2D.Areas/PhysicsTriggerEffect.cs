using System.ComponentModel.DataAnnotations;
using System.Numerics;
using YMM4Physics2D.Core.World;
using YMM4Physics2D.Shared;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Controls;
using YukkuriMovieMaker.Exo;
using YukkuriMovieMaker.ItemEditor.CustomVisibilityAttributes;
using YukkuriMovieMaker.Player.Video;
using YukkuriMovieMaker.Plugin.Effects;

namespace YMM4Physics2D.Areas
{
    [VideoEffect("物理トリガー", ["物理"], ["Physics", "Trigger", "トリガー", "当たり", "判定"], IsAviUtlSupported = false, IsEffectItemSupported = false)]
    internal class PhysicsTriggerEffect : VideoEffectBase, IAreaEffect, ISceneAware
    {
        private Guid? sceneId;

        Guid? IAreaEffect.SceneId { get => sceneId; set => sceneId = value; }

        Guid? ISceneAware.SceneId => sceneId;

        public override string Label => string.IsNullOrEmpty(TriggerName) ? "物理トリガー" : $"物理トリガー({TriggerName})";

        [Display(GroupName = "トリガー", Name = "ワールドID", Description = "同じワールドIDの物理演算エフェクトに反応します")]
        [TextBoxSlider("F0", "", 0, 10)]
        [Range(0, 100)]
        public double WorldId { get => worldId; set => Set(ref worldId, value); }
        double worldId;

        [Display(GroupName = "トリガー", Name = "名前", Description = "トリガー連動エフェクトで、このトリガーを指定するときの名前")]
        [TextEditor(AcceptsReturn = false)]
        public string TriggerName { get => triggerName; set => Set(ref triggerName, value ?? string.Empty, nameof(TriggerName), nameof(Label)); }
        string triggerName = string.Empty;

        [Display(GroupName = "トリガー", Name = "対象", Description = "反応する物理演算エフェクトの「名前」\n空欄ですべての物体に反応します\n「図形/」のように/で終わると、「図形/」から始まる名前すべてに反応します")]
        [NameEditor(NameKind.Body, true)]
        public string TargetName { get => targetName; set => Set(ref targetName, value ?? string.Empty); }
        string targetName = string.Empty;

        [Display(GroupName = "トリガー", Name = "範囲を表示", Description = "オフにすると、このアイテム自体は表示されなくなります")]
        [ToggleSlider]
        public bool ShowArea { get => showArea; set => Set(ref showArea, value); }
        bool showArea = true;

        [Display(GroupName = "動作", Name = "触れたとき", Description = "物体がこのアイテムの範囲に触れたときの動作")]
        [EnumComboBox]
        public TriggerAction Action { get => action; set => Set(ref action, value); }
        TriggerAction action = TriggerAction.Vanish;

        [Display(GroupName = "動作", Name = "消える時間", Description = "消え始めてから見えなくなるまでの時間")]
        [AnimationSlider("F2", "秒", 0, 3)]
        [ShowPropertyEditorWhen(nameof(Action), TriggerAction.Vanish)]
        public Animation FadeSeconds { get; } = new Animation(0.3, 0, 100);

        [Display(GroupName = "動作", Name = "弾く向き", Description = "0°で右、90°で下、180°で左、270°で上")]
        [AnimationSlider("F0", "°", 0, 360)]
        [ShowPropertyEditorWhen(nameof(Action), TriggerAction.Launch)]
        public Animation LaunchAngle { get; } = new Animation(270, -100000, 100000);

        [Display(GroupName = "動作", Name = "弾く速さ", Description = "弾き飛ばす速さ")]
        [AnimationSlider("F0", "px/s", 0, 3000)]
        [ShowPropertyEditorWhen(nameof(Action), TriggerAction.Launch)]
        public Animation LaunchSpeed { get; } = new Animation(1200, 0, 100000);

        [Display(GroupName = "動作", Name = "ワープ先 X", Description = "このアイテムの中心から見た、ワープ先の位置")]
        [AnimationSlider("F0", "px", -1000, 1000)]
        [ShowPropertyEditorWhen(nameof(Action), TriggerAction.Warp)]
        public Animation WarpX { get; } = new Animation(0, -100000, 100000);

        [Display(GroupName = "動作", Name = "ワープ先 Y", Description = "このアイテムの中心から見た、ワープ先の位置")]
        [AnimationSlider("F0", "px", -1000, 1000)]
        [ShowPropertyEditorWhen(nameof(Action), TriggerAction.Warp)]
        public Animation WarpY { get; } = new Animation(-400, -100000, 100000);

        [Display(GroupName = "動作", Name = "今の速度を保つ", Description = "弾く: 今の速度に弾く速度を足します\nワープ: ワープ後も今の速度のまま動きます")]
        [ToggleSlider]
        [ShowPropertyEditorWhen(nameof(Action), TriggerAction.Launch | TriggerAction.Warp)]
        public bool KeepVelocity { get => keepVelocity; set => Set(ref keepVelocity, value); }
        bool keepVelocity;

        public string AreaName => TriggerName;

        public AreaDefinition CreateDefinition()
        {
            return new AreaDefinition(AreaKind.Trigger, Action, TargetName, KeepVelocity);
        }

        public AreaParameters GetParameters(int frame, int length, int fps)
        {
            return new AreaParameters(
                0f,
                0f,
                0f,
                0f,
                0f,
                0f,
                (float)LaunchAngle.GetValue(frame, length, fps),
                (float)LaunchSpeed.GetValue(frame, length, fps),
                new Vector2((float)WarpX.GetValue(frame, length, fps), (float)WarpY.GetValue(frame, length, fps)),
                (float)FadeSeconds.GetValue(frame, length, fps));
        }

        public override IEnumerable<string> CreateExoVideoFilters(int keyFrameIndex, ExoOutputDescription exoOutputDescription)
        {
            return [];
        }

        public override IVideoEffectProcessor CreateVideoEffect(IGraphicsDevicesAndContext devices)
        {
            return new AreaEffectProcessor(devices, this);
        }

        protected override IEnumerable<IAnimatable> GetAnimatables() => [FadeSeconds, LaunchAngle, LaunchSpeed, WarpX, WarpY];
    }
}
