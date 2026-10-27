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
    public enum FlowKind
    {
        [Display(Name = "風", Description = "範囲の中の物体を、流れの向きへ押し流します")]
        Wind = 1,

        [Display(Name = "水", Description = "範囲の中の物体に浮力と水の抵抗がかかり、流れに運ばれます")]
        Water = 2,
    }

    [VideoEffect("物理エリア", ["物理"], ["Physics", "風", "水", "流れ", "浮力"], IsAviUtlSupported = false, IsEffectItemSupported = false)]
    internal class PhysicsAreaEffect : VideoEffectBase, IAreaEffect, ISceneAware
    {
        private Guid? sceneId;

        Guid? IAreaEffect.SceneId { get => sceneId; set => sceneId = value; }

        Guid? ISceneAware.SceneId => sceneId;

        public override string Label => $"物理エリア({(Kind == FlowKind.Water ? "水" : "風")})";

        [Display(GroupName = "エリア", Name = "ワールドID", Description = "同じワールドIDの物理演算エフェクトに作用します")]
        [TextBoxSlider("F0", "", 0, 10)]
        [Range(0, 100)]
        public double WorldId { get => worldId; set => Set(ref worldId, value); }
        double worldId;

        [Display(GroupName = "エリア", Name = "種類", Description = "このアイテムの範囲を、風か水の領域にします")]
        [EnumComboBox]
        public FlowKind Kind { get => kind; set => Set(ref kind, value, nameof(Kind), nameof(Label)); }
        FlowKind kind = FlowKind.Wind;

        [Display(GroupName = "エリア", Name = "対象", Description = "作用させる物理演算エフェクトの「名前」\n空欄ですべての物体に作用します\n「図形/」のように/で終わると、「図形/」から始まる名前すべてに作用します")]
        [NameEditor(NameKind.Body, true)]
        public string TargetName { get => targetName; set => Set(ref targetName, value ?? string.Empty); }
        string targetName = string.Empty;

        [Display(GroupName = "エリア", Name = "範囲を表示", Description = "オフにすると、このアイテム自体は表示されなくなります")]
        [ToggleSlider]
        public bool ShowArea { get => showArea; set => Set(ref showArea, value); }
        bool showArea = true;

        [Display(GroupName = "流れ", Name = "向き", Description = "流れの向き\n0°で右、90°で下、180°で左、270°で上")]
        [AnimationSlider("F0", "°", 0, 360)]
        public Animation FlowAngle { get; } = new Animation(0, -100000, 100000);

        [Display(GroupName = "流れ", Name = "速さ", Description = "流れの速さ")]
        [AnimationSlider("F0", "px/s", 0, 2000)]
        public Animation FlowSpeed { get; } = new Animation(600, 0, 100000);

        [Display(GroupName = "流れ", Name = "風の強さ", Description = "物体が風の速さに近づくまでの早さ\n大きいほどすぐに風に乗ります")]
        [AnimationSlider("F1", "", 0, 20)]
        [ShowPropertyEditorWhen(nameof(Kind), FlowKind.Wind)]
        public Animation Strength { get; } = new Animation(3, 0, 1000);

        [Display(GroupName = "流れ", Name = "乱れ", Description = "風の向きと強さの揺らぎ")]
        [AnimationSlider("F0", "%", 0, 100)]
        [ShowPropertyEditorWhen(nameof(Kind), FlowKind.Wind)]
        public Animation Turbulence { get; } = new Animation(30, 0, 100);

        [Display(GroupName = "流れ", Name = "浮力", Description = "沈んでいる部分にかかる浮き上がる力\n100%で重力とつり合い、それより大きいと浮きます")]
        [AnimationSlider("F0", "%", 0, 300)]
        [ShowPropertyEditorWhen(nameof(Kind), FlowKind.Water)]
        public Animation Buoyancy { get; } = new Animation(150, 0, 10000);

        [Display(GroupName = "流れ", Name = "水の抵抗", Description = "水の中での動きにくさ")]
        [AnimationSlider("F1", "", 0, 10)]
        [ShowPropertyEditorWhen(nameof(Kind), FlowKind.Water)]
        public Animation Drag { get; } = new Animation(2, 0, 1000);

        public string AreaName => string.Empty;

        public AreaDefinition CreateDefinition()
        {
            return new AreaDefinition(Kind == FlowKind.Water ? AreaKind.Water : AreaKind.Wind, TriggerAction.Vanish, TargetName, false);
        }

        public AreaParameters GetParameters(int frame, int length, int fps)
        {
            return new AreaParameters(
                (float)FlowAngle.GetValue(frame, length, fps),
                (float)FlowSpeed.GetValue(frame, length, fps),
                (float)Strength.GetValue(frame, length, fps),
                (float)Turbulence.GetValue(frame, length, fps) / 100f,
                (float)Buoyancy.GetValue(frame, length, fps) / 100f,
                (float)Drag.GetValue(frame, length, fps),
                0f,
                0f,
                Vector2.Zero,
                0f);
        }

        public override IEnumerable<string> CreateExoVideoFilters(int keyFrameIndex, ExoOutputDescription exoOutputDescription)
        {
            return [];
        }

        public override IVideoEffectProcessor CreateVideoEffect(IGraphicsDevicesAndContext devices)
        {
            return new AreaEffectProcessor(devices, this);
        }

        protected override IEnumerable<IAnimatable> GetAnimatables() => [FlowAngle, FlowSpeed, Strength, Turbulence, Buoyancy, Drag];
    }
}
