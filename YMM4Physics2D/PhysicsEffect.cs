using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.WorldSettings;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Controls;
using YukkuriMovieMaker.Exo;
using YukkuriMovieMaker.Player.Video;
using YukkuriMovieMaker.Plugin.Effects;

namespace YMM4Physics2D
{
    [VideoEffect("物理演算", ["アニメーション"], ["Physics"],IsAviUtlSupported =false,IsEffectItemSupported =false)]
    internal class PhysicsEffect : VideoEffectBase
    {
        public override string Label => "物理演算";

        [Display(GroupName = "ワールド", Name = "ワールド設定", Description = "ワールド設定")]
        [WorldSettingsButton]
        public bool IsOpen { get => isOpen; set => Set(ref isOpen, value); }
        private bool isOpen;

        [Display(GroupName = "ワールド", Name = "ワールドID", Description = "ワールドID")]
        [TextBoxSlider("F0", "", 0, 10)]
        [DefaultValue(0d)]
        [Range(0, 100)]
        public double WorldId { get => worldId; set => Set(ref worldId, value); }
        double worldId = 0;

        [Display(GroupName = "物理設定", Name = "種類", Description = "物理挙動の種類")]
        [EnumComboBox]
        public BodyType BodyType { get => bodyType; set => Set(ref bodyType, value); }
        BodyType bodyType = BodyType.Dynamic;

        [Display(GroupName = "物理設定", Name = "質量", Description = "質量")]
        [AnimationSlider("F1", "", 0.1, 50.0)]
        public Animation Mass { get; } = new Animation(20.0, 0.1, 1000.0);

        [Display(GroupName = "物理設定", Name = "反発係数", Description = "反発係数")]
        [AnimationSlider("F1", "", 0, 50.0)]
        public Animation Restitution { get; } = new Animation(0.5, 0, 1.0);

        [Display(GroupName = "物理設定", Name = "摩擦係数", Description = "摩擦係数")]
        [AnimationSlider("F1", "", 0, 50.0)]
        public Animation Friction { get; } = new Animation(0.5, 0, 1.0);

        [Display(GroupName = "物理設定", Name = "空気抵抗(移動)", Description = "空気抵抗(移動)")]
        [AnimationSlider("F2", "", 0, 50.0)]
        public Animation LinearDamping { get; } = new Animation(0.05, 0, 0.5);

        [Display(GroupName = "物理設定", Name = "空気抵抗(回転)", Description = "空気抵抗(回転)")]
        [AnimationSlider("F1", "", 0, 50.0)]
        public Animation AngularDamping { get; } = new Animation(3.0, 0, 5.0);

        [Display(GroupName = "生成設定", Name = "形状の簡略化", Description = "形状の簡略化")]
        [AnimationSlider("F1", "", 0, 50.0)]
        public Animation SimplifyTolerance { get; } = new Animation(1.5, 0.1, 5.0);

        [Display(GroupName = "生成設定", Name = "分離", Description = "分離")]
        [ToggleSlider]
        public bool IsSeparate { get => isSeparate; set => Set(ref isSeparate, value); }
        bool isSeparate = false;

        public override IEnumerable<string> CreateExoVideoFilters(int keyFrameIndex, ExoOutputDescription exoOutputDescription)
        {
            return [];
        }

        public override IVideoEffectProcessor CreateVideoEffect(IGraphicsDevicesAndContext devices)
        {
            return new PhysicsEffectProcessor(devices, this);
        }

        protected override IEnumerable<IAnimatable> GetAnimatables() => [Mass,Restitution,Friction,LinearDamping,AngularDamping,SimplifyTolerance];
    }
}
