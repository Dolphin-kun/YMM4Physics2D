using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using Newtonsoft.Json;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.World;
using YMM4Physics2D.WorldSettings;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Controls;
using YukkuriMovieMaker.Exo;
using YukkuriMovieMaker.ItemEditor.CustomVisibilityAttributes;
using YukkuriMovieMaker.Player.Video;
using YukkuriMovieMaker.Plugin.Effects;

namespace YMM4Physics2D
{
    [VideoEffect("物理演算", ["物理"], ["Physics"],IsAviUtlSupported =false,IsEffectItemSupported =false)]
    internal class PhysicsEffect : VideoEffectBase
    {
        public override string Label => "物理演算";

        [Display(GroupName = "ワールド", Name = "ワールド設定", Description = "重力や壁など、ワールド全体の設定を開きます")]
        [WorldSettingsButton]
        [JsonIgnore]
        public bool IsOpen { get => isOpen; set => Set(ref isOpen, value); }
        private bool isOpen;

        [Display(GroupName = "ワールド", Name = "ワールドID", Description = "同じワールドIDのアイテム同士が、同じ空間で物理演算されます")]
        [TextBoxSlider("F0", "", 0, 10)]
        [DefaultValue(0d)]
        [Range(0, 100)]
        public double WorldId { get => worldId; set => Set(ref worldId, value); }
        double worldId = 0;

        [Display(GroupName = "ワールド", Name = "名前", Description = "物理ジョイントで接続先を指定するときの名前")]
        [TextEditor(AcceptsReturn = false)]
        public string BodyName { get => bodyName; set => Set(ref bodyName, value ?? string.Empty); }
        string bodyName = string.Empty;

        [Display(GroupName = "物理", Name = "種類", Description = "物理挙動の種類")]
        [EnumComboBox]
        public BodyType BodyType { get => bodyType; set => Set(ref bodyType, value); }
        BodyType bodyType = BodyType.Dynamic;

        [Display(GroupName = "物理", Name = "質量", Description = "重さ。重い物体は軽い物体に押されにくくなります")]
        [AnimationSlider("F1", "", 0.1, 100.0)]
        public Animation Mass { get; } = new Animation(20.0, 0.1, 1000.0);

        [Display(GroupName = "物理", Name = "反発係数", Description = "ぶつかったときの跳ね返りやすさ\n0で跳ね返らず、1で勢いを保って跳ね返ります")]
        [AnimationSlider("F2", "", 0, 1.0)]
        public Animation Restitution { get; } = new Animation(0.5, 0, 1.0);

        [Display(GroupName = "物理", Name = "摩擦係数", Description = "接触したときの滑りにくさ")]
        [AnimationSlider("F2", "", 0, 1.0)]
        public Animation Friction { get; } = new Animation(0.5, 0, 1.0);

        [Display(GroupName = "物理", Name = "空気抵抗(移動)", Description = "移動の速さが自然に落ちる割合")]
        [AnimationSlider("F2", "", 0, 0.5)]
        public Animation LinearDamping { get; } = new Animation(0.05, 0, 0.5);

        [Display(GroupName = "物理", Name = "空気抵抗(回転)", Description = "回転が自然に止まる割合")]
        [AnimationSlider("F1", "", 0, 5.0)]
        public Animation AngularDamping { get; } = new Animation(3.0, 0, 5.0);

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Fluid)]
        [Display(GroupName = "液体", Name = "粘り気", Description = "どろっとした流れにくさ\n0%でさらさらの水、100%でとろみのある液体になります")]
        [AnimationSlider("F0", "%", 0, 100)]
        public Animation FluidViscosity { get; } = new Animation(30, 0, 100);

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Fluid)]
        [Display(GroupName = "液体", Name = "まとまりやすさ", Description = "粒どうしが引き合って、丸い水滴になろうとする強さ\n大きいほど、ちぎれにくく粒が離れにくくなります")]
        [AnimationSlider("F0", "%", 0, 100)]
        public Animation FluidCohesion { get; } = new Animation(50, 0, 100);

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Fluid)]
        [Display(GroupName = "液体", Name = "なめらかさ", Description = "粒のつながりをなめらかに見せる度合い\n0%で粒がそのまま見え、大きいほど液体らしい輪郭になります")]
        [AnimationSlider("F0", "%", 0, 100)]
        public Animation FluidSmoothness { get; } = new Animation(60, 0, 100);

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Fluid)]
        [Display(GroupName = "液体", Name = "テカリ", Description = "液体の表面に映る光の強さ\n0%で光らず、大きいほどつやつやに見えます")]
        [AnimationSlider("F0", "%", 0, 200)]
        public Animation FluidGloss { get; } = new Animation(100, 0, 500);

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Fluid)]
        [Display(GroupName = "液体", Name = "乾きやすさ", Description = "地面などに薄く広がった部分が消えていく速さ\n0%で消えません")]
        [AnimationSlider("F0", "%", 0, 100)]
        public Animation FluidDrying { get; } = new Animation(50, 0, 100);

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Fluid)]
        [Display(GroupName = "液体", Name = "細かさ", Description = "液体を作る粒のおおよその数\n多いほどなめらかに流れますが、重くなります")]
        [TextBoxSlider("F0", "", 30, 600)]
        [DefaultValue(120d)]
        [Range(10, 3000)]
        public double FluidResolution { get => fluidResolution; set => Set(ref fluidResolution, value); }
        double fluidResolution = 120;

        [Display(GroupName = "吸引", Name = "吸引", Description = "オンにすると、他のアイテムを引き寄せたり遠ざけたりします")]
        [ToggleSlider]
        public bool IsAttractionEnabled { get => isAttractionEnabled; set => Set(ref isAttractionEnabled, value); }
        bool isAttractionEnabled;

        [ShowPropertyEditorWhen(nameof(IsAttractionEnabled), true)]
        [Display(GroupName = "吸引", Name = "吸引力", Description = "他のアイテムをこのアイテムへ引き寄せる強さ\nマイナスにすると反発します")]
        [AnimationSlider("F1", "m/s²", -50.0, 50.0)]
        public Animation Attraction { get; } = new Animation(10.0, -10000.0, 10000.0);

        [ShowPropertyEditorWhen(nameof(IsAttractionEnabled), true)]
        [Display(GroupName = "吸引", Name = "吸引範囲", Description = "吸引が届く距離\n中心ほど強く、範囲の端で0になります\n0にすると距離に関係なく一定の強さになります")]
        [AnimationSlider("F0", "px", 0.0, 2000.0)]
        public Animation AttractionRange { get; } = new Animation(500.0, 0.0, 100000.0);

        [Display(GroupName = "当たり判定", Name = "形状の簡略化", Description = "輪郭を簡略化する度合い(px)\n大きいほど軽くなりますが、形が粗くなります")]
        [AnimationSlider("F1", "px", 0.1, 5.0)]
        public Animation SimplifyTolerance { get; } = new Animation(1.5, 0.1, 5.0);

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Dynamic | BodyType.Static)]
        [Display(GroupName = "当たり判定", Name = "分離", Description = "つながっていない部分を、別々の物体として動かします")]
        [ToggleSlider]
        public bool IsSeparate { get => isSeparate; set => Set(ref isSeparate, value); }
        bool isSeparate = false;

        [ShowPropertyEditorWhen(nameof(BodyType), BodyType.Dynamic | BodyType.Static)]
        [Display(GroupName = "当たり判定", Name = "マージン", Description = "当たり判定を見た目より広げる幅\nマイナスにすると狭くなり、物体どうしが少し重なって見えます")]
        [TextBoxSlider("F1", "px", -10, 10)]
        [DefaultValue(0d)]
        [Range(-100, 100)]
        public double CollisionMargin { get => collisionMargin; set => Set(ref collisionMargin, value); }
        double collisionMargin = 0;

        [Display(GroupName = "当たり判定", Name = "形状の更新", Description = "当たり判定を作り直すタイミング")]
        [EnumComboBox]
        public ShapeUpdateMode ShapeUpdateMode { get => shapeUpdateMode; set => Set(ref shapeUpdateMode, value); }
        ShapeUpdateMode shapeUpdateMode = ShapeUpdateMode.Once;

        public override IEnumerable<string> CreateExoVideoFilters(int keyFrameIndex, ExoOutputDescription exoOutputDescription)
        {
            return [];
        }

        public override IVideoEffectProcessor CreateVideoEffect(IGraphicsDevicesAndContext devices)
        {
            return new PhysicsEffectProcessor(devices, this);
        }

        protected override IEnumerable<IAnimatable> GetAnimatables() => [Mass, Restitution, Friction, LinearDamping, AngularDamping, SimplifyTolerance, Attraction, AttractionRange, FluidViscosity, FluidCohesion, FluidSmoothness, FluidGloss, FluidDrying];
    }
}
