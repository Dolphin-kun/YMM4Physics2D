using System.Collections.Concurrent;
using System.ComponentModel.DataAnnotations;
using System.Numerics;
using System.Reflection;
using System.Windows.Media;
using YMM4Physics2D.Core.World;
using YMM4Physics2D.Shared;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Controls;
using YukkuriMovieMaker.Exo;
using YukkuriMovieMaker.ItemEditor.CustomVisibilityAttributes;
using YukkuriMovieMaker.Player.Video;
using YukkuriMovieMaker.Plugin.Effects;

namespace YMM4Physics2D.Joints
{
    [VideoEffect("物理ジョイント", ["物理"], ["Physics", "Joint", "ジョイント", "ひも", "ばね", "つなぐ"], IsAviUtlSupported = false, IsEffectItemSupported = false)]
    internal class JointEffect : VideoEffectBase, ISceneAware
    {
        private static readonly ConcurrentDictionary<JointType, string> displayNames = new();

        public override string Label => $"物理ジョイント({GetDisplayName(JointType)} → {TargetDescription})";

        private string TargetDescription => TargetMode switch
        {
            JointTargetMode.Layer => $"レイヤー{(int)TargetLayer}",
            JointTargetMode.Name => string.IsNullOrEmpty(TargetName) ? "名前未設定" : TargetName,
            JointTargetMode.Previous => "前の文字",
            _ => "空中の点",
        };

        [Display(GroupName = "接続先", Name = "接続先", Description = "つなぐ相手\n物理演算エフェクトと同じアイテムに付けてください")]
        [EnumComboBox]
        public JointTargetMode TargetMode { get => targetMode; set => Set(ref targetMode, value, nameof(TargetMode), nameof(Label)); }
        JointTargetMode targetMode = JointTargetMode.FixedPoint;

        [ShowPropertyEditorWhen(nameof(TargetMode), JointTargetMode.Layer)]
        [Display(GroupName = "接続先", Name = "レイヤー", Description = "つなぐアイテムのレイヤー番号\n相手のアイテムにも物理演算エフェクトが必要です")]
        [TextBoxSlider("F0", "", 1, 20)]
        [Range(1, 1000)]
        public double TargetLayer { get => targetLayer; set => Set(ref targetLayer, value, nameof(TargetLayer), nameof(Label)); }
        double targetLayer = 1;

        [ShowPropertyEditorWhen(nameof(TargetMode), JointTargetMode.Name)]
        [Display(GroupName = "接続先", Name = "名前", Description = "つなぐ相手の物理演算エフェクトの「名前」")]
        [NameEditor]
        public string TargetName { get => targetName; set => Set(ref targetName, value ?? string.Empty, nameof(TargetName), nameof(Label)); }
        string targetName = string.Empty;

        [Display(GroupName = "つなぎ方", Name = "種類", Description = "つなぎ方")]
        [EnumComboBox]
        public JointType JointType { get => jointType; set => Set(ref jointType, value, nameof(JointType), nameof(Label)); }
        JointType jointType = JointType.Rope;

        [ShowPropertyEditorWhen(nameof(JointType), JointType.Rope | JointType.Distance | JointType.Spring)]
        [Display(GroupName = "つなぎ方", Name = "長さ", Description = "ひも・棒・ばねの長さ\n0にすると、置いたときの接続点どうしの距離になります")]
        [TextBoxSlider("F0", "px", 0, 1000)]
        [Range(0, 100000)]
        public double Length { get => length; set => Set(ref length, value); }
        double length;

        [ShowPropertyEditorWhen(nameof(JointType), JointType.Spring)]
        [Display(GroupName = "つなぎ方", Name = "ばねの固さ", Description = "1秒間に振動する回数\n大きいほど固いばねになります")]
        [TextBoxSlider("F1", "回/秒", 0.1, 10)]
        [Range(0.01, 100)]
        public double Frequency { get => frequency; set => Set(ref frequency, value); }
        double frequency = 2.0;

        [ShowPropertyEditorWhen(nameof(JointType), JointType.Spring)]
        [Display(GroupName = "つなぎ方", Name = "ばねの減衰", Description = "揺れの収まりやすさ\n1で揺れずに止まります")]
        [TextBoxSlider("F2", "", 0, 2)]
        [Range(0, 100)]
        public double DampingRatio { get => dampingRatio; set => Set(ref dampingRatio, value); }
        double dampingRatio = 0.3;

        [ShowPropertyEditorWhen(nameof(TargetMode), JointTargetMode.Layer | JointTargetMode.Name | JointTargetMode.Previous)]
        [Display(GroupName = "つなぎ方", Name = "相手との衝突", Description = "オフにすると、つないだ相手とはぶつからなくなります")]
        [ToggleSlider]
        public bool CollideConnected { get => collideConnected; set => Set(ref collideConnected, value); }
        bool collideConnected = false;

        [Display(GroupName = "接続位置", Name = "自分側 X", Description = "このアイテムの中心から見た接続位置\nプレビューの丸い操作点をドラッグしても変えられます")]
        [TextBoxSlider("F0", "px", -500, 500)]
        public double AnchorX { get => anchorX; set => Set(ref anchorX, value); }
        double anchorX;

        [Display(GroupName = "接続位置", Name = "自分側 Y", Description = "このアイテムの中心から見た接続位置\nプレビューの丸い操作点をドラッグしても変えられます")]
        [TextBoxSlider("F0", "px", -500, 500)]
        public double AnchorY { get => anchorY; set => Set(ref anchorY, value); }
        double anchorY;

        [Display(GroupName = "接続位置", Name = "相手側 X", Description = "空中の点: このアイテムを置いた位置から見た位置\nそれ以外: 相手のアイテムの中心から見た位置\nプレビューの四角い操作点をドラッグしても変えられます")]
        [TextBoxSlider("F0", "px", -500, 500)]
        public double TargetAnchorX { get => targetAnchorX; set => Set(ref targetAnchorX, value); }
        double targetAnchorX;

        [Display(GroupName = "接続位置", Name = "相手側 Y", Description = "空中の点: このアイテムを置いた位置から見た位置\nそれ以外: 相手のアイテムの中心から見た位置\nプレビューの四角い操作点をドラッグしても変えられます")]
        [TextBoxSlider("F0", "px", -500, 500)]
        public double TargetAnchorY { get => targetAnchorY; set => Set(ref targetAnchorY, value); }
        double targetAnchorY = -200;

        [Display(GroupName = "表示", Name = "線の太さ", Description = "接続点どうしを結ぶ線の太さ\n0で線を表示しません")]
        [TextBoxSlider("F1", "px", 0, 20)]
        [Range(0, 1000)]
        public double LineWidth { get => lineWidth; set => Set(ref lineWidth, value); }
        double lineWidth = 3;

        [Display(GroupName = "表示", Name = "線の色", Description = "接続点どうしを結ぶ線の色")]
        [ColorPicker]
        public Color LineColor { get => lineColor; set => Set(ref lineColor, value); }
        Color lineColor = Colors.White;

        internal Guid? SceneId { get; set; }

        Guid? ISceneAware.SceneId => SceneId;

        internal JointDefinition CreateDefinition()
        {
            return new JointDefinition(
                JointType,
                TargetMode,
                TargetName,
                (int)TargetLayer,
                new Vector2((float)AnchorX, (float)AnchorY),
                new Vector2((float)TargetAnchorX, (float)TargetAnchorY),
                (float)Length,
                (float)Frequency,
                (float)DampingRatio,
                CollideConnected,
                (float)LineWidth,
                (uint)(LineColor.A << 24 | LineColor.R << 16 | LineColor.G << 8 | LineColor.B));
        }

        private static string GetDisplayName(JointType value)
        {
            return displayNames.GetOrAdd(value, static v =>
                typeof(JointType).GetField(v.ToString())?.GetCustomAttribute<DisplayAttribute>()?.Name ?? v.ToString());
        }

        public override IEnumerable<string> CreateExoVideoFilters(int keyFrameIndex, ExoOutputDescription exoOutputDescription)
        {
            return [];
        }

        public override IVideoEffectProcessor CreateVideoEffect(IGraphicsDevicesAndContext devices)
        {
            return new JointEffectProcessor(this);
        }

        protected override IEnumerable<IAnimatable> GetAnimatables() => [];
    }
}
