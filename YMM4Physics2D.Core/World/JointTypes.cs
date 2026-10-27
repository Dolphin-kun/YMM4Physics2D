using System.ComponentModel.DataAnnotations;
using System.Numerics;

namespace YMM4Physics2D.Core.World
{
    public enum JointType
    {
        [Display(Name = "ひも", Description = "長さ以上に離れないようにつなぎます。近づくのは自由です")]
        Rope = 1,

        [Display(Name = "棒", Description = "接続点どうしの距離を一定に保ちます")]
        Distance = 2,

        [Display(Name = "ばね", Description = "伸び縮みしながら長さに戻ろうとします")]
        Spring = 4,

        [Display(Name = "ヒンジ", Description = "接続点どうしを重ねてつなぎ、その点を中心に回転できます")]
        Hinge = 8,

        [Display(Name = "固定", Description = "接続点どうしを重ねてつなぎ、向きも固定します")]
        Weld = 16,
    }

    public enum JointTargetMode
    {
        [Display(Name = "空中の点", Description = "空中の決まった位置につなぎます\n位置は、このアイテムを置いた位置から見た相対位置です")]
        FixedPoint = 1,

        [Display(Name = "レイヤーで指定", Description = "指定したレイヤーにあるアイテムとつなぎます\n相手のアイテムにも物理演算エフェクトが必要です")]
        Layer = 2,

        [Display(Name = "名前で指定", Description = "物理演算エフェクトの「名前」が一致するアイテムとつなぎます")]
        Name = 4,

        [Display(Name = "前の文字・アイテム", Description = "文字ごとに分割したテキストやグループ制御で、1つ前の文字・アイテムとつなぎます\n鎖のようにつながります")]
        Previous = 8,
    }

    public readonly record struct JointDefinition(
        JointType Type,
        JointTargetMode TargetMode,
        string TargetName,
        int TargetLayer,
        Vector2 AnchorA,
        Vector2 AnchorB,
        float Length,
        float Frequency,
        float DampingRatio,
        bool CollideConnected,
        float LineWidth,
        uint LineColor);

    public readonly record struct JointLine(Vector2 Start, Vector2 End, float Width, uint Color);

    public readonly record struct JointFrame(
        object Key,
        Vector2 PointA,
        Vector2 PointB,
        bool IsConnected,
        Matrix3x2 ToAnchorA,
        Matrix3x2 ToAnchorB,
        float LineWidth,
        uint LineColor);
}
