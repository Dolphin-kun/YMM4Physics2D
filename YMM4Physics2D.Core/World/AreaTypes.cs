using System.ComponentModel.DataAnnotations;
using System.Numerics;

namespace YMM4Physics2D.Core.World
{
    public enum AreaKind
    {
        [Display(Name = "風", Description = "範囲の中の物体を、流れの向きへ押し流します")]
        Wind = 1,

        [Display(Name = "水", Description = "範囲の中の物体に浮力と水の抵抗がかかり、流れに運ばれます")]
        Water = 2,

        [Display(Name = "トリガー", Description = "範囲に物体が触れたときに動作します")]
        Trigger = 4,
    }

    public enum TriggerAction
    {
        [Display(Name = "消える", Description = "触れた物体がすり抜けるようになり、フェードして消えます")]
        Vanish = 1,

        [Display(Name = "止まる", Description = "触れた物体がその場で固定されます")]
        Freeze = 2,

        [Display(Name = "弾く", Description = "触れた物体を指定した向きと速さで弾き飛ばします")]
        Launch = 4,

        [Display(Name = "ワープ", Description = "触れた物体を指定した位置へ移動させます")]
        Warp = 8,
    }

    public readonly record struct AreaDefinition(AreaKind Kind, TriggerAction Action, string TargetName, bool KeepVelocity);

    public readonly record struct AreaParameters(
        float FlowAngle,
        float FlowSpeed,
        float Strength,
        float Turbulence,
        float Buoyancy,
        float Drag,
        float LaunchAngle,
        float LaunchSpeed,
        Vector2 WarpOffset,
        float FadeSeconds);

    public delegate AreaParameters AreaParameterProvider(int localFrame, int length, int fps);

    public readonly record struct AreaBounds(float Left, float Top, float Right, float Bottom)
    {
        public bool IsEmpty => Right <= Left || Bottom <= Top;
        public Vector2 Center => new((Left + Right) / 2f, (Top + Bottom) / 2f);
    }
}
