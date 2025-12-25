using System.ComponentModel.DataAnnotations;

namespace YMM4Physics2D.Core.Bodies
{
    public enum BodyType
    {
        [Display(Name ="動的",Description = "物理演算の影響を受け、力や衝突によって移動・回転する")]
        Dynamic,

        [Display(Name ="静的",Description = "物理演算の影響を受けず、位置や回転は固定\nアニメーション可能")]
        Static,
    }
}
