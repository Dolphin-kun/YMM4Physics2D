using System.ComponentModel.DataAnnotations;
using Vortice.Direct2D1;
using YMM4Physics2D.Core;
using YMM4Physics2D.Shared;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Controls;
using YukkuriMovieMaker.Exo;
using YukkuriMovieMaker.Player.Video;
using YukkuriMovieMaker.Plugin.Effects;

namespace YMM4Physics2D.Areas
{
    public enum TriggerReaction
    {
        [Display(Name = "現れる", Description = "トリガーが反応するまで隠れていて、反応したら現れます")]
        Appear,

        [Display(Name = "消える", Description = "トリガーが反応したら消えます")]
        Disappear,
    }

    [VideoEffect("トリガー連動", ["物理"], ["Physics", "Trigger", "トリガー", "連動"], IsAviUtlSupported = false, IsEffectItemSupported = false)]
    internal class TriggerReactionEffect : VideoEffectBase, ISceneAware
    {
        internal Guid? SceneId { get; set; }

        Guid? ISceneAware.SceneId => SceneId;

        public override string Label => string.IsNullOrEmpty(TriggerName) ? "トリガー連動" : $"トリガー連動({TriggerName})";

        [Display(GroupName = "トリガー連動", Name = "ワールドID", Description = "トリガーを置いたワールドのID")]
        [TextBoxSlider("F0", "", 0, 10)]
        [Range(0, 100)]
        public double WorldId { get => worldId; set => Set(ref worldId, value); }
        double worldId;

        [Display(GroupName = "トリガー連動", Name = "トリガーの名前", Description = "物理トリガーエフェクトの「名前」")]
        [NameEditor(NameKind.Trigger)]
        public string TriggerName { get => triggerName; set => Set(ref triggerName, value ?? string.Empty, nameof(TriggerName), nameof(Label)); }
        string triggerName = string.Empty;

        [Display(GroupName = "トリガー連動", Name = "反応", Description = "トリガーが反応したときの、このアイテムの変化")]
        [EnumComboBox]
        public TriggerReaction Reaction { get => reaction; set => Set(ref reaction, value); }
        TriggerReaction reaction = TriggerReaction.Appear;

        [Display(GroupName = "トリガー連動", Name = "遅れ", Description = "トリガーが反応してから変化し始めるまでの時間")]
        [TextBoxSlider("F2", "秒", 0, 3)]
        [Range(0, 1000)]
        public double Delay { get => delay; set => Set(ref delay, value); }
        double delay;

        [Display(GroupName = "トリガー連動", Name = "フェード", Description = "変化にかける時間")]
        [TextBoxSlider("F2", "秒", 0, 3)]
        [Range(0, 1000)]
        public double FadeSeconds { get => fadeSeconds; set => Set(ref fadeSeconds, value); }
        double fadeSeconds = 0.3;

        public override IEnumerable<string> CreateExoVideoFilters(int keyFrameIndex, ExoOutputDescription exoOutputDescription)
        {
            return [];
        }

        public override IVideoEffectProcessor CreateVideoEffect(IGraphicsDevicesAndContext devices)
        {
            return new TriggerReactionProcessor(this);
        }

        protected override IEnumerable<IAnimatable> GetAnimatables() => [];
    }

    internal sealed class TriggerReactionProcessor(TriggerReactionEffect item) : IVideoEffectProcessor
    {
        private ID2D1Image? input;
        private bool errorLogged;

        public ID2D1Image Output => input ?? throw new InvalidOperationException("入力画像が設定されていません");

        public DrawDescription Update(EffectDescription effectDescription)
        {
            var drawDescription = effectDescription.DrawDescription;
            double progress = 0;
            try
            {
                int frame = effectDescription.TimelinePosition.Frame;
                item.SceneId = effectDescription.SceneId;
                if (PhysicsManager.TryGetTriggerFrame(effectDescription.SceneId, (int)item.WorldId, item.TriggerName, frame, out int fireFrame))
                {
                    double elapsed = (double)(frame - fireFrame) / effectDescription.FPS - item.Delay;
                    progress = elapsed < 0 ? 0 : item.FadeSeconds <= 0 ? 1 : Math.Clamp(elapsed / item.FadeSeconds, 0, 1);
                }
            }
            catch (Exception exception) when (exception is not OutOfMemoryException)
            {
                if (!errorLogged)
                {
                    errorLogged = true;
                    Log.Default.Write("トリガー連動の処理中にエラーが発生しました", exception);
                }
            }

            double visibility = item.Reaction == TriggerReaction.Appear ? progress : 1 - progress;
            return drawDescription with { Opacity = drawDescription.Opacity * visibility };
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
        }
    }
}
