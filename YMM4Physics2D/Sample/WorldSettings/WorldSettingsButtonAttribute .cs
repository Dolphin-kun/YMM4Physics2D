using System.Windows;
using System.Windows.Data;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Views.Converters;

namespace YMM4Physics2D.Sample.WorldSettings
{
    internal class WorldSettingsButtonAttribute: PropertyEditorAttribute2
    {
        public override FrameworkElement Create()
        {
            return new WorldSettingsButton();
        }

        public override void SetBindings(FrameworkElement control, ItemProperty[] itemProperties)
        {
            var editor = (WorldSettingsButton)control;
            editor.SetBinding(WorldSettingsButton.IsOpenProperty, ItemPropertiesBinding.Create2(itemProperties));
        }

        public override void ClearBindings(FrameworkElement control)
        {
            BindingOperations.ClearBinding(control, WorldSettingsButton.IsOpenProperty);
        }
    }
}
