using System.Windows;
using System.Windows.Data;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Controls;
using YukkuriMovieMaker.Views.Converters;

namespace YMM4Physics2D.Shared
{
    internal enum NameKind
    {
        Body,
        Trigger,
    }

    internal interface ISceneAware
    {
        Guid? SceneId { get; }
    }

    internal class NameEditorAttribute(NameKind kind = NameKind.Body, bool allowsGroup = false) : PropertyEditorAttribute2
    {
        public override PropertyEditorSize PropertyEditorSize { get; set; } = PropertyEditorSize.FullWidth;

        public override FrameworkElement Create()
        {
            return new NameEditor { Kind = kind, AllowsGroup = allowsGroup };
        }

        public override void SetBindings(FrameworkElement control, ItemProperty[] itemProperties)
        {
            var editor = (NameEditor)control;
            var binding = ItemPropertiesBinding.Create2(itemProperties);
            binding.UpdateSourceTrigger = UpdateSourceTrigger.PropertyChanged;
            editor.TextEditor.SetBinding(TextEditor.TextProperty, binding);
            editor.SceneIdProvider = () => itemProperties
                .Select(p => p.PropertyOwner)
                .OfType<ISceneAware>()
                .Select(owner => owner.SceneId)
                .FirstOrDefault(id => id != null);
        }

        public override void ClearBindings(FrameworkElement control)
        {
            var editor = (NameEditor)control;
            BindingOperations.ClearBinding(editor.TextEditor, TextEditor.TextProperty);
            editor.SceneIdProvider = null;
        }
    }
}
