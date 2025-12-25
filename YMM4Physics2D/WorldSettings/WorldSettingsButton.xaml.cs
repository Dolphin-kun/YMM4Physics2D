using System.Windows;
using System.Windows.Controls;
using YukkuriMovieMaker.Commons;

namespace YMM4Physics2D.WorldSettings
{
    public partial class WorldSettingsButton : UserControl, IPropertyEditorControl
    {
        public bool IsOpen
        {
            get { return (bool)GetValue(IsOpenProperty); }
            set { SetValue(IsOpenProperty, value); }
        }
        public static readonly DependencyProperty IsOpenProperty =
            DependencyProperty.Register(nameof(IsOpen), typeof(bool), typeof(WorldSettingsButton), new FrameworkPropertyMetadata(false, FrameworkPropertyMetadataOptions.BindsTwoWayByDefault));

        public event EventHandler? BeginEdit;
        public event EventHandler? EndEdit;

        public WorldSettingsButton()
        {
            InitializeComponent();
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            var window = new WorldSettingsWindow
            {
                Owner = Application.Current.MainWindow
            };
            window.Show();
        }
    }
}
