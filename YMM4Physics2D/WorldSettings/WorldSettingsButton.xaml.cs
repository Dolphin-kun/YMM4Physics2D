using System.Windows;
using System.Windows.Controls;
using YukkuriMovieMaker.Commons;

namespace YMM4Physics2D.WorldSettings
{
    public partial class WorldSettingsButton : UserControl, IPropertyEditorControl
    {
        private static WorldSettingsWindow? openWindow;

        public bool IsOpen
        {
            get { return (bool)GetValue(IsOpenProperty); }
            set { SetValue(IsOpenProperty, value); }
        }
        public static readonly DependencyProperty IsOpenProperty =
            DependencyProperty.Register(nameof(IsOpen), typeof(bool), typeof(WorldSettingsButton), new FrameworkPropertyMetadata(false, FrameworkPropertyMetadataOptions.BindsTwoWayByDefault));

        public Func<int?>? WorldIdProvider { get; set; }

#pragma warning disable CS0067
        public event EventHandler? BeginEdit;
        public event EventHandler? EndEdit;
#pragma warning restore CS0067

        public WorldSettingsButton()
        {
            InitializeComponent();
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            if (WorldIdProvider?.Invoke() is int worldId)
            {
                WorldSettings.Default.WorldId = worldId;
            }

            if (openWindow != null)
            {
                if (openWindow.WindowState == WindowState.Minimized) openWindow.WindowState = WindowState.Normal;
                openWindow.Activate();
                return;
            }

            openWindow = new WorldSettingsWindow
            {
                Owner = Application.Current.MainWindow
            };
            openWindow.Closed += (_, _) => openWindow = null;
            openWindow.Show();
        }
    }
}
