using System.Windows;
using System.Windows.Controls;

namespace YMM4Physics2D.WorldSettings
{
    public partial class WorldSettingsView : UserControl
    {
        public WorldSettingsView()
        {
            InitializeComponent();
        }

        private void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            var settings = WorldSettings.Default;
            var result = MessageBox.Show(
                Window.GetWindow(this),
                $"ワールドID {settings.WorldId} の設定を既定値に戻しますか?",
                "ワールド設定",
                MessageBoxButton.OKCancel);

            if (result == MessageBoxResult.OK)
            {
                settings.ResetCurrentWorld();
            }
        }

        private void CloseButton_Click(object sender, RoutedEventArgs e)
        {
            Window.GetWindow(this)?.Close();
        }
    }
}
