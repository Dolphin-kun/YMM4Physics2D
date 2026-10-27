using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Interop;
using System.Windows.Media;

namespace YMM4Physics2D.WorldSettings
{
    public partial class WorldSettingsWindow : Window
    {
        private const int DwmwaBorderColor = 34;
        private const int DwmwaCaptionColor = 35;
        private const int DwmwaTextColor = 36;

        [DllImport("dwmapi.dll")]
        private static extern int DwmSetWindowAttribute(nint hwnd, int attribute, ref int value, int size);

        public WorldSettingsWindow()
        {
            InitializeComponent();
            SourceInitialized += (_, _) => ApplyTitleBarColor();
        }

        private void ApplyTitleBarColor()
        {
            if (!OperatingSystem.IsWindowsVersionAtLeast(10, 0, 22000)) return;
            if (TryFindResource(SystemColors.ControlBrushKey) is not SolidColorBrush background) return;
            if (background.Color == SystemColors.ControlColor) return;

            var foreground = TryFindResource(SystemColors.ControlTextBrushKey) as SolidColorBrush;

            nint handle = new WindowInteropHelper(this).Handle;
            int caption = ToColorRef(background.Color);
            int text = ToColorRef(foreground?.Color ?? Colors.White);
            DwmSetWindowAttribute(handle, DwmwaCaptionColor, ref caption, sizeof(int));
            DwmSetWindowAttribute(handle, DwmwaBorderColor, ref caption, sizeof(int));
            DwmSetWindowAttribute(handle, DwmwaTextColor, ref text, sizeof(int));
        }

        private static int ToColorRef(Color color) => color.R | (color.G << 8) | (color.B << 16);
    }
}
