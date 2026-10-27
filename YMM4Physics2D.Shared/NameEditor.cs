using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using YMM4Physics2D.Core;
using YukkuriMovieMaker.Commons;
using YukkuriMovieMaker.Controls;

namespace YMM4Physics2D.Shared
{
    internal class NameEditor : UserControl, IPropertyEditorControl
    {
        private const char PathSeparator = '/';

        private readonly Button menuButton;
        private ContextMenu? openMenu;

        public TextEditor TextEditor { get; }

        public Func<Guid?>? SceneIdProvider { get; set; }

        public NameKind Kind { get; set; }

        public bool AllowsGroup { get; set; }

        public event EventHandler? BeginEdit;
        public event EventHandler? EndEdit;

        public NameEditor()
        {
            TextEditor = new TextEditor
            {
                AcceptsReturn = false,
                MaxHeight = 100,
            };
            TextEditor.BeginEdit += (_, e) => BeginEdit?.Invoke(this, e);
            TextEditor.EndEdit += (_, e) => EndEdit?.Invoke(this, e);

            menuButton = new Button
            {
                Width = 26,
                Height = 26,
                VerticalAlignment = VerticalAlignment.Top,
                ToolTip = "使われている名前から選ぶ",
                Content = new Path
                {
                    Data = Geometry.Parse("M0,0 L8,0 L4,5 Z"),
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                },
            };
            ((Path)menuButton.Content).SetResourceReference(Shape.FillProperty, SystemColors.ControlTextBrushKey);
            menuButton.Click += MenuButton_Click;
            menuButton.PreviewMouseLeftButtonDown += MenuButton_PreviewMouseLeftButtonDown;
            menuButton.PreviewMouseLeftButtonUp += (_, e) => e.Handled = true;

            var grid = new Grid();
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
            grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
            Grid.SetColumn(menuButton, 1);
            grid.Children.Add(TextEditor);
            grid.Children.Add(menuButton);
            Content = grid;
        }

        private void MenuButton_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            e.Handled = true;
            OpenMenu();
        }

        private void MenuButton_Click(object sender, RoutedEventArgs e)
        {
            e.Handled = true;
            OpenMenu();
        }

        private void OpenMenu()
        {
            if (openMenu is { IsOpen: true })
            {
                openMenu.IsOpen = false;
                return;
            }

            var menu = new ContextMenu
            {
                PlacementTarget = menuButton,
                Placement = PlacementMode.Bottom,
            };

            Guid? sceneId = SceneIdProvider?.Invoke();
            var names = Kind == NameKind.Trigger ? PhysicsManager.GetTriggerNames(sceneId) : PhysicsManager.GetBodyNames(sceneId);
            if (names.Count == 0)
            {
                menu.Items.Add(new MenuItem { Header = CreateHeader(Kind == NameKind.Trigger ? "名前の付いた物理トリガーがありません" : "名前の付いた物理演算エフェクトがありません"), IsEnabled = false });
            }
            else
            {
                var root = new NameNode(string.Empty);
                foreach (var name in names) root.Add(name);
                AddItems(menu.Items, root, string.Empty);
            }

            menu.Closed += (_, _) =>
            {
                if (openMenu == menu) openMenu = null;
            };
            openMenu = menu;
            menu.IsOpen = true;
        }

        private void AddItems(ItemCollection items, NameNode node, string prefix)
        {
            bool hasHeader = false;
            if (AllowsGroup && prefix.Length > 0)
            {
                items.Add(CreateSelectItem($"「{prefix}」で始まるすべて", prefix));
                hasHeader = true;
            }
            if (node.FullName != null)
            {
                items.Add(CreateSelectItem(node.FullName, node.FullName));
                hasHeader = true;
            }
            if (hasHeader) items.Add(new Separator());

            foreach (var child in node.Children)
            {
                if (child.Children.Count == 0)
                {
                    items.Add(CreateSelectItem(child.Label, child.FullName!));
                    continue;
                }

                var folder = new MenuItem { Header = CreateHeader(child.Label) };
                AddItems(folder.Items, child, prefix + child.Label + PathSeparator);
                items.Add(folder);
            }
        }

        private MenuItem CreateSelectItem(string label, string name)
        {
            var item = new MenuItem
            {
                Header = CreateHeader(label),
                IsChecked = TextEditor.Text == name,
            };
            item.Click += (_, _) => Select(name);
            return item;
        }

        private static TextBlock CreateHeader(string label)
        {
            return new TextBlock { Text = label };
        }

        private void Select(string name)
        {
            BeginEdit?.Invoke(this, EventArgs.Empty);
            TextEditor.Text = name;
            EndEdit?.Invoke(this, EventArgs.Empty);
        }

        private sealed class NameNode(string label)
        {
            public string Label { get; } = label;

            public string? FullName { get; private set; }

            public List<NameNode> Children { get; } = [];

            public void Add(string name)
            {
                var node = this;
                foreach (var part in name.Split(PathSeparator))
                {
                    node = node.GetChild(part);
                }
                node.FullName = name;
            }

            private NameNode GetChild(string label)
            {
                foreach (var child in Children)
                {
                    if (child.Label == label) return child;
                }

                var node = new NameNode(label);
                Children.Add(node);
                return node;
            }
        }
    }
}
