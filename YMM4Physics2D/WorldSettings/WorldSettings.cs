using System.ComponentModel;
using System.Numerics;
using YMM4Physics2D.Core.World;
using YukkuriMovieMaker.Plugin;

namespace YMM4Physics2D.WorldSettings
{
    internal class WorldSettings : SettingsBase<WorldSettings>
    {
        public override SettingsCategory Category => SettingsCategory.None;
        public override string Name => "ワールド設定";

        public override bool HasSettingView => false;
        public override object? SettingView => null;

        public List<WorldConfig> Configs { get; set; } = [];

        // ワールドID
        private int worldId = 0;
        public int WorldId
        { 
            get => worldId; 
            set
            {
                if (Set(ref worldId, value))
                {
                    OnPropertyChanged(string.Empty);
                }
            }
        }

        private WorldConfig GetCurrentConfig()
        {
            var config = Configs.FirstOrDefault(c => c.Id == worldId);

            if(config == null)
            {
                config = new WorldConfig
                {
                    Id = worldId
                };
                Configs.Add(config);
            }
            return config;
        }

        // 重力
        public Vector2 Gravity
        {
            get => GetCurrentConfig().Gravity;
            set
            {
                var config = GetCurrentConfig();
                if (config.Gravity != value)
                {
                    config.Gravity = value;
                    OnPropertyChanged(nameof(Gravity));
                    OnPropertyChanged(nameof(GravityX));
                    OnPropertyChanged(nameof(GravityY));
                    Save();
                }
            }
        }
        [DefaultValue(0f)]
        public float GravityX { get => Gravity.X / 100f; set => Gravity = new Vector2(value * 100f, Gravity.Y); }
        [DefaultValue(9.8f)]
        public float GravityY { get => Gravity.Y / 100f; set => Gravity = new Vector2(Gravity.X, value * 100f); }

        // 画面サイズの壁
        public bool EnableWalls
        {
            get => GetCurrentConfig().EnableWalls;
            set
            {
                var config = GetCurrentConfig();
                if (config.EnableWalls != value)
                {
                    config.EnableWalls = value;
                    OnPropertyChanged(nameof(EnableWalls));
                    Save();
                }
            }
        }

        // スナップショット間隔
        [DefaultValue(60)]
        public int SnapshotInterval
        {
            get => GetCurrentConfig().SnapshotInterval;
            set
            {
                var config = GetCurrentConfig();
                if (config.SnapshotInterval != value)
                {
                    config.SnapshotInterval = value;
                    OnPropertyChanged(nameof(SnapshotInterval));
                    Save();
                }
            }
        }

        // 衝突の解決回数
        [DefaultValue(4)]
        public int Iterations
        { 
            get => GetCurrentConfig().Iterations;  
            set
            {
                var config = GetCurrentConfig();
                if (config.Iterations != value)
                {
                    config.Iterations = value;
                    OnPropertyChanged(nameof(Iterations));
                    Save();
                }
            }
        }

        // 移動減衰
        [DefaultValue(0.05f)]
        public float LinearDamping 
        { 
            get => GetCurrentConfig().LinearDamping;
            set
            {
                var config = GetCurrentConfig();
                if (config.LinearDamping != value)
                {
                    config.LinearDamping = value;
                    OnPropertyChanged(nameof(LinearDamping));
                    Save();
                }
            }
        }

        // 回転減衰
        [DefaultValue(3.0f)]
        public float AngularDamping 
        { 
            get => GetCurrentConfig().AngularDamping;
            set
            {
                var config = GetCurrentConfig();
                if (config.AngularDamping != value)
                {
                    config.AngularDamping = value;
                    OnPropertyChanged(nameof(AngularDamping));
                    Save();
                }
            }
        }

        // 跳ね返り処理の無視
        [DefaultValue(1.0f)]
        public float MinVelocityForRestitution
        {
            get => GetCurrentConfig().MinVelocityForRestitution;
            set
            {
                var config = GetCurrentConfig();
                if (config.MinVelocityForRestitution != value)
                {
                    config.MinVelocityForRestitution = value;
                    OnPropertyChanged(nameof(MinVelocityForRestitution));
                    Save();
                }
            }
        }

        // めり込み解消割合
        [DefaultValue(0.4f)]
        public float PositionCorrectionPercent
        {
            get => GetCurrentConfig().PositionCorrectionPercent;
            set
            {
                var config = GetCurrentConfig();
                if (config.PositionCorrectionPercent != value)
                {
                    config.PositionCorrectionPercent = value;
                    OnPropertyChanged(nameof(PositionCorrectionPercent));
                    Save();
                }
            }
        }

        // 許容めり込み深さ
        [DefaultValue(1.0f)]
        public float PositionCorrectionSlop
        {
            get => GetCurrentConfig().PositionCorrectionSlop * 100f;
            set
            {
                var config = GetCurrentConfig();
                var val = value / 100f;
                if (config.PositionCorrectionSlop != val)
                {
                    config.PositionCorrectionSlop = val;
                    OnPropertyChanged(nameof(PositionCorrectionSlop));
                    Save();
                }
            }
        }

        public override void Initialize()
        {
        }
    }
}
