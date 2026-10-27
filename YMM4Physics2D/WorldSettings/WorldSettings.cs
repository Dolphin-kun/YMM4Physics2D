using System.Numerics;
using System.Runtime.CompilerServices;
using Newtonsoft.Json;
using YMM4Physics2D.Core.World;
using YukkuriMovieMaker.Plugin;

namespace YMM4Physics2D.WorldSettings
{
    internal class WorldSettings : SettingsBase<WorldSettings>
    {
        private static readonly WorldConfig DefaultConfig = new();

        private readonly Lock configLock = new();
        private int worldId = 0;
        private int version;

        public override SettingsCategory Category => SettingsCategory.None;
        public override string Name => "ワールド設定";

        public override bool HasSettingView => false;
        public override object? SettingView => null;

        public List<WorldConfig> Configs { get; set; } = [];

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

        [JsonIgnore]
        public int Version => Volatile.Read(ref version);

        public WorldConfig? GetConfigSnapshot(int id)
        {
            lock (configLock)
            {
                return Configs.FirstOrDefault(c => c.Id == id)?.Clone();
            }
        }

        [JsonIgnore]
        public Vector2 Gravity
        {
            get => FindCurrentConfig().Gravity;
            set
            {
                var config = GetOrCreateCurrentConfig();
                if (config.Gravity == value) return;

                config.Gravity = value;
                Interlocked.Increment(ref version);
                OnPropertyChanged(nameof(Gravity));
                OnPropertyChanged(nameof(GravityX));
                OnPropertyChanged(nameof(GravityY));
                OnPropertyChanged(nameof(GravityStrength));
                OnPropertyChanged(nameof(GravityAngle));
                Save();
            }
        }

        [JsonIgnore]
        public float GravityX { get => Gravity.X / 100f; set => Gravity = new Vector2(value * 100f, Gravity.Y); }

        [JsonIgnore]
        public float GravityY { get => Gravity.Y / 100f; set => Gravity = new Vector2(Gravity.X, value * 100f); }

        [JsonIgnore]
        public float GravityStrength
        {
            get => Gravity.Length() / 100f;
            set
            {
                float angle = GravityAngle;
                GetOrCreateCurrentConfig().GravityAngle = angle;
                Gravity = CreateGravity(angle, Math.Max(0f, value));
            }
        }

        [JsonIgnore]
        public float GravityAngle
        {
            get
            {
                var gravity = Gravity;
                if (gravity.LengthSquared() < 1e-6f) return FindCurrentConfig().GravityAngle;

                return MathF.Round(MathF.Atan2(-gravity.X, gravity.Y) * 180f / MathF.PI, 2);
            }
            set
            {
                var config = GetOrCreateCurrentConfig();
                if (config.GravityAngle != value)
                {
                    config.GravityAngle = value;
                    Interlocked.Increment(ref version);
                    OnPropertyChanged(nameof(GravityAngle));
                }
                Gravity = CreateGravity(value, Gravity.Length() / 100f);
            }
        }

        [JsonIgnore]
        public bool EnableWalls
        {
            get => FindCurrentConfig().EnableWalls;
            set => Update(value, c => c.EnableWalls, (c, v) => c.EnableWalls = v);
        }

        [JsonIgnore]
        public int SnapshotInterval
        {
            get => FindCurrentConfig().SnapshotInterval;
            set => Update(value, c => c.SnapshotInterval, (c, v) => c.SnapshotInterval = v);
        }

        [JsonIgnore]
        public int Iterations
        {
            get => FindCurrentConfig().Iterations;
            set => Update(value, c => c.Iterations, (c, v) => c.Iterations = v);
        }

        [JsonIgnore]
        public float MinVelocityForRestitution
        {
            get => FindCurrentConfig().MinVelocityForRestitution;
            set => Update(value, c => c.MinVelocityForRestitution, (c, v) => c.MinVelocityForRestitution = v);
        }

        [JsonIgnore]
        public float PositionCorrectionPercent
        {
            get => FindCurrentConfig().PositionCorrectionPercent;
            set => Update(value, c => c.PositionCorrectionPercent, (c, v) => c.PositionCorrectionPercent = v);
        }

        [JsonIgnore]
        public float PositionCorrectionSlop
        {
            get => FindCurrentConfig().PositionCorrectionSlop * 100f;
            set => Update(value / 100f, c => c.PositionCorrectionSlop, (c, v) => c.PositionCorrectionSlop = v);
        }

        public void ResetCurrentWorld()
        {
            lock (configLock)
            {
                int index = Configs.FindIndex(c => c.Id == worldId);
                var config = new WorldConfig { Id = worldId };
                if (index >= 0) Configs[index] = config;
                else Configs.Add(config);
            }

            Interlocked.Increment(ref version);
            OnPropertyChanged(string.Empty);
            Save();
        }

        public override void Initialize()
        {
        }

        private WorldConfig FindCurrentConfig()
        {
            lock (configLock)
            {
                return Configs.FirstOrDefault(c => c.Id == worldId) ?? DefaultConfig;
            }
        }

        private WorldConfig GetOrCreateCurrentConfig()
        {
            lock (configLock)
            {
                var config = Configs.FirstOrDefault(c => c.Id == worldId);
                if (config == null)
                {
                    config = new WorldConfig { Id = worldId };
                    Configs.Add(config);
                }
                return config;
            }
        }

        private void Update<T>(T value, Func<WorldConfig, T> getter, Action<WorldConfig, T> setter, [CallerMemberName] string propertyName = "")
        {
            var config = GetOrCreateCurrentConfig();
            if (EqualityComparer<T>.Default.Equals(getter(config), value)) return;

            setter(config, value);
            Interlocked.Increment(ref version);
            OnPropertyChanged(propertyName);
            Save();
        }

        private static Vector2 CreateGravity(float angle, float strength)
        {
            float radians = angle * MathF.PI / 180f;
            var direction = new Vector2(-MathF.Sin(radians), MathF.Cos(radians));
            var gravity = direction * strength * 100f;

            return new Vector2(MathF.Round(gravity.X, 3), MathF.Round(gravity.Y, 3));
        }
    }
}
