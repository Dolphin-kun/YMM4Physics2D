using Vortice.Direct2D1;
using YMM4Physics2D.Core.World;

namespace YMM4Physics2D.Core.Controllers
{
    public sealed class AreaInput
    {
        public Guid SceneId { get; init; }
        public int WorldId { get; init; }
        public int TimelineFrame { get; init; }
        public int LocalFrame { get; init; }
        public int Length { get; init; }
        public int Layer { get; init; }
        public int GroupIndex { get; init; }
        public int InputIndex { get; init; }
        public TrackObservation Observation { get; init; }
        public string Name { get; init; } = string.Empty;
        public AreaDefinition Definition { get; init; }
        public AreaParameters Parameters { get; init; }
        public required AreaParameterProvider ParameterProvider { get; init; }
        public Func<TrackSettings>? SettingsProvider { get; init; }
    }

    public sealed class AreaController(object owner) : IDisposable
    {
        private static readonly BodyParameterProvider NoBodyParameters = static (_, _, _) => default;

        private PhysicsWorld? world;
        private BodyTrack? track;
        private (Guid SceneId, int WorldId) worldKey;
        private (int GroupIndex, int InputIndex) trackIndex;
        private bool disposed;

        public void Update(ID2D1DeviceContext context, ID2D1Image input, AreaInput data)
        {
            if (disposed || PhysicsManager.IsUiThread) return;

            var newWorldKey = (data.SceneId, data.WorldId);
            var newTrackIndex = (data.GroupIndex, data.InputIndex);
            if (world == null || track == null || worldKey != newWorldKey || trackIndex != newTrackIndex)
            {
                if (world != null && track != null) world.Detach(track, this, remove: worldKey != newWorldKey);

                (world, track) = PhysicsManager.Attach(data.SceneId, data.WorldId, new TrackKey(owner, data.GroupIndex, data.InputIndex), this);
                worldKey = newWorldKey;
                trackIndex = newTrackIndex;
            }

            if (world.IsSecondary(track, this)) return;
            PhysicsManager.SweepIfDue();

            var bounds = context.GetImageLocalBounds(input);

            world.UpdateTrack(track, new TrackUpdate
            {
                StartFrame = data.TimelineFrame - data.LocalFrame,
                Length = data.Length,
                Layer = data.Layer,
                LocalFrame = data.LocalFrame,
                BodyType = Bodies.BodyType.Static,
                Observation = data.Observation,
                ParameterProvider = NoBodyParameters,
                SettingsProvider = data.SettingsProvider,
                Name = data.Name,
                Area = data.Definition,
                AreaBounds = new AreaBounds(bounds.Left, bounds.Top, bounds.Right, bounds.Bottom),
                AreaParameters = data.Parameters,
                AreaParameterProvider = data.ParameterProvider,
            }, this);
        }

        public void Dispose()
        {
            if (disposed) return;
            disposed = true;

            if (world != null && track != null) world.Detach(track, this, remove: false);
            world = null;
            track = null;
        }
    }
}
