using System.Collections.Concurrent;
using System.Diagnostics;
using YMM4Physics2D.Core.World;

namespace YMM4Physics2D.Core
{
    public static class PhysicsManager
    {
        private static readonly ConcurrentDictionary<(Guid SceneId, int WorldId), PhysicsWorld> worlds = new();
        private static readonly Lock registryLock = new();
        private static readonly long SweepInterval = Stopwatch.Frequency * 5;
        private static readonly long IdleLifetime = Stopwatch.Frequency * 60;

        private static long settingsVersion;
        private static long nextSweep;

        [ThreadStatic]
        private static LastUpdate? lastUpdate;

        private sealed record LastUpdate(Guid SceneId, PhysicsWorld World, BodyTrack Track, bool IsSecondary);

        internal static long SettingsVersion => Interlocked.Read(ref settingsVersion);

        public static void NotifySettingsChanged()
        {
            Interlocked.Increment(ref settingsVersion);
        }

        public static bool IsUiThread => System.Windows.Application.Current is { } application && application.Dispatcher.CheckAccess();

        public static bool TryGetTriggerFrame(Guid sceneId, int worldId, string name, int frame, out int fireFrame)
        {
            fireFrame = 0;
            if (!worlds.TryGetValue((sceneId, worldId), out var world)) return false;
            return world.TryGetTriggerFrame(name, frame, !IsUiThread, out fireFrame);
        }

        public static (PhysicsWorld World, BodyTrack Track) Attach(Guid sceneId, int worldId, TrackKey key, object writer)
        {
            lock (registryLock)
            {
                var world = GetWorld(sceneId, worldId);
                return (world, world.Attach(key, writer));
            }
        }

        public static (PhysicsWorld World, BodyTrack Track)? TryAttachObserver(Guid sceneId, int worldId, TrackKey key, object observer)
        {
            if (!registryLock.TryEnter()) return null;
            try
            {
                var world = GetWorld(sceneId, worldId);
                return world.TryAttachObserver(key, observer) is { } track ? (world, track) : null;
            }
            finally
            {
                registryLock.Exit();
            }
        }

        public static void SweepIfDue()
        {
            long now = Stopwatch.GetTimestamp();
            long due = Interlocked.Read(ref nextSweep);
            if (now < due || Interlocked.CompareExchange(ref nextSweep, now + SweepInterval, due) != due) return;
            if (!registryLock.TryEnter()) return;
            try
            {
                foreach (var (key, world) in worlds)
                {
                    if (world.TryRetireIfIdle(now, IdleLifetime)) worlds.TryRemove(key, out _);
                }
            }
            finally
            {
                registryLock.Exit();
            }
        }

        private static PhysicsWorld GetWorld(Guid sceneId, int worldId)
        {
            return worlds.GetOrAdd((sceneId, worldId), static _ => new PhysicsWorld());
        }

        public static IReadOnlyList<string> GetBodyNames(Guid? sceneId)
        {
            return CollectNames(sceneId, static (world, names) => world.CollectBodyNames(names));
        }

        public static IReadOnlyList<string> GetTriggerNames(Guid? sceneId)
        {
            return CollectNames(sceneId, static (world, names) => world.CollectTriggerNames(names));
        }

        private static List<string> CollectNames(Guid? sceneId, Action<PhysicsWorld, ISet<string>> collect)
        {
            var names = new HashSet<string>(StringComparer.Ordinal);
            foreach (var ((worldSceneId, _), world) in worlds)
            {
                if (sceneId is Guid id && worldSceneId != id) continue;
                collect(world, names);
            }
            return names.Order(StringComparer.CurrentCulture).ToList();
        }

        internal static void SetLastUpdate(Guid sceneId, PhysicsWorld world, BodyTrack track, bool isSecondary)
        {
            lastUpdate = new LastUpdate(sceneId, world, track, isSecondary);
        }

        public static bool TryFindTrack(Guid sceneId, int startFrame, int layer, int groupIndex, int inputIndex, out PhysicsWorld world, out BodyTrack track, out bool isSecondary)
        {
            if (lastUpdate is { } last && last.SceneId == sceneId && last.Track.Matches(startFrame, layer, groupIndex, inputIndex))
            {
                world = last.World;
                track = last.Track;
                isSecondary = last.IsSecondary;
                return true;
            }

            bool wait = !IsUiThread;
            isSecondary = !wait;
            foreach (var ((worldSceneId, _), candidate) in worlds)
            {
                if (worldSceneId != sceneId) continue;

                var found = candidate.FindTrack(startFrame, layer, groupIndex, inputIndex, wait);
                if (found != null)
                {
                    world = candidate;
                    track = found;
                    return true;
                }
            }

            world = null!;
            track = null!;
            return false;
        }
    }
}
