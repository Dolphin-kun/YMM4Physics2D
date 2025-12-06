using System.Drawing;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.World;

namespace YMM4Physics2D.Core.Core
{
    public static class PhysicsManager
    {
        private static readonly Dictionary<int, PhysicsWorld> _worlds = [];
        private static readonly Dictionary<int, int> _userCounts = [];
        private static readonly Dictionary<int, int> _lastFrameCounts = [];
        private static readonly Lock _lock = new();

        public static PhysicsWorld JoinWorld(int worldId, WorldConfig config)
        {
            lock (_lock)
            {
                if (!_worlds.TryGetValue(worldId, out PhysicsWorld? world))
                {
                    world = new PhysicsWorld();
                    _worlds[worldId] = world;
                    _userCounts[worldId] = 0;
                    _lastFrameCounts[worldId] = -1;
                }

                ApplySettings(world, config, null);

                _userCounts[worldId]++;
                return world;
            }
        }

        public static void LeaveWorld(int worldId, RigidBody bodyToRemove)
        {
            lock (_lock)
            {
                if (_worlds.TryGetValue(worldId, out PhysicsWorld? world))
                {
                    world.RemoveBody(bodyToRemove);

                    _userCounts[worldId]--;

                    /*
                    if (_userCounts[worldId] <= 0)
                    {
                        world.Clear();
                        _worlds.Remove(worldId);
                        _userCounts.Remove(worldId);
                        _lastFrameCounts.Remove(worldId);
                    }*/
                }
            }
        }

        public static void DeleteWorld(int worldId)
        {
            lock (_lock)
            {
                if (_worlds.TryGetValue(worldId, out var world))
                {
                    world.Clear();
                    _worlds.Remove(worldId);
                    _userCounts.Remove(worldId);
                    _lastFrameCounts.Remove(worldId);
                }
            }
        }

        public static void UpdateWorldSettings(int worldId, WorldConfig config, Size? screenSize)
        {
            lock (_lock)
            {
                if (_worlds.TryGetValue(worldId, out PhysicsWorld? world))
                {
                    ApplySettings(world, config, screenSize);
                }
            }
        }

        public static void StepWorld(int worldId, int frame, double fps)
        {
            PhysicsWorld? world;

            lock (_lock)
            {
                if (!_worlds.TryGetValue(worldId, out world)) return;
                if (_lastFrameCounts.TryGetValue(worldId, out int lastFrame) && lastFrame == frame) return;

                _lastFrameCounts[worldId] = frame;
            }

            world?.SyncTo(frame, fps);
        }

        public static void Atomic(int worldId, Action<PhysicsWorld> action)
        {
            PhysicsWorld world;

            lock (_lock)
            {
                if (!_worlds.TryGetValue(worldId, out world)) return;
            }

            world.RunAtomic(action);
        }

        private static void ApplySettings(PhysicsWorld world, WorldConfig config, Size? screenSize)
        {
            if (config == null) return;

            world.Gravity = config.Gravity;
            world.Iterations = config.Iterations;
            world.SnapshotInterval = config.SnapshotInterval;
            world.MinVelocityForRestitution = config.MinVelocityForRestitution;
            world.PositionCorrectionPercent = config.PositionCorrectionPercent;
            world.PositionCorrectionSlop = config.PositionCorrectionSlop;

            if (screenSize != null)
            {
                config.ScreenSize = screenSize.Value;
            }
            world.UpdateWalls(config.EnableWalls, config.ScreenSize);
        }
    }
}
