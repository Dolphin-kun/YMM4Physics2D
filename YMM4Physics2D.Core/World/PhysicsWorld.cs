using System.Drawing;
using System.Numerics;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Colliders;
using YMM4Physics2D.Core.Collision;
using static YMM4Physics2D.Core.Core.BodyState;

namespace YMM4Physics2D.Core.World
{
    public class PhysicsWorld
    {
        private readonly List<RigidBody> _bodies = [];
        public IReadOnlyList<RigidBody> Bodies => _bodies;

        private readonly List<RigidBody> _wallBodies = [];
        private readonly List<WorldState> _snapshots = [];

        public Vector2 Gravity { get; set; } = new Vector2(0f, 981.0f);
        public int Iterations { get; set; } = 4;
        public int SnapshotInterval { get; set; } = 60;

        public float MinVelocityForRestitution { get; set; } = 1.0f;
        public float PositionCorrectionPercent { get; set; } = 0.4f;
        public float PositionCorrectionSlop { get; set; } = 0.01f;

        public int CurrentFrame { get; private set; } = 0;

        private bool _lastEnabled = false;
        private Size _lastScreenSize = Size.Empty;
        private bool _isInitialized = false;

        private readonly Lock _lock = new();

        public void AddBody(RigidBody body)
        {
            lock (_lock)
            {
                if (body != null && !_bodies.Contains(body))
                    _bodies.Add(body);
            }
        }

        public void RemoveBody(RigidBody body)
        {
            lock (_lock)
            {
                _bodies.Remove(body);
            }
        }

        public void Clear()
        {
            lock (_lock)
            {
                _bodies.Clear();
                _snapshots.Clear();
                CurrentFrame = 0;
                _isInitialized = false;
            }
        }

        public void Step(float deltaTime)
        {
            lock (_lock)
            {
                IntegrateBodies(deltaTime);

                for (int i = 0; i < Iterations; i++)
                {
                    SolveCollisions();
                }
            }
        }

        public void RunAtomic(Action<PhysicsWorld> action)
        {
            lock (_lock)
            {
                action(this);
            }
        }

        public void SyncTo(int targetFrame, double fps)
        {
            lock (_lock)
            {
                if (targetFrame == 0)
                {
                    RestoreFrameZero();
                    return;
                }

                if (targetFrame < CurrentFrame || targetFrame > CurrentFrame + 60)
                {
                    if (!LoadClosestSnapshot(targetFrame))
                    {
                        RestoreFrameZero();
                    }
                }

                float deltaTime = 1.0f / (float)fps;

                while (CurrentFrame < targetFrame)
                {
                    Step(deltaTime);
                    CurrentFrame++;

                    if (CurrentFrame % SnapshotInterval == 0)
                    {
                        SaveSnapshot(CurrentFrame);
                    }
                }
            }
        }

        public void UpdateWalls(bool enabled, Size screenSize)
        {
            lock (_lock)
            {
                if (_isInitialized && _lastEnabled == enabled && _lastScreenSize == screenSize)
                    return;

                _isInitialized = true;
                _lastEnabled = enabled;
                _lastScreenSize = screenSize;

                foreach (var wall in _wallBodies)
                {
                    _bodies.Remove(wall);
                }
                _wallBodies.Clear();

                if (!enabled) return;

                float thickness = 20.0f;

                CreateWall(new Vector2(0, screenSize.Height / 2), screenSize.Width, thickness);
                CreateWall(new Vector2(0, -screenSize.Height / 2), screenSize.Width, thickness);
                CreateWall(new Vector2(screenSize.Width / 2, 0), thickness, screenSize.Height);
                CreateWall(new Vector2(-screenSize.Width / 2, 0), thickness, screenSize.Height);
            }
        }

        private void IntegrateBodies(float deltaTime)
        {
            for (int i = _bodies.Count - 1; i >= 0; i--)
            {
                var body = _bodies[i];

                if (!body.IsActive) continue;
                if (CurrentFrame < body.StartFrame) continue;

                if (body.Type != BodyType.Static)
                {
                    Vector2 acceleration = (body.Force * body.InvMass) + Gravity;
                    body.LinearVelocity += acceleration * deltaTime;

                    body.Position += body.LinearVelocity * deltaTime;
                    body.Rotation += body.AngularVelocity * deltaTime;

                    body.LinearVelocity *= MathF.Exp(-body.LinearDamping * deltaTime);
                    body.AngularVelocity *= MathF.Exp(-body.AngularDamping * deltaTime);

                    if (MathF.Abs(body.AngularVelocity) < 0.03f)
                    {
                        body.AngularVelocity = 0f;
                    }

                    body.ClearForces();
                }

                foreach (var collider in body.Colliders)
                {
                    collider.RecomputeAABB();
                }

                body.UpdateWorldAABB();
            }
        }

        private void SolveCollisions()
        {
            for (int i = 0; i < _bodies.Count; i++)
            {
                RigidBody bodyA = _bodies[i];
                if (bodyA == null) continue;

                AABB aabbA = bodyA.WorldAABB;

                for (int j = i + 1; j < _bodies.Count; j++)
                {
                    RigidBody bodyB = _bodies[j];

                    if (!bodyB.IsActive) continue;
                    if (bodyA.InvMass == 0f && bodyB.InvMass == 0f) continue;
                    if (CurrentFrame < bodyA.StartFrame || CurrentFrame < bodyB.StartFrame) continue;

                    if (!aabbA.Overlaps(bodyB.WorldAABB))
                    {
                        continue;
                    }

                    List<Manifold> manifolds = CollisionDetection.Detect(bodyA, bodyB);

                    foreach (var m in manifolds)
                    {
                        if (m.HasCollision)
                        {
                            CollisionResolver.Resolve(m, MinVelocityForRestitution, PositionCorrectionPercent, PositionCorrectionSlop);
                        }
                    }
                }
            }
        }

        private void RestoreFrameZero()
        {
            CurrentFrame = 0;
            _snapshots.Clear();

            foreach (var body in _bodies)
            {
                body.OnReset?.Invoke();
                foreach (var col in body.Colliders) col.RecomputeAABB();
                body.UpdateWorldAABB();
            }

            SaveSnapshot(0);
        }

        private void SaveSnapshot(int frame)
        {
            if (_snapshots.Count > 0 && _snapshots[^1].Frame == frame)
            {
                _snapshots[^1] = new WorldState(frame, _bodies);
            }
            else
            {
                _snapshots.Add(new WorldState(frame, _bodies));
            }
        }

        private bool LoadClosestSnapshot(int targetFrame)
        {
            for (int i = _snapshots.Count - 1; i >= 0; i--)
            {
                if (_snapshots[i].Frame <= targetFrame)
                {
                    var snap = _snapshots[i];

                    CurrentFrame = snap.Frame;

                    foreach (var body in _bodies)
                    {
                        if (snap.BodyStates.TryGetValue(body.Id, out var state))
                        {
                            state.ApplyTo(body);
                            foreach (var col in body.Colliders) col.RecomputeAABB();
                            body.UpdateWorldAABB();
                        }
                    }

                    return true;
                }
            }
            return false;
        }

        public void CreateWall(Vector2 position, float width, float height)
        {
            var wall = new RigidBody(position, 0f, BodyType.Static);
            var collider = new BoxCollider(width, height);

            wall.AddCollider(collider);

            _bodies.Add(wall);
            _wallBodies.Add(wall);
        }

        private static AABB GetBodyAABB(RigidBody body)
        {
            Vector2 min = new(float.MaxValue, float.MaxValue);
            Vector2 max = new(float.MinValue, float.MinValue);

            bool hasValidCollider = false;

            foreach (var col in body.Colliders)
            {
                if (col == null) continue;

                hasValidCollider = true;
                AABB box = col.WorldAABB;

                min = Vector2.Min(min, box.Min);
                max = Vector2.Max(max, box.Max);
            }

            if (!hasValidCollider)
            {
                return new AABB(body.Position, body.Position);
            }

            return new AABB(min, max);
        }
    }
}
