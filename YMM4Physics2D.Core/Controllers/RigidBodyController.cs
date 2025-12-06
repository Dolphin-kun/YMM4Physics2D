using System.Numerics;
using Vortice.Direct2D1;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Colliders;
using YMM4Physics2D.Core.Core;
using YMM4Physics2D.Core.World;

namespace YMM4Physics2D.Core.Controllers
{
    public class RigidBodyController : IDisposable
    {
        private List<RigidBody> _bodies = [];
        public IReadOnlyList<RigidBody> Bodies => _bodies;

        private int _worldId = -1;
        private bool _isInitialized = false;
        private bool _disposed = false;

        private ID2D1Image? _lastInput;
        private Vector2 _lastZoom;
        private float _lastSimplifyTolerance;

        private Vector2 _currentDrawPosition;
        private float _currentDrawRotation;

        private float _currentMass = 20.0f;
        private BodyType _currentBodyType;

        public RigidBodyController(int worldId, WorldConfig? config, Vector2 position)
        {
            _currentDrawPosition = position;
            _currentDrawRotation = 0f;

            JoinWorld(worldId, config);
        }

        public void Step(int timelineFrame, int itemFrame, double fps, Vector2 targetPos, float targetRot, BodyType type)
        {
            if (_disposed) return;

            _currentDrawPosition = targetPos;
            _currentDrawRotation = targetRot;
            _currentBodyType = type;

            foreach (var body in _bodies)
            {
                body.StartFrame = timelineFrame - itemFrame;

                if (body.Type != type)
                {
                    body.Type = type;
                }
            }

            if (!_isInitialized || itemFrame == 0)
            {
                foreach (var body in _bodies) body.OnReset?.Invoke();

                PhysicsManager.StepWorld(_worldId, timelineFrame, fps);
                _isInitialized = true;
            }
            else
            {
                if (type != BodyType.Static)
                {
                    PhysicsManager.StepWorld(_worldId, timelineFrame, fps);
                }
            }
        }

        public void UpdateBodyProperties(float mass = 20.0f, float restitution = 0.5f, float friction = 0.5f, float linearDamping = 0.05f, float angularDamping = 3.0f)
        {
            _currentMass = mass;

            PhysicsManager.Atomic(_worldId, (world) =>
            {
                float massPerBody = (_bodies.Count > 0) ? mass / _bodies.Count : mass;

                foreach (var body in _bodies)
                {
                    body.Mass = massPerBody;

                    body.Restitution = restitution;
                    body.Friction = friction;
                    body.LinearDamping = linearDamping;
                    body.AngularDamping = angularDamping;
                }
            });
        }

        public void SyncShape(ID2D1DeviceContext context, ID2D1Image? input, Vector2 zoom, byte alphaThreshold = 20, bool separateParts = false, float simplifyTolerance = 1.5f)
        {
            if (_disposed || input == null) return;

            bool isInputChanged = (input != _lastInput);
            bool isZoomChanged = (zoom != _lastZoom);
            bool isToleranceChanged = (simplifyTolerance != _lastSimplifyTolerance);

            if (isInputChanged || isZoomChanged || isToleranceChanged || _bodies.Count == 0)
            {
                RebuildBodies(context, input, zoom, simplifyTolerance, alphaThreshold, separateParts); 
                
                _lastInput = input;
                _lastZoom = zoom;
                _lastSimplifyTolerance = simplifyTolerance;
            }
        }

        private void RebuildBodies(ID2D1DeviceContext context, ID2D1Image input, Vector2 zoom, float simplifyTolerance, byte alphaThreshold, bool separateParts)
        {
            var rect = context.GetImageLocalBounds(input);
            var topLeft = new Vector2(rect.Left, rect.Top);

            PhysicsManager.Atomic(_worldId, (world) =>
            {
                foreach (var oldBody in _bodies)
                {
                    PhysicsManager.LeaveWorld(_worldId, oldBody);
                }
                _bodies.Clear();

                List<Vector2[]> partsVertices = ColliderGenerator.GetVerticesFromImage(
                    context,
                    input,
                    topLeft,
                    zoom,
                    alphaThreshold,
                    simplifyTolerance
                );

                var newBodies = BodyFactory.CreateBodies(
                    partsVertices,
                    _currentDrawPosition,
                    _currentMass,
                    separateParts,
                    _currentBodyType
                );

                foreach(var body in newBodies )
                {
                    body.Rotation = _currentDrawRotation;
                    Vector2 offset = body.Position - _currentDrawPosition;

                    body.OnReset = () =>
                    {
                        body.Position = _currentDrawPosition + offset;
                        body.Rotation = _currentDrawRotation;
                        body.LinearVelocity = Vector2.Zero;
                        body.AngularVelocity = 0f;
                        body.ClearForces();
                    };

                    _bodies.Add(body);
                    PhysicsManager.JoinWorld(_worldId, new WorldConfig { Id = _worldId });
                    world.AddBody(body);
                }
            });
        }

        public void SetWorld(int worldId, WorldConfig? config)
        {
            if (_worldId != worldId)
            {
                if (_worldId != -1)
                {
                    foreach (var body in _bodies)
                    {
                        PhysicsManager.LeaveWorld(_worldId, body);
                    }
                }

                JoinWorld(worldId, config);
                foreach (var b in _bodies)
                {
                    var w = PhysicsManager.JoinWorld(worldId, config ?? new WorldConfig { Id = worldId });
                    w.AddBody(b);
                }

                _isInitialized = false;
            }
        }

        private void JoinWorld(int worldId, WorldConfig? config)
        {
            _worldId = worldId;
            PhysicsManager.JoinWorld(worldId, config ?? new WorldConfig { Id = worldId });
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;

            if (disposing)
            {
                if (_worldId != -1)
                {
                    foreach(var body in _bodies)
                    {
                        PhysicsManager.LeaveWorld(_worldId, body);
                    }
                    _worldId = -1;
                }
            }
            _disposed = true;
        }
    }
}
