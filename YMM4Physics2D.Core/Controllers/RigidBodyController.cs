using System.Diagnostics;
using System.Numerics;
using Vortice;
using Vortice.Direct2D1;
using Vortice.Mathematics;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Colliders;
using YMM4Physics2D.Core.Core;
using YMM4Physics2D.Core.World;

namespace YMM4Physics2D.Core.Controllers
{
    public class RigidBodyController : IDisposable
    {
        private readonly List<RigidBody> _bodies = [];
        public IReadOnlyList<RigidBody> Bodies => _bodies;

        private int _worldId = -1;
        private bool _isInitialized = false;
        private bool _disposed = false;

        private ID2D1Image? _lastInput;
        private Vector2 _lastZoom;
        private float _lastSimplifyTolerance;
        private bool _lastSeparateParts;

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
            bool isSeparateChanged = (separateParts != _lastSeparateParts);

            if (isInputChanged || isZoomChanged || isToleranceChanged || isSeparateChanged || _bodies.Count == 0)
            {
                RebuildBodies(context, input, zoom, simplifyTolerance, alphaThreshold, separateParts);

                _lastInput = input;
                _lastZoom = zoom;
                _lastSimplifyTolerance = simplifyTolerance;
                _lastSeparateParts = separateParts;
            }
        }

        public void Draw(ID2D1DeviceContext target, ID2D1Image image)
        {
            if (_bodies.Count == 0 || image == null) return;

            var bounds = target.GetImageLocalBounds(image);
            var topLeft = new Vector2(bounds.Left, bounds.Top);

            using var factory = target.Factory;

            foreach (var body in _bodies)
            {
                Matrix3x2 worldMatrix = Matrix3x2.CreateRotation(body.Rotation) * Matrix3x2.CreateTranslation(body.Position);

                Vector2 textureShift = -(body.VisualOffset + topLeft);
                Matrix3x2 textureMatrix = Matrix3x2.CreateTranslation(textureShift) * worldMatrix;

                if (!_lastSeparateParts)
                {
                    target.Transform = textureMatrix;
                    target.DrawImage(image);
                }
                else
                {
                    using var pathGeometry = factory.CreatePathGeometry();
                    using var sink = pathGeometry.Open();
                    bool hasFigure = false;

                    foreach (var col in body.Colliders)
                    {
                        if (col is PolygonCollider poly && poly.LocalVertices.Length > 0)
                        {
                            sink.BeginFigure(poly.LocalVertices[0], FigureBegin.Filled);
                            for (int i = 1; i < poly.LocalVertices.Length; i++)
                            {
                                sink.AddLine(poly.LocalVertices[i]);
                            }
                            sink.EndFigure(FigureEnd.Closed);
                            hasFigure = true;
                        }
                        else if (col is CircleCollider circle)
                        {
                            Vector2 center = circle.Offset;
                            float r = circle.Radius;

                            Vector2 start = new Vector2(center.X + r, center.Y);
                            Vector2 end = new Vector2(center.X - r, center.Y);

                            sink.BeginFigure(start, FigureBegin.Filled);
                            sink.AddArc(new ArcSegment(end, new Size(r, r), 0, SweepDirection.Clockwise, ArcSize.Small));
                            sink.AddArc(new ArcSegment(start, new Size(r, r), 0, SweepDirection.Clockwise, ArcSize.Small));
                            sink.EndFigure(FigureEnd.Closed);
                            hasFigure = true;
                        }
                    }
                    sink.Close();

                    if (!hasFigure) continue;

                    var layerParameters = new LayerParameters1
                    {
                        ContentBounds = new RawRectF(-10000, -10000, 20000, 20000),
                        GeometricMask = pathGeometry,
                        MaskTransform = Matrix3x2.Identity,
                        Opacity = 1f
                    };

                    target.Transform = worldMatrix;

                    using var layer = target.CreateLayer(null);
                    target.PushLayer(layerParameters, layer);

                    target.Transform = textureMatrix;
                    target.DrawImage(image);

                    target.PopLayer();
                }
            }
        }

        private void RebuildBodies(ID2D1DeviceContext context, ID2D1Image input, Vector2 zoom, float simplifyTolerance, byte alphaThreshold, bool separateParts)
        {
            var rect = context.GetImageLocalBounds(input);
            var topLeft = new Vector2(rect.Left, rect.Top);

            PhysicsManager.Atomic(_worldId, (world) =>
            {
                var sw = Stopwatch.StartNew();
                foreach (var oldBody in _bodies)
                {
                    PhysicsManager.LeaveWorld(_worldId, oldBody);
                    world.RemoveBody(oldBody);
                }
                _bodies.Clear();
                
                var partsVertices = ColliderGenerator.GetVerticesFromImage(
                    context,
                    input,
                    topLeft,
                    zoom,
                    alphaThreshold,
                    simplifyTolerance
                );

                var newBodies = BodyFactory.CreateBodies(
                    partsVertices,
                    Vector2.Zero,
                    _currentMass,
                    separateParts,
                    _currentBodyType
                );

                foreach (var body in newBodies)
                {
                    Vector2 localCenter = body.VisualOffset + topLeft;
                    Vector2 worldOffset = Vector2.Transform(localCenter, Matrix3x2.CreateRotation(_currentDrawRotation));

                    body.Position = _currentDrawPosition + worldOffset;
                    body.Rotation = _currentDrawRotation;

                    body.OnReset = () =>
                    {
                        Vector2 currentLocalCenter = body.VisualOffset + topLeft;
                        Vector2 currentWorldOffset = Vector2.Transform(currentLocalCenter, Matrix3x2.CreateRotation(_currentDrawRotation));

                        body.Position = _currentDrawPosition + currentWorldOffset;
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
                    foreach (var body in _bodies)
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
