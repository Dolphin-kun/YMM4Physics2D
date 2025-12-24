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

        private readonly Dictionary<int, ID2D1PathGeometry> _geometryCache = [];

        private int _worldId = -1;
        private bool _isInitialized = false;
        private bool _disposed = false;

        private ID2D1Image? _lastInput;
        private Vector2 _lastZoom;
        private float _lastSimplifyTolerance;
        private bool _lastSeparateParts;

        private Vector2 _currentDrawPosition;
        private float _currentDrawRotation;
        private BodyType _currentBodyType;
        private float _currentMass = 20.0f;

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
            _currentDrawRotation = targetRot * MathF.PI / 180.0f;
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

            if (!_lastSeparateParts)
            {
                foreach (var body in _bodies)
                {
                    Matrix3x2 worldMatrix = Matrix3x2.CreateRotation(body.Rotation) * Matrix3x2.CreateTranslation(body.Position);

                    Vector2 textureShift = -(body.VisualOffset + topLeft);
                    Matrix3x2 textureMatrix = Matrix3x2.CreateTranslation(textureShift) * worldMatrix;

                    target.Transform = textureMatrix;
                    target.DrawImage(image);
                }
            }
            else
            {
                DrawSeparateParts(target, image, topLeft);
            }
        }

        private void DrawSeparateParts(ID2D1DeviceContext target, ID2D1Image image, Vector2 topLeft)
        {
            using var factory = target.Factory;

            var layerParams = new LayerParameters1
            {
                ContentBounds = new RawRectF(-10000, -10000, 20000, 20000),
                MaskTransform = Matrix3x2.Identity,
                Opacity = 1f
            };

            foreach (var body in _bodies)
            {
                if (!_geometryCache.TryGetValue(body.Id, out var geometry))
                {
                    geometry = CreateBodyGeometry(factory, body);
                    if (geometry == null) continue;
                    _geometryCache[body.Id] = geometry;
                }

                Matrix3x2 worldMatrix = Matrix3x2.CreateRotation(body.Rotation) * Matrix3x2.CreateTranslation(body.Position);
                Vector2 textureShift = -(body.VisualOffset + topLeft);
                Matrix3x2 textureMatrix = Matrix3x2.CreateTranslation(textureShift) * worldMatrix;

                using var layer = target.CreateLayer(null);

                layerParams.GeometricMask = geometry;

                target.Transform = worldMatrix;
                target.PushLayer(layerParams, layer);

                target.Transform = textureMatrix;
                target.DrawImage(image);

                target.PopLayer();
            }
        }

        private static ID2D1PathGeometry? CreateBodyGeometry(ID2D1Factory factory, RigidBody body)
        {
            var pathGeometry = factory.CreatePathGeometry();
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

                    Vector2 start = new(center.X + r, center.Y);
                    Vector2 end = new(center.X - r, center.Y);

                    sink.BeginFigure(start, FigureBegin.Filled);
                    sink.AddArc(new ArcSegment(end, new Size(r, r), 0, SweepDirection.Clockwise, ArcSize.Small));
                    sink.AddArc(new ArcSegment(start, new Size(r, r), 0, SweepDirection.Clockwise, ArcSize.Small));
                    sink.EndFigure(FigureEnd.Closed);
                    hasFigure = true;
                }
            }
            sink.Close();

            if (!hasFigure)
            {
                pathGeometry.Dispose();
                return null;
            }

            return pathGeometry;
        }

        private void RebuildBodies(ID2D1DeviceContext context, ID2D1Image input, Vector2 zoom, float simplifyTolerance, byte alphaThreshold, bool separateParts)
        {
            var rect = context.GetImageLocalBounds(input);
            var topLeft = new Vector2(rect.Left, rect.Top);

            PhysicsManager.Atomic(_worldId, (world) =>
            {
                ClearBodies(world);

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
                    SetupNewBody(body, topLeft);
                    _bodies.Add(body);
                    world.AddBody(body);
                }
            });
        }

        private void SetupNewBody(RigidBody body, Vector2 topLeft)
        {
            Vector2 localCenter = body.VisualOffset + topLeft;
            Vector2 worldOffset = Vector2.Transform(localCenter, Matrix3x2.CreateRotation(_currentDrawRotation));

            body.Position = _currentDrawPosition + worldOffset;
            body.Rotation = _currentDrawRotation;

            Vector2 capturedLocalCenter = localCenter;

            body.OnReset = () =>
            {
                Vector2 currentWorldOffset = Vector2.Transform(capturedLocalCenter, Matrix3x2.CreateRotation(_currentDrawRotation));

                body.Position = _currentDrawPosition + currentWorldOffset;
                body.Rotation = _currentDrawRotation;
                body.LinearVelocity = Vector2.Zero;
                body.AngularVelocity = 0f;
                body.ClearForces();
            };
        }

        private void ClearBodies(PhysicsWorld world)
        {
            foreach (var oldBody in _bodies)
            {
                PhysicsManager.LeaveWorld(_worldId, oldBody);
                world.RemoveBody(oldBody);
            }
            _bodies.Clear();

            foreach (var geo in _geometryCache.Values)
            {
                geo.Dispose();
            }
            _geometryCache.Clear();
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

                foreach (var geo in _geometryCache.Values)
                {
                    geo.Dispose();
                }
                _geometryCache.Clear();
            }
            _disposed = true;
        }
    }
}
