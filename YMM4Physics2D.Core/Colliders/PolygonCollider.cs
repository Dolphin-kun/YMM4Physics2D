using System.Numerics;
using YMM4Physics2D.Core.Math;

namespace YMM4Physics2D.Core.Colliders
{
    public class PolygonCollider : Collider
    {
        public Vector2[] WorldVertices
        {
            get
            {
                if (IsDirty) UpdateCache();
                return _worldVertices;
            }
        }

        public Vector2[] WorldAxes
        {
            get
            {
                if (IsDirty) UpdateCache();
                return _worldNormals;
            }
        }

        public Vector2[] LocalVertices { get; private set; } = [];

        private Vector2[] _worldVertices = [];
        private Vector2[] _worldNormals = [];
        private Vector2[] _localNormals = [];

        public PolygonCollider(Vector2[] vertices, bool autoCenter = true)
        {
            ShapeType = ShapeType.Polygon;
            SetVertices(vertices, autoCenter);
        }

        public void SetVertices(Vector2[] vertices, bool autoCenter = true)
        {
            if (vertices.Length < 3) return;

            if (autoCenter)
            {
                Vector2 centroid = GeometryUtils.CalculateCentroid(vertices);

                LocalVertices = new Vector2[vertices.Length];
                for (int i = 0; i < vertices.Length; i++)
                {
                    LocalVertices[i] = vertices[i] - centroid;
                }
            }
            else
            {
                LocalVertices = (Vector2[])vertices.Clone();
            }

            if (GeometryUtils.CalculateSignedArea(LocalVertices) < 0)
            {
                Array.Reverse(LocalVertices);
            }

            int count = LocalVertices.Length;
            _worldVertices = new Vector2[count];
            _localNormals = new Vector2[count];
            _worldNormals = new Vector2[count];

            for (int i = 0; i < count; i++)
            {
                Vector2 p1 = LocalVertices[i];
                Vector2 p2 = LocalVertices[(i + 1) % count];
                Vector2 edge = p2 - p1;

                _localNormals[i] = Vector2.Normalize(new Vector2(-edge.Y, edge.X));
            }

            MarkDirty();
        }

        private void UpdateCache()
        {
            if (Body == null)
            {
                Array.Copy(LocalVertices, _worldVertices, LocalVertices.Length);
                Array.Copy(_localNormals, _worldNormals, _localNormals.Length);
                IsDirty = false;
                return;
            }

            float rotation = Body.Rotation;
            Vector2 position = Body.Position;

            Matrix3x2 transform = Matrix3x2.CreateRotation(rotation);
            transform.Translation = position;

            Matrix3x2 rotationMatrix = Matrix3x2.CreateRotation(rotation);

            for (int i = 0; i < LocalVertices.Length; i++)
            {
                Vector2 v = LocalVertices[i] + Offset;
                _worldVertices[i] = Vector2.Transform(v, transform);
            }

            for (int i = 0; i < _localNormals.Length; i++)
            {
                _worldNormals[i] = Vector2.Transform(_localNormals[i], rotationMatrix);
            }

            IsDirty = false;
        }

        public override void RecomputeAABB()
        {
            if (IsDirty) UpdateCache();

            if (_worldVertices.Length == 0)
            {
                WorldAABB = new AABB(Vector2.Zero, Vector2.Zero);
                return;
            }

            float minX = float.MaxValue;
            float minY = float.MaxValue;
            float maxX = float.MinValue;
            float maxY = float.MinValue;

            foreach (var v in _worldVertices)
            {
                if (v.X < minX) minX = v.X;
                if (v.Y < minY) minY = v.Y;
                if (v.X > maxX) maxX = v.X;
                if (v.Y > maxY) maxY = v.Y;
            }

            WorldAABB = new AABB(new Vector2(minX, minY), new Vector2(maxX, maxY));
        }

        public override float CalculateArea()
        {
            return MathF.Abs(GeometryUtils.CalculateSignedArea(LocalVertices));
        }

        public override float CalculateInertia(float mass)
        {
            if (LocalVertices == null || LocalVertices.Length == 0) return 0f;

            float numerator = 0f;
            float denominator = 0f;

            for (int i = 0; i < LocalVertices.Length; i++)
            {
                Vector2 p1 = LocalVertices[i];
                Vector2 p2 = LocalVertices[(i + 1) % LocalVertices.Length];

                float cross = MathF.Abs(p1.X * p2.Y - p1.Y * p2.X);

                float dotSum = (p1.X * p1.X + p1.Y * p1.Y) +
                               (p1.X * p2.X + p1.Y * p2.Y) +
                               (p2.X * p2.X + p2.Y * p2.Y);

                numerator += cross * dotSum;
                denominator += cross;
            }

            return denominator > 1e-6f ? (mass / 6.0f) * (numerator / denominator) : 0f;
        }

        public static PolygonCollider CreateBox(float width, float height)
        {
            float hw = width * 0.5f;
            float hh = height * 0.5f;

            var vertices = new Vector2[]
            {
                new(-hw, -hh),
                new( hw, -hh),
                new( hw,  hh),
                new(-hw,  hh)
            };

            return new PolygonCollider(vertices, autoCenter: false);
        }

        

        
    }
}
