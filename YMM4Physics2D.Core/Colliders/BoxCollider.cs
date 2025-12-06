using System.Numerics;

namespace YMM4Physics2D.Core.Colliders
{
    public class BoxCollider : Collider
    {
        public float Width { get; set; }
        public float Height { get; set; }

        private bool _isDirty = true;
        private readonly Vector2[] _worldVertices = new Vector2[4];

        public BoxCollider(float width, float height)
        {
            ShapeType = ShapeType.Box;
            SetSize(width, height);
        }

        public void SetSize(float width, float height)
        {
            Width = width;
            Height = height;

            MarkDirty();
        }

        public Vector2[] WorldVertices
        {
            get
            {
                if (_isDirty) UpdateCache();
                return _worldVertices;
            }
        }

        public override void MarkDirty()
        {
            _isDirty = true;
        }

        public override void RecomputeAABB()
        {
            if (_worldVertices == null || _worldVertices.Length == 0)
            {
                WorldAABB = new AABB { Min = Vector2.Zero, Max = Vector2.Zero };
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

            WorldAABB = new AABB
            {
                Min = new Vector2(minX, minY),
                Max = new Vector2(maxX, maxY)
            };
        }

        public override float CalculateArea()
        {
            return Width * Height;
        }

        public override float CalculateInertia(float mass)
        {
            return mass * (Width * Width + Height * Height) / 12.0f;
        }

        private void UpdateCache()
        {
            if (Body == null)
            {
                RecomputeAABB();
                _isDirty = false;
                return;
            }

            float rotation = Body.Rotation;
            Vector2 position = Body.Position;

            Matrix3x2 transform = Matrix3x2.CreateRotation(rotation);
            transform.Translation = position;

            float halfWidth = Width / 2f;
            float halfHeight = Height / 2f;
            Vector2[] localVertices =
            [
                new Vector2(-halfWidth, -halfHeight),
                new Vector2(halfWidth, -halfHeight),
                new Vector2(halfWidth, halfHeight),
                new Vector2(-halfWidth, halfHeight)
            ];

            for (int i = 0; i < 4; i++)
            {
                Vector2 vertexWithOffset = localVertices[i] + Offset;
                _worldVertices[i] = Vector2.Transform(vertexWithOffset, transform);
            }

            RecomputeAABB();

            _isDirty = false;
        }
    }
}
