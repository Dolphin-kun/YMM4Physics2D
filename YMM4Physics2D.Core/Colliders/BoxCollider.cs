using System.Numerics;

namespace YMM4Physics2D.Core.Colliders
{
    public class BoxCollider : Collider
    {
        private readonly Vector2[] _worldVertices = new Vector2[4];
        private readonly Vector2[] _worldAxes = new Vector2[2];

        public float Width { get; private set; }
        public float Height { get; private set; }

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
                return _worldAxes;
            }
        }

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

        public override void RecomputeAABB()
        {
            if (IsDirty)
            {
                UpdateCache();
                return;
            }

            Vector2 min = _worldVertices[0];
            Vector2 max = _worldVertices[0];

            for (int i = 1; i < 4; i++)
            {
                Vector2 v = _worldVertices[i];
                min = Vector2.Min(min, v);
                max = Vector2.Max(max, v);
            }

            WorldAABB = new AABB(min, max);
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
                float hw = Width / 2f;
                float hh = Height / 2f;
                _worldVertices[0] = Offset + new Vector2(-hw, -hh);
                _worldVertices[1] = Offset + new Vector2(hw, -hh);
                _worldVertices[2] = Offset + new Vector2(hw, hh);
                _worldVertices[3] = Offset + new Vector2(-hw, hh);

                _worldAxes[0] = Vector2.UnitX;
                _worldAxes[1] = Vector2.UnitY;
            }
            else
            {

                float rotation = Body.Rotation;
                Vector2 position = Body.Position;

                Matrix3x2 transform = Matrix3x2.CreateRotation(rotation);
                transform.Translation = position;

                _worldAxes[0] = Vector2.Normalize(new Vector2(transform.M11, transform.M12));
                _worldAxes[1] = Vector2.Normalize(new Vector2(transform.M21, transform.M22));

                float halfWidth = Width / 2f;
                float halfHeight = Height / 2f;

                _worldVertices[0] = Vector2.Transform(new Vector2(-halfWidth, -halfHeight) + Offset, transform);
                _worldVertices[1] = Vector2.Transform(new Vector2(halfWidth, -halfHeight) + Offset, transform);
                _worldVertices[2] = Vector2.Transform(new Vector2(halfWidth, halfHeight) + Offset, transform);
                _worldVertices[3] = Vector2.Transform(new Vector2(-halfWidth, halfHeight) + Offset, transform);
            }

            Vector2 min = _worldVertices[0];
            Vector2 max = _worldVertices[0];
            for (int i = 1; i < 4; i++)
            {
                min = Vector2.Min(min, _worldVertices[i]);
                max = Vector2.Max(max, _worldVertices[i]);
            }
            WorldAABB = new AABB(min, max);
            IsDirty = false;
        }
    }
}
