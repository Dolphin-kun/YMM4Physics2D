using System.Numerics;

namespace YMM4Physics2D.Core.Colliders
{
    public class CircleCollider : Collider
    {
        public float Radius { get; set; }

        private Vector2 _worldCenter;

        public CircleCollider(float radius, Vector2 offset = default)
        {
            ShapeType = ShapeType.Circle;
            Radius = radius;
            Offset = offset;
        }

        public void SetRadius(float radius)
        {
            Radius = radius;
            MarkDirty();
        }

        public Vector2 WorldCenter
        {
            get
            {
                if (IsDirty) UpdateCache();
                return _worldCenter;
            }
        }

        public override void RecomputeAABB()
        {
            if (IsDirty) UpdateCache();

            WorldAABB = new AABB
            {
                Min = new Vector2(_worldCenter.X - Radius, _worldCenter.Y - Radius),
                Max = new Vector2(_worldCenter.X + Radius, _worldCenter.Y + Radius)
            };
        }

        public override float CalculateArea()
        {
            return MathF.PI * Radius * Radius;
        }

        public override float CalculateInertia(float mass)
        {
            return 0.5f * mass * Radius * Radius;
        }

        private void UpdateCache()
        {
            if (Body == null)
            {
                _worldCenter = Offset;
                RecomputeAABB();
                IsDirty = false;
                return;
            }

            float rotation = Body.Rotation;
            Vector2 position = Body.Position;

            Matrix3x2 transform = Matrix3x2.CreateRotation(rotation);
            transform.Translation = position;

            _worldCenter = Vector2.Transform(Offset, transform);

            WorldAABB = new AABB
            {
                Min = new Vector2(_worldCenter.X - Radius, _worldCenter.Y - Radius),
                Max = new Vector2(_worldCenter.X + Radius, _worldCenter.Y + Radius)
            };

            IsDirty = false;
        }
    }
}
