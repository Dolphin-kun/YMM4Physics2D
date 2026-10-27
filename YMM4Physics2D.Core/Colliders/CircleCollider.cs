using System.Numerics;

namespace YMM4Physics2D.Core.Colliders
{
    public class CircleCollider : Collider
    {
        private Vector2 worldCenter;
        private float radius;

        public float Radius
        {
            get => radius;
            set
            {
                radius = value;
                MarkDirty();
            }
        }

        public Vector2 WorldCenter
        {
            get
            {
                EnsureUpdated();
                return worldCenter;
            }
        }

        public CircleCollider(float radius, Vector2 offset = default)
        {
            Radius = radius;
            Offset = offset;
        }

        public override float MinWidth => Radius * 2f;

        public override float CalculateArea()
        {
            return MathF.PI * Radius * Radius;
        }

        public override float CalculateInertia(float mass)
        {
            return mass * (0.5f * Radius * Radius + Offset.LengthSquared());
        }

        protected override AABB ComputeLocalBounds()
        {
            Vector2 extent = new(Radius);
            return new AABB(Offset - extent, Offset + extent);
        }

        protected override AABB UpdateWorld(in Matrix3x2 transform)
        {
            worldCenter = Vector2.Transform(Offset, transform);
            Vector2 extent = new(Radius);
            return new AABB(worldCenter - extent, worldCenter + extent);
        }
    }
}
