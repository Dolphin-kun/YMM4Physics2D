using System.Numerics;
using YMM4Physics2D.Core.Bodies;

namespace YMM4Physics2D.Core.Colliders
{
    public abstract class Collider
    {
        protected bool IsDirty { get; set; } = true;

        public ShapeType ShapeType { get; protected set; }
        public RigidBody? Body { get; protected set; }
        public Vector2 Offset { get; set; } = Vector2.Zero;
        public AABB WorldAABB { get; protected set; }

        public abstract float CalculateArea();
        public abstract float CalculateInertia(float mass);
        public abstract void RecomputeAABB();

        public virtual void MarkDirty()
        {
            IsDirty = true;
        }

        public virtual void SetBody(RigidBody body)
        {
            Body = body;
        }
    }
}
