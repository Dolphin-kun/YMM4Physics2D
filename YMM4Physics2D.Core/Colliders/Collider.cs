using System.Numerics;
using YMM4Physics2D.Core.Bodies;

namespace YMM4Physics2D.Core.Colliders
{
    public abstract class Collider
    {
        private Vector2 offset;
        private AABB worldAABB;
        private AABB localBounds;
        private bool shapeDirty = true;
        private int cachedVersion = -1;

        public RigidBody? Body { get; private set; }

        public Vector2 Offset
        {
            get => offset;
            set
            {
                offset = value;
                MarkDirty();
            }
        }

        public AABB WorldAABB
        {
            get
            {
                EnsureUpdated();
                return worldAABB;
            }
        }

        public AABB LocalBounds
        {
            get
            {
                if (shapeDirty)
                {
                    localBounds = ComputeLocalBounds();
                    shapeDirty = false;
                    cachedVersion = -1;
                }
                return localBounds;
            }
        }

        public abstract float MinWidth { get; }

        public abstract float CalculateArea();
        public abstract float CalculateInertia(float mass);

        protected abstract AABB ComputeLocalBounds();
        protected abstract AABB UpdateWorld(in Matrix3x2 transform);

        protected void EnsureUpdated()
        {
            _ = LocalBounds;
            int version = Body?.Version ?? 0;
            if (cachedVersion == version) return;

            worldAABB = UpdateWorld(Body?.Transform ?? Matrix3x2.Identity);
            cachedVersion = version;
        }

        public void MarkDirty()
        {
            shapeDirty = true;
            cachedVersion = -1;
            Body?.MarkShapeDirty();
        }

        internal void SetBody(RigidBody body)
        {
            Body = body;
            cachedVersion = -1;
        }
    }
}
