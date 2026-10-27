using System.Numerics;
using YMM4Physics2D.Core.Colliders;

namespace YMM4Physics2D.Core.Bodies
{
    public class RigidBody
    {
        private Vector2 position;
        public Vector2 Position
        {
            get => position;
            set
            {
                if (position != value)
                {
                    position = value;
                    Version++;
                }
            }
        }

        private float rotation;
        public float Rotation
        {
            get => rotation;
            set
            {
                if (rotation != value)
                {
                    rotation = value;
                    Version++;
                }
            }
        }

        public Vector2 LinearVelocity;
        public float AngularVelocity;

        public Vector2 Force;

        private float mass;
        public float Mass
        {
            get => mass;
            set
            {
                if (value < 0f) value = 0f;

                if (mass != value)
                {
                    mass = value;
                    RecalculateMassData();
                }
            }
        }

        public float InvMass { get; private set; }
        public float InvInertia { get; private set; }
        public float Restitution { get; set; } = 0.5f;
        public float Friction { get; set; } = 1f;
        public float LinearDamping { get; set; } = 0.05f;
        public float AngularDamping { get; set; } = 3.0f;

        private BodyType type;
        public BodyType Type
        {
            get => type;
            set
            {
                if (type != value)
                {
                    type = value;
                    RecalculateMassData();
                }
            }
        }

        private readonly List<Collider> colliders = [];
        public IReadOnlyList<Collider> Colliders => colliders;

        public AABB WorldAABB { get; private set; }

        public int Version { get; private set; }

        public Matrix3x2 Transform
        {
            get
            {
                var transform = Matrix3x2.CreateRotation(rotation);
                transform.Translation = position;
                return transform;
            }
        }

        private AABB localBounds;
        private float minThickness;
        private bool shapeDirty = true;

        public float MinThickness
        {
            get
            {
                UpdateLocalBounds();
                return minThickness;
            }
        }

        public Vector2 ImageCentroid { get; set; }

        public Vector2 LocalCentroid { get; set; }

        public object? Owner { get; set; }

        public bool IsSpawning { get; set; }


        public float Opacity { get; set; } = 1f;
        public float FadeRate { get; set; }
        public bool IsGhost { get; set; }
        public bool IsFrozen { get; set; }
        public float Presence { get; set; } = 1f;
        public bool TouchingSolid { get; set; }
        public List<long> InsideTriggers { get; } = [];

        public bool IsActive => !IsSpawning && !IsGhost && Presence > 0f;

        public bool IsParticle { get; init; }

        public RigidBody(Vector2 position, float mass, BodyType bodyType)
        {
            this.position = position;
            type = bodyType;
            this.mass = Math.Max(0f, mass);
            WorldAABB = new AABB(position, position);

            RecalculateMassData();
        }

        public void ClearForces()
        {
            Force = Vector2.Zero;
        }

        public void AddColliders(IEnumerable<Collider> newColliders)
        {
            bool addedAny = false;
            foreach (var collider in newColliders)
            {
                if (collider != null && !colliders.Contains(collider))
                {
                    collider.SetBody(this);
                    colliders.Add(collider);
                    addedAny = true;
                }
            }

            if (addedAny)
            {
                shapeDirty = true;
                UpdateWorldAABB();
                RecalculateMassData();
            }
        }

        public void UpdateWorldAABB()
        {
            if (colliders.Count == 0)
            {
                WorldAABB = new AABB(position, position);
                return;
            }

            if (IsParticle && colliders[0] is CircleCollider circle)
            {
                Vector2 center = position + circle.Offset;
                Vector2 extent = new(circle.Radius);
                WorldAABB = new AABB(center - extent, center + extent);
                return;
            }

            UpdateLocalBounds();

            var transform = Transform;
            Vector2 a = Vector2.Transform(localBounds.Min, transform);
            Vector2 b = Vector2.Transform(new Vector2(localBounds.Max.X, localBounds.Min.Y), transform);
            Vector2 c = Vector2.Transform(localBounds.Max, transform);
            Vector2 d = Vector2.Transform(new Vector2(localBounds.Min.X, localBounds.Max.Y), transform);
            WorldAABB = new AABB(Vector2.Min(Vector2.Min(a, b), Vector2.Min(c, d)), Vector2.Max(Vector2.Max(a, b), Vector2.Max(c, d)));
        }

        internal void MarkShapeDirty()
        {
            shapeDirty = true;
        }

        private void UpdateLocalBounds()
        {
            if (!shapeDirty) return;
            shapeDirty = false;

            Vector2 min = new(float.MaxValue);
            Vector2 max = new(float.MinValue);
            float thickness = float.MaxValue;
            foreach (var col in colliders)
            {
                AABB box = col.LocalBounds;
                min = Vector2.Min(min, box.Min);
                max = Vector2.Max(max, box.Max);
                thickness = MathF.Min(thickness, col.MinWidth);
            }

            localBounds = colliders.Count > 0 ? new AABB(min, max) : default;
            minThickness = colliders.Count > 0 ? thickness : float.MaxValue;
        }

        public void ComputeInertia()
        {
            if (Type == BodyType.Static || IsParticle || mass <= 0.0001f || colliders.Count == 0)
            {
                InvInertia = 0f;
                return;
            }

            Span<float> areas = colliders.Count <= 64 ? stackalloc float[colliders.Count] : new float[colliders.Count];
            float totalArea = 0f;
            for (int i = 0; i < colliders.Count; i++)
            {
                areas[i] = colliders[i].CalculateArea();
                totalArea += areas[i];
            }

            if (totalArea < 0.0001f)
            {
                InvInertia = 0f;
                return;
            }

            float totalInertia = 0f;
            for (int i = 0; i < colliders.Count; i++)
            {
                totalInertia += colliders[i].CalculateInertia(mass * areas[i] / totalArea);
            }

            InvInertia = totalInertia > 0f ? 1.0f / totalInertia : 0f;
        }

        private void RecalculateMassData()
        {
            if (Type == BodyType.Static || mass <= 0.0001f)
            {
                InvMass = 0f;
                InvInertia = 0f;
            }
            else
            {
                InvMass = 1.0f / mass;
                ComputeInertia();
            }
        }
    }
}
