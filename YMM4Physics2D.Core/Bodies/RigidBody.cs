using System.Numerics;
using YMM4Physics2D.Core.Colliders;

namespace YMM4Physics2D.Core.Bodies
{
    public class RigidBody
    {
        private static int _nextId = 0;
        public readonly int Id;

        private Vector2 _position;
        public Vector2 Position
        {
            get => _position;
            set
            {
                if (_position != value)
                {
                    _position = value;
                    MarkDirtyColliders();
                }
            }
        }

        private float _rotation;
        public float Rotation
        {
            get => _rotation;
            set
            {
                if (_rotation != value)
                {
                    _rotation = value;
                    MarkDirtyColliders();
                }
            }
        }

        public Vector2 LinearVelocity;
        public float AngularVelocity;

        public Vector2 Force;
        public float Torque;

        private float _mass;
        public float Mass
        {
            get => _mass;
            set
            {
                if (value < 0f) value = 0f;

                if (_mass != value)
                {
                    _mass = value;
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

        private BodyType _type;
        public BodyType Type
        {
            get => _type;
            set
            {
                if (_type != value)
                {
                    _type = value;
                    RecalculateMassData();
                }
            }
        }

        private readonly List<Collider> _colliders = [];
        public IReadOnlyList<Collider> Colliders => _colliders;

        public AABB WorldAABB { get; private set; }
        public Vector2 VisualOffset { get; set; }

        public bool IsActive { get; set; } = true;
        public int StartFrame { get; set; } = 0;
        public Action? OnReset { get; set; }

        public RigidBody(Vector2 position, float mass, BodyType bodyType)
        {
            Id = Interlocked.Increment(ref _nextId);
            _position = position;
            _type = bodyType;
            _mass = System.Math.Max(0f, mass);
            WorldAABB = new AABB(position, position);

            RecalculateMassData();
        }

        public void AddForce(Vector2 force)
        {
            if (Type == BodyType.Static) return;
            Force += force;
        }

        public void ClearForces()
        {
            Force = Vector2.Zero;
            Torque = 0f;
        }

        public void AddCollider(Collider collider)
        {
            if (collider == null) return;

            if (!_colliders.Contains(collider))
            {
                collider.SetBody(this);
                _colliders.Add(collider);
                collider.MarkDirty();
                UpdateWorldAABB();
                RecalculateMassData();
            }
        }

        public void AddColliders(IEnumerable<Collider> colliders)
        {
            bool addedAny = false;
            foreach (var collider in colliders)
            {
                if (collider != null && !_colliders.Contains(collider))
                {
                    collider.SetBody(this);
                    _colliders.Add(collider);
                    collider.MarkDirty();
                    addedAny = true;
                }
            }

            if (addedAny)
            {
                UpdateWorldAABB();
                RecalculateMassData();
            }
        }

        public void RemoveCollider(Collider collider)
        {
            if (_colliders.Remove(collider))
            {
                UpdateWorldAABB();
                RecalculateMassData();
            }
        }

        public void ClearColliders()
        {
            _colliders.Clear();
            UpdateWorldAABB();
            ComputeInertia();
        }

        public void UpdateWorldAABB()
        {
            if (_colliders.Count == 0)
            {
                WorldAABB = new AABB(_position, _position);
                return;
            }

            Vector2 min = new(float.MaxValue, float.MaxValue);
            Vector2 max = new(float.MinValue, float.MinValue);

            foreach (var col in _colliders)
            {
                AABB box = col.WorldAABB;
                min = Vector2.Min(min, box.Min);
                max = Vector2.Max(max, box.Max);
            }

            WorldAABB = new AABB(min, max);
        }

        public void ComputeInertia()
        {
            if (Type == BodyType.Static || _mass <= 0.0001f || _colliders.Count == 0)
            {
                InvInertia = 0f;
                return;
            }

            float totalArea = 0f;

            foreach (var col in _colliders)
            {
                totalArea += col.CalculateArea();
            }

            if (totalArea < 0.0001f)
            {
                InvInertia = 0f;
                return;
            }

            float totalInertia = 0f;

            foreach (var col in _colliders)
            {
                float area = col.CalculateArea();
                float areaRatio = area / totalArea;
                float partMass = Mass * areaRatio;

                totalInertia += col.CalculateInertia(partMass);
            }

            InvInertia = totalInertia > 0f ? 1.0f / totalInertia : 0f;
        }

        private void RecalculateMassData()
        {
            if (Type == BodyType.Static || _mass <= 0.0001f)
            {
                InvMass = 0f;
                InvInertia = 0f;
            }
            else
            {
                InvMass = 1.0f / _mass;
                ComputeInertia();
            }
        }

        private void MarkDirtyColliders()
        {
            foreach (var collider in _colliders)
            {
                collider?.MarkDirty();
            }
        }
    }
}
