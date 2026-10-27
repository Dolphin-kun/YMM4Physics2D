using System.Numerics;

namespace YMM4Physics2D.Core.Colliders
{
    public class BoxCollider : Collider
    {
        private readonly Vector2[] worldVertices = new Vector2[4];
        private readonly Vector2[] worldNormals = new Vector2[4];

        public float Width { get; }
        public float Height { get; }

        public Vector2[] WorldVertices
        {
            get
            {
                EnsureUpdated();
                return worldVertices;
            }
        }

        public Vector2[] WorldNormals
        {
            get
            {
                EnsureUpdated();
                return worldNormals;
            }
        }

        public BoxCollider(float width, float height)
        {
            Width = width;
            Height = height;
        }

        public override float MinWidth => MathF.Min(Width, Height);

        public override float CalculateArea()
        {
            return Width * Height;
        }

        public override float CalculateInertia(float mass)
        {
            return mass * ((Width * Width + Height * Height) / 12.0f + Offset.LengthSquared());
        }

        protected override AABB ComputeLocalBounds()
        {
            Vector2 half = new(Width / 2f, Height / 2f);
            return new AABB(Offset - half, Offset + half);
        }

        protected override AABB UpdateWorld(in Matrix3x2 transform)
        {
            float halfWidth = Width / 2f;
            float halfHeight = Height / 2f;

            worldVertices[0] = Vector2.Transform(new Vector2(-halfWidth, -halfHeight) + Offset, transform);
            worldVertices[1] = Vector2.Transform(new Vector2(halfWidth, -halfHeight) + Offset, transform);
            worldVertices[2] = Vector2.Transform(new Vector2(halfWidth, halfHeight) + Offset, transform);
            worldVertices[3] = Vector2.Transform(new Vector2(-halfWidth, halfHeight) + Offset, transform);

            worldNormals[0] = Vector2.TransformNormal(-Vector2.UnitY, transform);
            worldNormals[1] = Vector2.TransformNormal(Vector2.UnitX, transform);
            worldNormals[2] = Vector2.TransformNormal(Vector2.UnitY, transform);
            worldNormals[3] = Vector2.TransformNormal(-Vector2.UnitX, transform);

            Vector2 min = worldVertices[0];
            Vector2 max = worldVertices[0];
            for (int i = 1; i < 4; i++)
            {
                min = Vector2.Min(min, worldVertices[i]);
                max = Vector2.Max(max, worldVertices[i]);
            }
            return new AABB(min, max);
        }
    }
}
