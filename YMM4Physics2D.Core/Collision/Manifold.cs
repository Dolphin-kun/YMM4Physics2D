using System.Numerics;
using YMM4Physics2D.Core.Bodies;

namespace YMM4Physics2D.Core.Collision
{
    public struct Manifold(RigidBody a, RigidBody b)
    {
        public RigidBody BodyA = a;
        public RigidBody BodyB = b;

        public Vector2 Normal = Vector2.Zero;
        public float Depth = 0f;
        public Vector2 Contact1 = Vector2.Zero;
        public Vector2 Contact2 = Vector2.Zero;
        public int ContactCount = 0;

        public void AddContact(Vector2 point)
        {
            if (ContactCount == 0)
            {
                Contact1 = point;
                ContactCount = 1;
            }
            else if (ContactCount == 1)
            {
                Contact2 = point;
                ContactCount = 2;
            }
        }
    }
}
