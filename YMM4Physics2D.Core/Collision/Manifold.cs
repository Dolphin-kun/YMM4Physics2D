using System.Numerics;
using YMM4Physics2D.Core.Bodies;

namespace YMM4Physics2D.Core.Collision
{
    public struct Manifold(RigidBody a, RigidBody b)
    {
        public RigidBody BodyA = a;
        public RigidBody BodyB = b;

        public Vector2 Normal = Vector2.Zero;       // 衝突法線
        public float Depth = 0f;                    // めり込み深さ
        public Vector2 Contact1 = Vector2.Zero;     // 衝突点1
        public Vector2 Contact2 = Vector2.Zero;     // 衝突点2
        public int ContactCount = 0;                // 衝突点の数
        public bool HasCollision = false;           // 衝突判定

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
