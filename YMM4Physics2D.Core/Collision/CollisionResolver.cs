using System.Numerics;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Math;

namespace YMM4Physics2D.Core.Collision
{
    public static class CollisionResolver
    {
        public static void Resolve(Manifold m, float minVelocity, float correctionPercent, float slop)
        {
            RigidBody bodyA = m.BodyA;
            RigidBody bodyB = m.BodyB;

            float invMassSum = bodyA.InvMass + bodyB.InvMass;
            if (invMassSum == 0f) return;

            Vector2 normal = m.Normal;
            Vector2 contact = m.Contact1;

            Vector2 rA = contact - bodyA.Position;
            Vector2 rB = contact - bodyB.Position;

            Vector2 relativeVelocity = CalculateRelativeVelocity(bodyA, bodyB, rA, rB);
            float velocityAlongNormal = Vector2.Dot(relativeVelocity, normal);

            if (velocityAlongNormal > 0)
            {
                return;
            }

            // 反発
            float normalImpulse = ApplyNormalImpulse(bodyA, bodyB, rA, rB, normal, minVelocity, velocityAlongNormal, invMassSum);

            // 摩擦
            ApplyFrictionImpulse(bodyA, bodyB, rA, rB, normal, normalImpulse, invMassSum);

            // めり込み解消
            ApplyPositionalCorrection(bodyA, bodyB, normal, correctionPercent, slop, m.Depth, invMassSum);
        }

        private static Vector2 CalculateRelativeVelocity(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB)
        {
            Vector2 velocityA = bodyA.LinearVelocity + Vector2Extensions.Cross(bodyA.AngularVelocity, rA);
            Vector2 velocityB = bodyB.LinearVelocity + Vector2Extensions.Cross(bodyB.AngularVelocity, rB);
            return velocityB - velocityA;
        }

        private static float ApplyNormalImpulse(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, Vector2 normal, float minVelocity, float velocityAlongNormal, float invMassSum)
        {
            float restitution = MathF.Min(bodyA.Restitution, bodyB.Restitution);
            if (MathF.Abs(velocityAlongNormal) < minVelocity)
            {
                restitution = 0.0f;
            }

            float effectiveInvMass = CalculateEffectiveInvMass(bodyA, bodyB, rA, rB, normal, invMassSum);

            float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal / effectiveInvMass;
            Vector2 impulse = normal * impulseMagnitude;

            ApplyImpulse(bodyA, bodyB, rA, rB, impulse);
            return impulseMagnitude;
        }

        private static void ApplyFrictionImpulse(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, Vector2 normal, float normalImpulseMagnitude, float invMassSum)
        {
            Vector2 relativeVelocity = CalculateRelativeVelocity(bodyA, bodyB, rA, rB);
            Vector2 tangent = relativeVelocity - (normal * Vector2.Dot(relativeVelocity, normal));

            if (tangent.LengthSquared() < 0.0001f)
                return;

            tangent = Vector2.Normalize(tangent);
            float velocityAlongTangent = Vector2.Dot(relativeVelocity, tangent);
            float effectiveInvMass = CalculateEffectiveInvMass(bodyA, bodyB, rA, rB, tangent, invMassSum);
            float frictionImpulseMagnitude = -velocityAlongTangent / effectiveInvMass;

            float mu = MathF.Sqrt(bodyA.Friction * bodyB.Friction);
            float maxFriction = MathF.Abs(normalImpulseMagnitude) * mu;
            frictionImpulseMagnitude = System.Math.Clamp(
                frictionImpulseMagnitude,
                -maxFriction,
                maxFriction);

            Vector2 frictionImpulse = tangent * frictionImpulseMagnitude;

            ApplyImpulse(bodyA, bodyB, rA, rB, frictionImpulse);
        }

        private static void ApplyImpulse(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, Vector2 impulse)
        {
            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyA.AngularVelocity -= rA.Cross(impulse) * bodyA.InvInertia;

            bodyB.LinearVelocity += impulse * bodyB.InvMass;
            bodyB.AngularVelocity += rB.Cross(impulse) * bodyB.InvInertia;
        }

        private static void ApplyPositionalCorrection(RigidBody bodyA, RigidBody bodyB, Vector2 normal, float percent, float slop, float depth, float invMassSum)
        {
            float correctionDepth = MathF.Max(depth - slop, 0.0f);

            if (correctionDepth <= 0)
                return;

            Vector2 correction = normal * (correctionDepth / invMassSum) * percent;

            bodyA.Position -= correction * bodyA.InvMass;
            bodyB.Position += correction * bodyB.InvMass;
        }

        private static float CalculateEffectiveInvMass(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, Vector2 direction, float invMassSum)
        {
            float raCross = rA.Cross(direction);
            float rbCross = rB.Cross(direction);

            return invMassSum +
                   (raCross * raCross) * bodyA.InvInertia +
                   (rbCross * rbCross) * bodyB.InvInertia;
        }
    }
}