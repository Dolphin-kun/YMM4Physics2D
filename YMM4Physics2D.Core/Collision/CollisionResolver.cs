using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Geometry;

namespace YMM4Physics2D.Core.Collision
{
    public static class CollisionResolver
    {
        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public static void Resolve(in Manifold m, float depth, float minVelocity, float correctionPercent, float slop)
        {
            RigidBody bodyA = m.BodyA;
            RigidBody bodyB = m.BodyB;

            float invMassSum = bodyA.InvMass + bodyB.InvMass;
            if (invMassSum == 0f) return;

            Vector2 normal = m.Normal;
            int count = m.ContactCount;

            Span<Vector2> rAs = stackalloc Vector2[count];
            Span<Vector2> rBs = stackalloc Vector2[count];
            Span<Vector2> impulses = stackalloc Vector2[count];
            Span<float> normalImpulses = stackalloc float[count];

            bool separating = true;
            float share = 1f / count;

            for (int i = 0; i < count; i++)
            {
                Vector2 contact = i == 0 ? m.Contact1 : m.Contact2;
                rAs[i] = contact - bodyA.Position;
                rBs[i] = contact - bodyB.Position;

                Vector2 relativeVelocity = CalculateRelativeVelocity(bodyA, bodyB, rAs[i], rBs[i]);
                float velocityAlongNormal = Vector2.Dot(relativeVelocity, normal);

                if (velocityAlongNormal > 0) continue;
                separating = false;

                normalImpulses[i] = CalculateNormalImpulse(bodyA, bodyB, rAs[i], rBs[i], normal, minVelocity, velocityAlongNormal, invMassSum) * share;
                impulses[i] = normal * normalImpulses[i];
            }

            if (separating) return;

            for (int i = 0; i < count; i++)
            {
                ApplyImpulse(bodyA, bodyB, rAs[i], rBs[i], impulses[i]);
            }

            for (int i = 0; i < count; i++)
            {
                impulses[i] = normalImpulses[i] != 0f
                    ? CalculateFrictionImpulse(bodyA, bodyB, rAs[i], rBs[i], normal, normalImpulses[i], invMassSum, share)
                    : Vector2.Zero;
            }

            for (int i = 0; i < count; i++)
            {
                ApplyImpulse(bodyA, bodyB, rAs[i], rBs[i], impulses[i]);
            }

            ApplyPositionalCorrection(bodyA, bodyB, normal, correctionPercent, slop, depth, invMassSum);
        }

        private static Vector2 CalculateRelativeVelocity(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB)
        {
            Vector2 velocityA = bodyA.LinearVelocity + Vector2Extensions.Cross(bodyA.AngularVelocity, rA);
            Vector2 velocityB = bodyB.LinearVelocity + Vector2Extensions.Cross(bodyB.AngularVelocity, rB);
            return velocityB - velocityA;
        }

        private static float CalculateNormalImpulse(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, Vector2 normal, float minVelocity, float velocityAlongNormal, float invMassSum)
        {
            float restitution = MathF.Min(bodyA.Restitution, bodyB.Restitution);
            if (MathF.Abs(velocityAlongNormal) < minVelocity)
            {
                restitution = 0.0f;
            }

            float effectiveInvMass = CalculateEffectiveInvMass(bodyA, bodyB, rA, rB, normal, invMassSum);
            return -(1.0f + restitution) * velocityAlongNormal / effectiveInvMass;
        }

        private static Vector2 CalculateFrictionImpulse(RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, Vector2 normal, float normalImpulseMagnitude, float invMassSum, float share)
        {
            Vector2 relativeVelocity = CalculateRelativeVelocity(bodyA, bodyB, rA, rB);
            Vector2 tangent = relativeVelocity - (normal * Vector2.Dot(relativeVelocity, normal));

            if (tangent.LengthSquared() < 0.0001f)
                return Vector2.Zero;

            tangent = Vector2.Normalize(tangent);
            float velocityAlongTangent = Vector2.Dot(relativeVelocity, tangent);
            float effectiveInvMass = CalculateEffectiveInvMass(bodyA, bodyB, rA, rB, tangent, invMassSum);
            float frictionImpulseMagnitude = -velocityAlongTangent / effectiveInvMass * share;

            float mu = MathF.Sqrt(bodyA.Friction * bodyB.Friction);
            float maxFriction = MathF.Abs(normalImpulseMagnitude) * mu;
            frictionImpulseMagnitude = Math.Clamp(
                frictionImpulseMagnitude,
                -maxFriction,
                maxFriction);

            return tangent * frictionImpulseMagnitude;
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
