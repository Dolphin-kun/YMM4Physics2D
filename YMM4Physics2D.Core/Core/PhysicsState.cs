using System.Numerics;
using YMM4Physics2D.Core.Bodies;

namespace YMM4Physics2D.Core.Core
{
    public struct BodyState(RigidBody body)
    {
        public Vector2 Position = body.Position;
        public float Rotation = body.Rotation;
        public Vector2 LinearVelocity = body.LinearVelocity;
        public float AngularVelocity = body.AngularVelocity;
        public Vector2 Force = body.Force;
        public float Torque = body.Torque;
        public bool IsActive = body.IsActive;

        public readonly void ApplyTo(RigidBody body)
        {
            body.Position = Position;
            body.Rotation = Rotation;
            body.LinearVelocity = LinearVelocity;
            body.AngularVelocity = AngularVelocity;
            body.Force = Force;
            body.Torque = Torque;
            body.IsActive = IsActive;
        }

        public class WorldState
        {
            public int Frame;
            public BodyState[] BodyStates;

            public WorldState(int frame, List<RigidBody> bodies)
            {
                Frame = frame;
                BodyStates = new BodyState[bodies.Count];
                for (int i = 0; i < bodies.Count; i++)
                {
                    BodyStates[i] = new BodyState(bodies[i]);
                }
            }
        }
    }
}
