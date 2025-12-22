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
            public Dictionary<int, BodyState> BodyStates;

            public WorldState(int frame, List<RigidBody> bodies)
            {
                Frame = frame;
                BodyStates = new Dictionary<int, BodyState>(bodies.Count);

                foreach (var body in bodies)
                {
                    BodyStates[body.Id] = new BodyState(body);
                }
            }
        }
    }
}
