using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Colliders;
using YMM4Physics2D.Core.Shapes;

namespace YMM4Physics2D.Core.World
{
    public sealed partial class PhysicsWorld
    {
        private const int DefaultFluidResolution = 120;
        private const int FluidIterations = 4;
        private const float FluidKernelScale = 2.1f;
        private const float FluidRelaxation = 0.05f;
        private const float FluidMaxCorrection = 0.15f;
        private const float FluidMaxSpeed = 0.5f;
        private const float FluidCorrectionScale = 2e-5f;
        private const float FluidCohesionStrength = 6000f;
        private const float FluidVelocityTransfer = 1f;
        private const float FluidDrag = 3f;
        private const float FluidCorrectionDistance = 0.25f;
        private const float FluidWetCohesion = 0.1f;
        private const int FluidThinNeighbors = 7;
        private const float FluidDryingSeconds = 1f;

        private readonly List<RigidBody> fluidBodies = [];
        private readonly List<BodyTrack> fluidOwners = [];
        private readonly Dictionary<long, int> fluidCellStart = [];
        private const int MaxRestDensityCache = 64;

        private readonly Dictionary<(float Spacing, float Kernel), float> fluidRestDensity = [];
        private Vector2[] fluidPositions = [];
        private Vector2[] fluidPredicted = [];
        private Vector2[] fluidStart = [];
        private Vector2[] fluidDelta = [];
        private float[] fluidLambda = [];
        private float[] fluidKernel = [];
        private float[] fluidPoly6Scale = [];
        private float[] fluidSpikyScale = [];
        private float[] fluidDensity = [];
        private float[] fluidViscosity = [];
        private float[] fluidCohesion = [];
        private float[] fluidDrying = [];
        private float[] fluidWeights = [];
        private int[] fluidOrder = [];
        private long[] fluidCellKeys = [];
        private int[] fluidNeighbors = [];
        private int[] fluidNeighborStart = [];
        private int fluidNeighborTotal;
        private float fluidGridSize;
        private int fluidGridCount;

        private static FluidShape? GetFluidShape(BodyTrack track, ShapeVersion shape, Matrix3x2 imageTransform)
        {
            int resolution = track.FluidResolution > 0 ? track.FluidResolution : DefaultFluidResolution;
            var key = (shape, imageTransform, resolution);
            if (!track.FluidShapeCache.TryGetValue(key, out var fluid))
            {
                if (track.FluidShapeCache.Count > 16) track.FluidShapeCache.Clear();
                fluid = FluidSampler.Build(shape, imageTransform, resolution);
                track.FluidShapeCache[key] = fluid;
            }
            return fluid;
        }

        private static List<RigidBody> CreateParticles(FluidShape fluid, Matrix3x2 imageTransform, float mass)
        {
            Matrix3x2.Invert(imageTransform, out var toImage);
            float particleMass = mass / Math.Max(fluid.Count, 1);

            var bodies = new List<RigidBody>(fluid.Count);
            for (int i = 0; i < fluid.Count; i++)
            {
                var body = new RigidBody(Vector2.Zero, particleMass, BodyType.Dynamic)
                {
                    IsParticle = true,
                    LocalCentroid = fluid.RestLocal[i],
                    ImageCentroid = Vector2.Transform(fluid.RestLocal[i], toImage),
                };
                body.AddColliders([new CircleCollider(fluid.Radius)]);
                bodies.Add(body);
            }
            return bodies;
        }

        private List<RigidBody> CreateFluidBodies(BodyTrack track, ShapeVersion shape, Matrix3x2 imageTransform, float mass)
        {
            var fluid = GetFluidShape(track, shape, imageTransform);
            track.FluidShape = fluid;
            return fluid == null ? [] : CreateParticles(fluid, imageTransform, mass);
        }

        private void ReshapeFluid(BodyTrack track, ShapeVersion shape, Matrix3x2 imageTransform, BodyParameters bodyParameters)
        {
            Vector2 center = Vector2.Zero;
            Vector2 velocity = Vector2.Zero;
            foreach (var body in track.Bodies)
            {
                center += body.Position;
                velocity += body.LinearVelocity;
            }
            if (track.Bodies.Count > 0)
            {
                center /= track.Bodies.Count;
                velocity /= track.Bodies.Count;
            }

            var bodies = CreateFluidBodies(track, shape, imageTransform, bodyParameters.Mass);
            if (track.FluidShape is { } fluid && bodies.Count > 0)
            {
                Vector2 restCenter = Vector2.Zero;
                foreach (var point in fluid.RestLocal) restCenter += point;
                restCenter /= fluid.Count;

                foreach (var body in bodies)
                {
                    body.Position = center + body.LocalCentroid - restCenter;
                    body.LinearVelocity = velocity;
                    body.UpdateWorldAABB();
                }
            }

            track.Bodies = bodies;
            track.SpawnedShape = shape;
            track.SpawnedTransform = imageTransform;
            bodiesDirty = true;
        }

        private void GatherFluid()
        {
            fluidBodies.Clear();
            fluidOwners.Clear();

            foreach (var track in tracks)
            {
                if (!track.IsSpawned || !track.IsFluid || track.FluidShape is not { } fluid) continue;
                if (track.Bodies.Count != fluid.Count) continue;

                foreach (var body in track.Bodies)
                {
                    if (body.IsSpawning || body.Presence <= 0f || (body.IsGhost && body.Opacity <= 0f)) continue;
                    fluidBodies.Add(body);
                    fluidOwners.Add(track);
                }
            }

            int count = fluidBodies.Count;
            if (count == 0) return;

            if (fluidPositions.Length < count)
            {
                fluidPositions = new Vector2[count];
                fluidPredicted = new Vector2[count];
                fluidStart = new Vector2[count];
                fluidDelta = new Vector2[count];
                fluidLambda = new float[count];
                fluidKernel = new float[count];
                fluidPoly6Scale = new float[count];
                fluidSpikyScale = new float[count];
                fluidDensity = new float[count];
                fluidViscosity = new float[count];
                fluidCohesion = new float[count];
                fluidDrying = new float[count];
                fluidWeights = new float[count];
                fluidOrder = new int[count];
                fluidCellKeys = new long[count];
            }

            for (int i = 0; i < count; i++)
            {
                var track = fluidOwners[i];
                var fluid = track.FluidShape!;
                var bodyParameters = track.CurrentParameters;

                fluidStart[i] = fluidBodies[i].Position;
                fluidKernel[i] = fluid.Spacing * FluidKernelScale;
                fluidPoly6Scale[i] = Poly6Scale(fluidKernel[i]);
                fluidSpikyScale[i] = SpikyScale(fluidKernel[i]);
                fluidDensity[i] = RestDensity(fluid.Spacing, fluidKernel[i]);
                fluidViscosity[i] = Math.Clamp(bodyParameters.FluidViscosity, 0f, 1f);
                fluidCohesion[i] = Math.Clamp(bodyParameters.FluidCohesion, 0f, 1f) * (fluidBodies[i].TouchingSolid ? FluidWetCohesion : 1f);
                fluidDrying[i] = Math.Clamp(bodyParameters.FluidDrying, 0f, 1f);
                fluidWeights[i] = fluidBodies[i].InvMass > 0f ? 1f : 0f;
            }
        }

        private float RestDensity(float spacing, float kernel)
        {
            if (fluidRestDensity.TryGetValue((spacing, kernel), out float cached)) return cached;

            float rowHeight = spacing * 0.8660254f;
            int range = (int)MathF.Ceiling(kernel / MathF.Min(spacing, rowHeight)) + 1;
            float density = 0f;
            float scale = Poly6Scale(kernel);
            for (int row = -range; row <= range; row++)
            {
                for (int column = -range; column <= range; column++)
                {
                    var offset = new Vector2(column * spacing + (row & 1) * spacing * 0.5f, row * rowHeight);
                    density += Poly6(offset.Length(), kernel, scale);
                }
            }

            if (fluidRestDensity.Count >= MaxRestDensityCache) fluidRestDensity.Clear();
            fluidRestDensity[(spacing, kernel)] = density;
            return density;
        }

        private static float Poly6Scale(float kernel) => 4f / (MathF.PI * MathF.Pow(kernel, 8f));

        private static float SpikyScale(float kernel) => -30f / (MathF.PI * MathF.Pow(kernel, 5f));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float Poly6(float distance, float kernel, float scale)
        {
            if (distance >= kernel) return 0f;
            float difference = kernel * kernel - distance * distance;
            return scale * difference * difference * difference;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Vector2 SpikyGradient(Vector2 offset, float distance, float kernel, float scale)
        {
            if (distance <= 1e-5f || distance >= kernel) return Vector2.Zero;
            float difference = kernel - distance;
            return offset * (scale * difference * difference / distance);
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private void SolveFluids(float deltaTime)
        {
            int count = fluidBodies.Count;
            if (count == 0) return;

            for (int i = 0; i < count; i++)
            {
                fluidPositions[i] = fluidBodies[i].Position;
                fluidPredicted[i] = fluidBodies[i].Position;
            }

            float cellSize = 0f;
            for (int i = 0; i < count; i++) cellSize = MathF.Max(cellSize, fluidKernel[i]);
            BuildFluidGrid(count, cellSize);
            BuildFluidNeighbors(count);
            UpdateFluidPresence(count, deltaTime);

            for (int iteration = 0; iteration < FluidIterations; iteration++)
            {
                for (int i = 0; i < count; i++)
                {
                    if (fluidWeights[i] <= 0f)
                    {
                        fluidLambda[i] = 0f;
                        continue;
                    }

                    float kernel = fluidKernel[i];
                    float poly6Scale = fluidPoly6Scale[i];
                    float spikyScale = fluidSpikyScale[i];
                    float density = Poly6(0f, kernel, poly6Scale);
                    float gradientSum = 0f;
                    Vector2 selfGradient = Vector2.Zero;

                    int start = fluidNeighborStart[i], end = fluidNeighborStart[i + 1];
                    for (int n = start; n < end; n++)
                    {
                        int j = fluidNeighbors[n];
                        Vector2 offset = fluidPositions[i] - fluidPositions[j];
                        float distance = offset.Length();
                        if (distance >= kernel) continue;

                        density += Poly6(distance, kernel, poly6Scale);
                        Vector2 gradient = SpikyGradient(offset, distance, kernel, spikyScale) / fluidDensity[i];
                        selfGradient += gradient;
                        gradientSum += gradient.LengthSquared();
                    }

                    float error = density / fluidDensity[i] - 1f;
                    gradientSum += selfGradient.LengthSquared();
                    float relaxation = FluidRelaxation / (kernel * kernel);
                    fluidLambda[i] = error > 0f ? -error / (gradientSum + relaxation) : 0f;
                }

                for (int i = 0; i < count; i++)
                {
                    fluidDelta[i] = Vector2.Zero;
                    if (fluidWeights[i] <= 0f) continue;

                    float kernel = fluidKernel[i];
                    float correctionDistance = FluidCorrectionDistance * kernel;
                    float poly6Scale = fluidPoly6Scale[i];
                    float spikyScale = fluidSpikyScale[i];
                    float correctionBase = Poly6(correctionDistance, kernel, poly6Scale);
                    float strength = FluidCorrectionScale * kernel * kernel;

                    Vector2 delta = Vector2.Zero;
                    int start = fluidNeighborStart[i], end = fluidNeighborStart[i + 1];
                    for (int n = start; n < end; n++)
                    {
                        int j = fluidNeighbors[n];
                        Vector2 offset = fluidPositions[i] - fluidPositions[j];
                        float distance = offset.Length();
                        if (distance >= kernel) continue;

                        float correction = 0f;
                        if (strength > 0f && correctionBase > 0f)
                        {
                            float ratio = Poly6(distance, kernel, poly6Scale) / correctionBase;
                            correction = -strength * ratio * ratio * ratio * ratio;
                        }

                        delta += SpikyGradient(offset, distance, kernel, spikyScale) * (fluidLambda[i] + fluidLambda[j] + correction);
                    }

                    delta /= fluidDensity[i];
                    float limit = FluidMaxCorrection * fluidKernel[i] / FluidKernelScale;
                    float length = delta.Length();
                    if (length > limit) delta *= limit / length;
                    fluidDelta[i] = delta;
                }

                for (int i = 0; i < count; i++) fluidPositions[i] += fluidDelta[i];
            }

            float inverseDelta = 1f / deltaTime;
            for (int i = 0; i < count; i++)
            {
                if (fluidWeights[i] <= 0f) continue;

                var body = fluidBodies[i];
                body.Position = fluidPositions[i];

                Vector2 motion = fluidPredicted[i] - fluidStart[i] + (fluidPositions[i] - fluidPredicted[i]) * FluidVelocityTransfer;
                Vector2 velocity = motion * inverseDelta;
                float maxSpeed = FluidMaxSpeed * fluidKernel[i] / FluidKernelScale * inverseDelta;
                float speed = velocity.Length();
                body.LinearVelocity = speed > maxSpeed ? velocity * (maxSpeed / speed) : velocity;
            }

            ApplyFluidCohesion(count, deltaTime);
            ApplyFluidViscosity(count, deltaTime);

            for (int i = 0; i < count; i++) fluidBodies[i].UpdateWorldAABB();
        }

        private void UpdateFluidPresence(int count, float deltaTime)
        {
            for (int i = 0; i < count; i++)
            {
                float drying = fluidDrying[i];
                if (drying <= 0f || fluidWeights[i] <= 0f) continue;

                var body = fluidBodies[i];
                int neighbors = fluidNeighborStart[i + 1] - fluidNeighborStart[i];
                if (body.TouchingSolid && neighbors <= FluidThinNeighbors)
                {
                    body.Presence = MathF.Max(0f, body.Presence - deltaTime * drying / FluidDryingSeconds);
                }
                else if (neighbors > FluidThinNeighbors + 1 && body.Presence < 1f)
                {
                    body.Presence = MathF.Min(1f, body.Presence + deltaTime / FluidDryingSeconds);
                }
            }
        }

        private void ApplyFluidCohesion(int count, float deltaTime)
        {
            for (int i = 0; i < count; i++)
            {
                fluidDelta[i] = Vector2.Zero;
                if (fluidWeights[i] <= 0f || fluidCohesion[i] <= 0f) continue;

                float kernel = fluidKernel[i];
                Vector2 pull = Vector2.Zero;

                int start = fluidNeighborStart[i], end = fluidNeighborStart[i + 1];
                for (int n = start; n < end; n++)
                {
                    int j = fluidNeighbors[n];
                    Vector2 offset = fluidPositions[j] - fluidPositions[i];
                    float distance = offset.Length();
                    if (distance < 1e-4f || distance >= kernel) continue;

                    float falloff = 1f - distance / kernel;
                    pull += offset * (falloff * falloff / distance);
                }

                float spacing = kernel / FluidKernelScale;
                fluidDelta[i] = pull * (fluidCohesion[i] * FluidCohesionStrength / spacing * deltaTime);
            }

            for (int i = 0; i < count; i++)
            {
                if (fluidWeights[i] <= 0f) continue;
                fluidBodies[i].LinearVelocity += fluidDelta[i];
            }
        }

        private void ApplyFluidViscosity(int count, float deltaTime)
        {
            for (int i = 0; i < count; i++)
            {
                if (fluidWeights[i] <= 0f || fluidViscosity[i] <= 0f) continue;

                float kernel = fluidKernel[i];
                Vector2 velocity = fluidBodies[i].LinearVelocity;
                Vector2 average = Vector2.Zero;
                float weight = 0f;

                int start = fluidNeighborStart[i], end = fluidNeighborStart[i + 1];
                for (int n = start; n < end; n++)
                {
                    int j = fluidNeighbors[n];
                    float distance = Vector2.Distance(fluidPositions[i], fluidPositions[j]);
                    if (distance >= kernel) continue;

                    float influence = Poly6(distance, kernel, fluidPoly6Scale[i]);
                    average += fluidBodies[j].LinearVelocity * influence;
                    weight += influence;
                }

                if (weight <= 0f) continue;
                fluidDelta[i] = (average / weight - velocity) * fluidViscosity[i];
            }

            for (int i = 0; i < count; i++)
            {
                if (fluidWeights[i] <= 0f || fluidViscosity[i] <= 0f) continue;
                fluidBodies[i].LinearVelocity += fluidDelta[i];
            }

            for (int i = 0; i < count; i++)
            {
                if (fluidWeights[i] <= 0f || fluidViscosity[i] <= 0f) continue;
                fluidBodies[i].LinearVelocity *= MathF.Exp(-FluidDrag * fluidViscosity[i] * deltaTime);
            }
        }

        private void BuildFluidGrid(int count, float cellSize)
        {
            fluidCellStart.Clear();
            float inverse = 1f / MathF.Max(cellSize, 1e-3f);

            for (int i = 0; i < count; i++)
            {
                int x = (int)MathF.Floor(fluidPositions[i].X * inverse);
                int y = (int)MathF.Floor(fluidPositions[i].Y * inverse);
                fluidOrder[i] = i;
                fluidCellKeys[i] = CellKey(x, y);
            }

            Array.Sort(fluidCellKeys, fluidOrder, 0, count);

            for (int i = 0; i < count; i++)
            {
                if (i == 0 || fluidCellKeys[i] != fluidCellKeys[i - 1]) fluidCellStart[fluidCellKeys[i]] = i;
            }

            fluidGridSize = inverse;
            fluidGridCount = count;
        }

        private static long CellKey(int x, int y) => ((long)x << 32) ^ (uint)y;

        private void BuildFluidNeighbors(int count)
        {
            if (fluidNeighborStart.Length < count + 1) fluidNeighborStart = new int[count + 1];
            if (fluidNeighbors.Length < count * 16) fluidNeighbors = new int[count * 16];

            fluidNeighborTotal = 0;
            for (int i = 0; i < count; i++)
            {
                fluidNeighborStart[i] = fluidNeighborTotal;

                float kernel = fluidKernel[i];
                float kernelSquared = kernel * kernel;
                int x = (int)MathF.Floor(fluidPositions[i].X * fluidGridSize);
                int y = (int)MathF.Floor(fluidPositions[i].Y * fluidGridSize);

                for (int offsetY = -1; offsetY <= 1; offsetY++)
                {
                    for (int offsetX = -1; offsetX <= 1; offsetX++)
                    {
                        long key = CellKey(x + offsetX, y + offsetY);
                        if (!fluidCellStart.TryGetValue(key, out int start)) continue;

                        for (int index = start; index < fluidGridCount && fluidCellKeys[index] == key; index++)
                        {
                            int candidate = fluidOrder[index];
                            if (candidate == i) continue;
                            if (Vector2.DistanceSquared(fluidPositions[i], fluidPositions[candidate]) >= kernelSquared) continue;

                            if (fluidNeighborTotal >= fluidNeighbors.Length) Array.Resize(ref fluidNeighbors, fluidNeighbors.Length * 2);
                            fluidNeighbors[fluidNeighborTotal++] = candidate;
                        }
                    }
                }
            }
            fluidNeighborStart[count] = fluidNeighborTotal;
        }

        private void RecordFluid(BodyTrack track, ref BodyTrack.FrameRecord record)
        {
            var fluid = track.FluidShape;
            var bodies = track.Bodies;
            if (fluid == null || bodies.Count != fluid.Count)
            {
                record.Poses = [];
                record.Centroids = [];
                record.FluidShape = null;
                record.FluidPositions = null;
                record.FluidVelocities = null;
                record.FluidPresences = null;
                return;
            }

            Vector2 center = Vector2.Zero;
            Vector2 restCenter = Vector2.Zero;
            float opacity = 0f;
            int present = 0;
            var positions = new Vector2[bodies.Count];
            var velocities = new Vector2[bodies.Count];
            var presences = new float[bodies.Count];
            for (int i = 0; i < bodies.Count; i++)
            {
                positions[i] = bodies[i].Position;
                velocities[i] = bodies[i].LinearVelocity;
                presences[i] = bodies[i].Presence;
                center += bodies[i].Position;
                restCenter += fluid.RestLocal[i];
                if (bodies[i].Presence > 0f)
                {
                    opacity += bodies[i].Opacity;
                    present++;
                }
            }
            center /= bodies.Count;
            restCenter /= bodies.Count;
            opacity = present > 0 ? opacity / present : 0f;

            if (record.Poses == null || record.Poses.Length != 1)
            {
                record.Poses = new PartPose[1];
                record.Centroids = new Vector2[1];
            }
            record.Poses[0] = new PartPose(center, 0f, opacity);
            record.Centroids[0] = restCenter;
            record.FluidShape = fluid;
            record.FluidPositions = positions;
            record.FluidVelocities = velocities;
            record.FluidPresences = presences;
        }

        private bool FireFluidTrigger(ActiveArea area, BodyTrack track)
        {
            long id = area.Track.Sequence;
            bool entered = false;
            bool touching = false;
            foreach (var body in track.Bodies)
            {
                if (body.IsSpawning || body.IsGhost) continue;
                body.UpdateWorldAABB();
                if (!Overlaps(area, body.WorldAABB)) continue;
                touching = true;
                if (!body.InsideTriggers.Contains(id)) entered = true;
            }

            if (!touching)
            {
                foreach (var body in track.Bodies) body.InsideTriggers.Remove(id);
                return false;
            }

            foreach (var body in track.Bodies)
            {
                if (!body.InsideTriggers.Contains(id)) body.InsideTriggers.Add(id);
            }
            if (!entered) return false;

            if (area.Definition.Action == TriggerAction.Warp)
            {
                Vector2 center = Vector2.Zero;
                foreach (var body in track.Bodies) center += body.Position;
                center /= track.Bodies.Count;
                Vector2 offset = area.Center + area.Parameters.WarpOffset - center;
                foreach (var body in track.Bodies)
                {
                    body.Position += offset;
                    if (!area.Definition.KeepVelocity) body.LinearVelocity = Vector2.Zero;
                    body.UpdateWorldAABB();
                }
            }
            else
            {
                foreach (var body in track.Bodies) Fire(area, body);
            }
            return true;
        }
    }
}
