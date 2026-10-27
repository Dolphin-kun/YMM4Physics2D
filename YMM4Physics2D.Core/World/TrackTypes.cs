using System.ComponentModel.DataAnnotations;
using System.Numerics;
using System.Runtime.CompilerServices;
using YMM4Physics2D.Core.Bodies;
using YMM4Physics2D.Core.Shapes;

namespace YMM4Physics2D.Core.World
{
    public enum ShapeUpdateMode
    {
        [Display(Name = "初回のみ", Description = "最初に取得した画像から当たり判定を作ります\n軽量です")]
        Once,

        [Display(Name = "毎フレーム", Description = "毎フレーム画像を読み取り、形が変わったら当たり判定を作り直します\nアイテム数が多いと重くなります")]
        EveryFrame,
    }

    public readonly struct TrackKey(object owner, int groupIndex, int inputIndex) : IEquatable<TrackKey>
    {
        public object Owner { get; } = owner;
        public int GroupIndex { get; } = groupIndex;
        public int InputIndex { get; } = inputIndex;

        public bool Equals(TrackKey other) => ReferenceEquals(Owner, other.Owner) && GroupIndex == other.GroupIndex && InputIndex == other.InputIndex;
        public override bool Equals(object? obj) => obj is TrackKey other && Equals(other);
        public override int GetHashCode() => HashCode.Combine(RuntimeHelpers.GetHashCode(Owner), GroupIndex, InputIndex);
    }

    public readonly record struct TrackObservation(Vector2 Draw, float Rotation, Vector2 Zoom, bool Invert, Vector2 CenterPoint)
    {
        public Matrix3x2 ImageTransform =>
            (Invert ? Matrix3x2.CreateScale(-1f, 1f, CenterPoint) : Matrix3x2.Identity) * Matrix3x2.CreateScale(Zoom);

        public bool NearlyEquals(in TrackObservation other)
        {
            const float Epsilon = 1e-3f;
            return Invert == other.Invert &&
                   Vector2.DistanceSquared(Draw, other.Draw) < Epsilon * Epsilon &&
                   MathF.Abs(Rotation - other.Rotation) < 1e-5f &&
                   Vector2.DistanceSquared(Zoom, other.Zoom) < 1e-10f &&
                   (!Invert || Vector2.DistanceSquared(CenterPoint, other.CenterPoint) < Epsilon * Epsilon);
        }

        public static TrackObservation Lerp(in TrackObservation a, in TrackObservation b, float t)
        {
            return new TrackObservation(
                Vector2.Lerp(a.Draw, b.Draw, t),
                a.Rotation + (b.Rotation - a.Rotation) * t,
                Vector2.Lerp(a.Zoom, b.Zoom, t),
                a.Invert,
                a.CenterPoint);
        }
    }

    public readonly record struct BodyParameters(float Mass, float Restitution, float Friction, float LinearDamping, float AngularDamping, float Attraction = 0f, float AttractionRange = 0f, float FluidViscosity = 0.3f, float FluidCohesion = 0.5f, float FluidDrying = 0.5f);

    public delegate BodyParameters BodyParameterProvider(int localFrame, int length, int fps);

    public readonly record struct TrackSettings(BodyType BodyType, bool IsSeparate, string Name, AreaDefinition? Area = null, int FluidResolution = 0, float CollisionMargin = 0f);

    public readonly record struct PartPose(Vector2 Position, float Rotation, float Opacity = 1f);

    public sealed class TrackUpdate
    {
        public int StartFrame { get; init; }
        public int Length { get; init; }
        public int Layer { get; init; }
        public int LocalFrame { get; init; }
        public BodyType BodyType { get; init; }
        public bool IsSeparate { get; init; }
        public float CollisionMargin { get; init; }
        public int FluidResolution { get; init; }
        public ShapeUpdateMode ShapeUpdateMode { get; init; }
        public TrackObservation Observation { get; init; }
        public BodyParameters Parameters { get; init; }
        public required BodyParameterProvider ParameterProvider { get; init; }
        public Func<TrackSettings>? SettingsProvider { get; init; }

        public string Name { get; init; } = string.Empty;

        public AreaDefinition? Area { get; init; }
        public AreaBounds AreaBounds { get; init; }
        public AreaParameters AreaParameters { get; init; }
        public AreaParameterProvider? AreaParameterProvider { get; init; }

        public ShapeVersion? Shape { get; init; }

        public bool ShapeReplaced { get; init; }
    }

    public sealed class TrackFrameResult
    {
        public ShapeVersion Shape { get; set; } = ShapeVersion.Empty;
        public Matrix3x2 ImageTransform { get; set; } = Matrix3x2.Identity;
        public bool IsSeparate { get; set; }
        public List<PartPose> Poses { get; } = [];
        public List<Vector2> LocalCentroids { get; } = [];
        public List<JointLine> JointLines { get; } = [];
        public FluidShape? FluidShape { get; set; }
        public List<Vector2> FluidPositions { get; } = [];
        public List<Vector2> FluidVelocities { get; } = [];
        public List<float> FluidPresences { get; } = [];
    }
}
