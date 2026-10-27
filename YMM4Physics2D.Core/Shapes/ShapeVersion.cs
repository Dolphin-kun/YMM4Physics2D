using System.Numerics;

namespace YMM4Physics2D.Core.Shapes
{
    public sealed class ShapeComponent(Vector2[][] convexParts, Vector2[][] outline, float area, Vector2 centroid)
    {
        public Vector2[][] ConvexParts { get; } = convexParts;

        public Vector2[][] Outline { get; } = outline;

        public float Area { get; } = area;

        public Vector2 Centroid { get; } = centroid;
    }

    public sealed class ShapeVersion(ulong hash, ShapeComponent[] components)
    {
        public static ShapeVersion Empty { get; } = new(0, []);

        public ulong Hash { get; } = hash;

        public ShapeComponent[] Components { get; } = components;

        public bool IsEmpty => Components.Length == 0;
    }
}
