using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using Vortice.DCommon;
using Vortice.DXGI;
using Vortice.Direct2D1;
using Vortice.Mathematics;

namespace YMM4Physics2D.Core.Shapes
{
    public sealed class ShapeReader : IDisposable
    {
        public const int MaxAnalysisSize = 1024;

        private ID2D1Bitmap1? targetBitmap;
        private ID2D1Bitmap1? stagingBitmap;
        private SizeI bitmapSize;
        private byte[] mask = [];

        public byte[] Mask => mask;
        public int Width { get; private set; }
        public int Height { get; private set; }
        public Vector2 Origin { get; private set; }
        public float PixelSize { get; private set; }
        public ulong Hash { get; private set; }

        public unsafe bool Read(ID2D1DeviceContext context, ID2D1Image input, byte alphaThreshold)
        {
            var bounds = context.GetImageLocalBounds(input);
            if (!float.IsFinite(bounds.Left) || !float.IsFinite(bounds.Top) || !float.IsFinite(bounds.Right) || !float.IsFinite(bounds.Bottom)) return false;

            float left = MathF.Floor(bounds.Left);
            float top = MathF.Floor(bounds.Top);
            float imageWidth = MathF.Ceiling(bounds.Right) - left;
            float imageHeight = MathF.Ceiling(bounds.Bottom) - top;
            if (imageWidth < 1f || imageHeight < 1f) return false;

            float scale = MathF.Min(1f, MaxAnalysisSize / MathF.Max(imageWidth, imageHeight));
            int width = Math.Max(1, (int)MathF.Ceiling(imageWidth * scale));
            int height = Math.Max(1, (int)MathF.Ceiling(imageHeight * scale));

            EnsureBitmaps(context, new SizeI(width, height));

            using (var oldTarget = context.Target)
            {
                var oldTransform = context.Transform;
                context.Target = targetBitmap;
                context.Transform = Matrix3x2.CreateTranslation(-left, -top) * Matrix3x2.CreateScale(scale);
                try
                {
                    context.BeginDraw();
                    try
                    {
                        context.Clear(new Color4(0, 0, 0, 0));
                        context.DrawImage(input);
                    }
                    finally
                    {
                        context.EndDraw();
                    }
                }
                finally
                {
                    context.Target = oldTarget;
                    context.Transform = oldTransform;
                }
            }

            stagingBitmap!.CopyFromBitmap(targetBitmap!);

            if (mask.Length < width * height) mask = new byte[width * height];

            var map = stagingBitmap.Map(MapOptions.Read);
            try
            {
                for (int y = 0; y < height; y++)
                {
                    var row = new ReadOnlySpan<byte>((void*)(map.Bits + y * map.Pitch), width);
                    Threshold(row, mask.AsSpan(y * width, width), alphaThreshold);
                }
            }
            finally
            {
                stagingBitmap.Unmap();
            }

            Width = width;
            Height = height;
            Origin = new Vector2(left, top);
            PixelSize = 1f / scale;
            Hash = ComputeHash(mask.AsSpan(0, width * height), width, height, Origin, PixelSize);
            return true;
        }

        private void EnsureBitmaps(ID2D1DeviceContext context, SizeI size)
        {
            if (targetBitmap != null && bitmapSize.Width == size.Width && bitmapSize.Height == size.Height) return;

            targetBitmap?.Dispose();
            stagingBitmap?.Dispose();

            var format = new PixelFormat(Format.A8_UNorm, Vortice.DCommon.AlphaMode.Premultiplied);
            targetBitmap = context.CreateBitmap(size, new BitmapProperties1(format, 96f, 96f, BitmapOptions.Target));
            stagingBitmap = context.CreateBitmap(size, new BitmapProperties1(format, 96f, 96f, BitmapOptions.CpuRead | BitmapOptions.CannotDraw));
            bitmapSize = size;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static void Threshold(ReadOnlySpan<byte> source, Span<byte> destination, byte alphaThreshold)
        {
            int i = 0;
            if (Vector256.IsHardwareAccelerated)
            {
                var threshold = Vector256.Create(alphaThreshold);
                var one = Vector256.Create((byte)1);
                ref byte sourceRef = ref MemoryMarshal.GetReference(source);
                ref byte destinationRef = ref MemoryMarshal.GetReference(destination);
                for (; i + 32 <= source.Length; i += 32)
                {
                    var values = Vector256.LoadUnsafe(ref sourceRef, (nuint)i);
                    (Vector256.GreaterThan(values, threshold) & one).StoreUnsafe(ref destinationRef, (nuint)i);
                }
            }
            for (; i < source.Length; i++)
            {
                destination[i] = (byte)(source[i] > alphaThreshold ? 1 : 0);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        private static ulong ComputeHash(ReadOnlySpan<byte> data, int width, int height, Vector2 origin, float pixelSize)
        {
            const ulong Prime = 0x9E3779B185EBCA87UL;

            ulong hash = (ulong)width * 0x100000001B3UL ^ (ulong)height;
            hash = Mix(hash, BitConverter.SingleToUInt32Bits(origin.X));
            hash = Mix(hash, BitConverter.SingleToUInt32Bits(origin.Y));
            hash = Mix(hash, BitConverter.SingleToUInt32Bits(pixelSize));

            var words = MemoryMarshal.Cast<byte, ulong>(data);
            foreach (ulong word in words)
            {
                hash = Mix(hash, word);
            }
            for (int i = words.Length * 8; i < data.Length; i++)
            {
                hash = Mix(hash, data[i]);
            }
            return hash;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static ulong Mix(ulong h, ulong value) => BitOperations.RotateLeft((h ^ value) * Prime, 31);
        }

        public void Dispose()
        {
            targetBitmap?.Dispose();
            targetBitmap = null;
            stagingBitmap?.Dispose();
            stagingBitmap = null;
        }
    }
}
