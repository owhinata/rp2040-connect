// SPDX-License-Identifier: Apache-2.0
namespace ImuStudio.Core.Processing;

/// <summary>In-place radix-2 FFT of a fixed power-of-two size.</summary>
public sealed class Fft
{
    private readonly int[] _reversed;
    private readonly float[] _cos, _sin;

    public int Size { get; }

    public Fft(int size)
    {
        if (size < 2 || (size & (size - 1)) != 0)
        {
            throw new ArgumentException("The size must be a power of two.", nameof(size));
        }
        Size = size;
        int bits = int.TrailingZeroCount(size);
        _reversed = new int[size];
        for (int i = 0; i < size; i++)
        {
            int r = 0;
            for (int b = 0; b < bits; b++)
            {
                r |= (i >> b & 1) << (bits - 1 - b);
            }
            _reversed[i] = r;
        }
        _cos = new float[size / 2];
        _sin = new float[size / 2];
        for (int i = 0; i < size / 2; i++)
        {
            double angle = -2 * Math.PI * i / size;
            _cos[i] = (float)Math.Cos(angle);
            _sin[i] = (float)Math.Sin(angle);
        }
    }

    public void Transform(Span<float> re, Span<float> im)
    {
        int n = Size;
        for (int i = 0; i < n; i++)
        {
            int j = _reversed[i];
            if (j > i)
            {
                (re[i], re[j]) = (re[j], re[i]);
                (im[i], im[j]) = (im[j], im[i]);
            }
        }
        for (int half = 1; half < n; half <<= 1)
        {
            int step = n / (half * 2);
            for (int start = 0; start < n; start += half * 2)
            {
                for (int k = 0; k < half; k++)
                {
                    float c = _cos[k * step], s = _sin[k * step];
                    int a = start + k, b = a + half;
                    float tr = re[b] * c - im[b] * s;
                    float ti = re[b] * s + im[b] * c;
                    re[b] = re[a] - tr;
                    im[b] = im[a] - ti;
                    re[a] += tr;
                    im[a] += ti;
                }
            }
        }
    }
}
