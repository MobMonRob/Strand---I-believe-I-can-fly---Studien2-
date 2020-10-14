using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using System.Windows.Media;

namespace AR.Drone.Client.HUD.Controls
{

    /// <summary>
    /// Generates a color representation of a depth frame.
    /// </summary>
    internal class DepthColorizer
    {
        /// <summary>
        /// Red component shifts for tinting players.
        /// </summary>
        private static readonly int[] IntensityShiftByPlayerR = { 0, 2, 0, 2, 0, 0, 2, 0 };

        /// <summary>
        /// Green component shifts for tinting players.
        /// </summary>
        private static readonly int[] IntensityShiftByPlayerG = { 0, 2, 2, 0, 2, 0, 0, 0 };

        /// <summary>
        /// Blue component shifts for tinting players.
        /// </summary>
        private static readonly int[] IntensityShiftByPlayerB = { 0, 0, 2, 2, 0, 2, 0, 0 };

        /// <summary>
        /// Color to represent background depth.
        /// </summary>
        private static readonly ColorMapping BackgroundDepthColor =
            new ColorMapping { R = 0x00, G = 0x00, B = 0x00 };  // black

        /// <summary>
        /// Color to represent the nearest depth.
        /// </summary>
        private static readonly ColorMapping NearestDepthColor =
            new ColorMapping { R = 0xff, G = 0x00, B = 0x00 };  // white

        /// <summary>
        /// Number of bits per pixel in a Bgr32 format bitmap.
        /// </summary>
        private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Byte offset to the alpha byte of a Bgra32 pixel
        /// </summary>
        private const int AlphaIndex = 3;

        /// <summary>
        /// Byte offset to the red byte of a Bgra32 pixel.
        /// </summary>
        private const int RedIndex = 2;

        /// <summary>
        /// Byte offset to the green byte of a Bgra32 pixel.
        /// </summary>
        private const int GreenIndex = 1;

        /// <summary>
        /// Byte offset to the blue byte of a Bgra32 pixel.
        /// </summary>
        private const int BlueIndex = 0;

        /// <summary>
        /// The nearest depth (in millimeters) to be rendered with a gradient.
        /// </summary>
        private const int MinMinDepth = 400;

        /// <summary>
        /// The furthest depth (in millimeters) to be rendered with a gradient.
        /// </summary>
        private const int MaxMaxDepth = 16383;

        /// <summary>
        /// The sentinel depth value indicating unknown depth.
        /// </summary>
        private const int UnknownDepth = 0;

        /// <summary>
        /// A static lookup table that maps depth (in millimeters) to intensity (0-255).
        /// </summary>
        private static readonly byte[] intensityTable = new byte[MaxMaxDepth + 1];  // 16 KiB

        /// <summary>
        /// A lookup table that maps depth (in millimeters) to a color.
        /// </summary>
        private ColorMapping[] colorMappingTable =
            new ColorMapping[short.MaxValue - short.MinValue + 1];  // 192 KiB

        /// <summary>
        /// Flag to indicate whether the color mapping table needs to be initialized
        /// </summary>
        private bool initializeColorMappingTable = true;

        /// <summary>
        /// Minimum reliable depth in the current color mapping table.
        /// </summary>
        private int currentMinDepth;

        /// <summary>
        /// Maximum reliable depth in the current color mapping table.
        /// </summary>
        private int currentMaxDepth;

        /// <summary>
        /// Initializes static members of the DepthColorizer class.
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1810:InitializeReferenceTypeStaticFieldsInline", Justification = "Loops are necessary to initialize intensityTable")]
        static DepthColorizer()
        {
            for (int i = 0; i < MinMinDepth; i++)
            {
                // Map any value nearer than MinMinDepth to maximum intensity.
                intensityTable[i] = byte.MaxValue;
            }
            
            for (int i = MinMinDepth; i < MaxMaxDepth; i++)
            {
                // Use a logarithmic scale that shows more detail for nearer depths.
                // The constants in this formula were chosen such that values between
                // MinMinDepth and MaxMaxDepth will map to the full range of possible
                // byte values.
                const float DepthRangeScale = 500.0f;
                const int IntensityRangeScale = 74;
                intensityTable[i] = (byte)(~(byte)Math.Min(
                    byte.MaxValue,
                    Math.Log((((double)(i - MinMinDepth)) / DepthRangeScale) + 1) * IntensityRangeScale));
            }
        }

        /// <summary>
        /// Converts an array of DepthImagePixels into a byte array in Bgr32 format.
        /// Pixel intensity represents depth; colors indicate players.
        /// </summary>
        /// <param name="depthFrame">The depth buffer to convert.</param>
        /// <param name="minDepth">The minimum reliable depth for this frame.</param>
        /// <param name="maxDepth">The maximum reliable depth for this frame.</param>
        /// <param name="colorFrame">The buffer to fill with color pixels.</param>
        public void ConvertDepthFrame(
            DepthImagePixel[] depthFrame,
            int minDepth,
            int maxDepth,
            byte[] colorFrame)
        {
            if ((depthFrame.Length * Bgr32BytesPerPixel) != colorFrame.Length)
            {
                throw new InvalidOperationException();
            }

            ColorMapping[] mappingTable = GetColorMappingTable(minDepth, maxDepth);

            for (int depthIndex = 0, colorIndex = 0;
                colorIndex < colorFrame.Length;
                depthIndex++, colorIndex += Bgr32BytesPerPixel)
            {
                short depth = depthFrame[depthIndex].Depth;
                ColorMapping color = mappingTable[(ushort)depth];

                int player = depthFrame[depthIndex].PlayerIndex;

                if (player <= 0)
                {
                    colorFrame[colorIndex + AlphaIndex] = (byte)0x00;
                }
                else
                {
                    // Colorize players
                    colorFrame[colorIndex + RedIndex] = (byte)(color.R >> IntensityShiftByPlayerR[player]);
                    colorFrame[colorIndex + GreenIndex] = (byte)(color.G >> IntensityShiftByPlayerG[player]);
                    colorFrame[colorIndex + BlueIndex] = (byte)(color.B >> IntensityShiftByPlayerB[player]);
                    colorFrame[colorIndex + AlphaIndex] = (byte)0xff;
                }
            }
        }

        /// <summary>
        /// Returns the depth-to-color mapping table.
        /// </summary>
        /// <param name="minDepth">The minimum reliable depth value.</param>
        /// <param name="maxDepth">The maximum reliable depth value.</param>
        /// <returns>The color mapping table.</returns>
        private ColorMapping[] GetColorMappingTable(int minDepth, int maxDepth)
        {
            if (this.initializeColorMappingTable ||
                minDepth != this.currentMinDepth ||
                maxDepth != this.currentMaxDepth)
            {
                this.initializeColorMappingTable = false;
                this.currentMinDepth = minDepth;
                this.currentMaxDepth = maxDepth;

                // Initialize table to all zero (black)
                Array.Clear(this.colorMappingTable, 0, this.colorMappingTable.Length);

                // Fill in values that will be rendered normally with a gray gradient
                for (int i = minDepth; i < maxDepth; i++)
                {
                    byte intensity = intensityTable[i];
                    this.colorMappingTable[i] = new ColorMapping
                    {
                        R = intensity,
                        G = intensity,
                        B = intensity
                    };
                }
            }

            return this.colorMappingTable;
        }

        /// <summary>
        /// Represents an entry in the depth-to-color lookup table.
        /// </summary>
        private struct ColorMapping
        {
            public byte R;
            public byte G;
            public byte B;
        }
    }
}
