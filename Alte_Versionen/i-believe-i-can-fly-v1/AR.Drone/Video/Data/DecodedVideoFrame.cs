using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Video.Data
{
    public class DecodedVideoFrame
    {
        public int width { private set; get; }
        public int height { private set; get; }
        public int stride { private set; get; }
        public byte[] pixelData { private set; get; }

        public DecodedVideoFrame(int width, int height, int stride, byte[] pixelData)
        {
            this.width = width;
            this.height = height;
            this.stride = stride;
            this.pixelData = pixelData;
        }

        public DecodedVideoFrame(int width, int height, int stride, IntPtr dataPointer)
        {
            this.width = width;
            this.height = height;
            this.stride = stride;

            pixelData = new byte[stride * height];
            System.Runtime.InteropServices.Marshal.Copy(dataPointer, pixelData, 0, pixelData.Length);
        }
    }
}
