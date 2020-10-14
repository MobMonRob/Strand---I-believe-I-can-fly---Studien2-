using AR.Drone.Video.Data;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Video
{
    public class VideoFrameReadyEventArgs : EventArgs
    {
        public DecodedVideoFrame frame { get; set; }

        public VideoFrameReadyEventArgs(DecodedVideoFrame frame)
        {
            this.frame = frame;
        }
    }
}
