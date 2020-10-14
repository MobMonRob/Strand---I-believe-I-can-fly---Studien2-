using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Video.Data
{
    public class VideoFrame
    {
        internal parrot_video_encapsulation_t header { get; set; }
        internal byte[] payload { get; set; }
    }
}
