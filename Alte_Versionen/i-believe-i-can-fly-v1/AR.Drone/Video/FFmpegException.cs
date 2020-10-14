using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Video
{
    [Serializable()]
    public class FFmpegException : System.Exception
    {
        public FFmpegException() : base() { }
        public FFmpegException(string message) : base(message) { }
        public FFmpegException(string message, System.Exception inner) : base(message, inner) { }

        // A constructor is needed for serialization when an
        // exception propagates from a remoting server to the client. 
        protected FFmpegException(System.Runtime.Serialization.SerializationInfo info,
            System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }
}
