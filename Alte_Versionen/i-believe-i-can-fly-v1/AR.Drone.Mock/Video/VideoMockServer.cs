using AR.Drone.Configuration;
using AR.Drone.Mock;
using AR.Drone.Util;
using AR.Drone.Video.Demo;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Runtime.Serialization.Formatters.Binary;
using System.Threading;

namespace AR.Drone.Mock.Video
{
    public class VideoMockServer : Worker
    {
        private List<RawVideoFrame> frameList;
        private TcpListener _tcpListener;

        private const int FPS = 30;
        private const int FRAME_TIME_DELTA = 1000 / FPS;

        public VideoMockServer(string videoFile):base("VideoMockServer")
        {
            BinaryFormatter binaryFormatter = new BinaryFormatter();
            frameList = (List<RawVideoFrame>) binaryFormatter.Deserialize(File.Open(videoFile, FileMode.Open));
            _tcpListener = new TcpListener(IPAddress.Parse("127.0.0.1"), NetworkConfig.VIDEO_STREAM_PORT);
            _tcpListener.Start(1);
        }

        public override void DoWork()
        {

            while (!IsStopRequested())
            {
                using (TcpClient client = _tcpListener.AcceptTcpClient())
                {
                    var stream = client.GetStream();
                    int framePos = 0;
                    RawVideoFrame currentFrame;

                    while (!IsStopRequested())
                    {
                        currentFrame = frameList[framePos];
                        stream.Write(currentFrame.frameData, 0, currentFrame.frameData.Length);

                        // send the first frame again, when end is reached
                        framePos = (framePos == frameList.Count - 1) ? 0 : framePos + 1;

                        // limit the data to the given FPS
                        Thread.Sleep(FRAME_TIME_DELTA);
                    }

                }
            }
          
        }

        public override void OnStopRequest()
        {
            _tcpListener.Stop();
        }
    }
}
