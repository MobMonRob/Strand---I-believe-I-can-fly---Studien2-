using AR.Drone.Configuration;
using AR.Drone.Util;
using AR.Drone.Video.Data;
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.Video
{
    internal class VideoReceiver : Worker
    {
        private bool _disposed = false;

        private const int BUFFER_SIZE = 8192;
        private const uint HEADER_SIGNATURE = 1163288912;
        private readonly uint HEADER_SIZE;
        private BlockingCollection<VideoFrame> _videoFrameQueue;

        public VideoReceiver(BlockingCollection<VideoFrame> videoFrameQueue)
            :base("VideoReceiver")
        {
            _videoFrameQueue = videoFrameQueue;
            unsafe
            {
                HEADER_SIZE = (uint) sizeof(parrot_video_encapsulation_t);
            }
        }

        private void ReceiveStream()
        {
            using (var tcpClient = new TcpClient(DroneConstants.IP_ADDRESS, DroneConstants.VIDEO_STREAM_PORT))
            {
                var watch = Stopwatch.StartNew();
                Console.WriteLine("Connection is open!");
                byte[] frameBuffer,
                       headerDiffBuff,
                       headerBuffer = new byte[HEADER_SIZE];

                NetworkStream networkStream = tcpClient.GetStream();
                networkStream.ReadTimeout = 1000;
                parrot_video_encapsulation_t header;

                while (!IsStopRequested())
                {
                    // Read the header from the stream
                    ReadUntilBufferIsFull(ref headerBuffer, ref networkStream);
                    while (!IsHeaderValid(ref headerBuffer))
                    {
                        // Terminate connection when header signature is wrong
                        break;
                    }

                    header = ParseHeader(ref headerBuffer);
                    headerDiffBuff = new byte[header.header_size - HEADER_SIZE];
                    ReadUntilBufferIsFull(ref headerDiffBuff, ref networkStream);

                    // Read payload from the stream
                    frameBuffer = new byte[header.payload_size];
                    ReadUntilBufferIsFull(ref frameBuffer, ref networkStream);

                    _videoFrameQueue.Add(new VideoFrame { header = header, payload = frameBuffer });
                }
            }

            if (!IsStopRequested())
            {
                Console.WriteLine("Video receiver has lost the connection");
                // connection possible lost, so wait before reconnect
                Thread.Sleep(1000);
            }
        }

        internal override void DoWork()
        {
            // this while enables us to rebuild a connection if we lose the data stream
            while (!IsStopRequested())
            {
                try
                {
                    ReceiveStream();
                }
                catch (Exception e)
                {
                    resultState.successfull = false;
                    resultState.exception = e;
                    return;
                }
            }
        }

        /// <summary>
        /// Checks if the data is a valid header by testing the first 4 Bytes
        /// </summary>
        /// <param name="headerData">The byte array that should be the header</param>
        /// <returns>Whether the data is a valid header or not</returns>
        private unsafe bool IsHeaderValid(ref byte[] headerData) 
        {
            fixed (byte* pData = &headerData[0]) {
                return (*(uint*)pData == HEADER_SIGNATURE);
            }
        }

        /// <summary>
        /// Parses the data to the parrot video structure
        /// </summary>
        /// <param name="data">The data that should be parsed</param>
        private unsafe parrot_video_encapsulation_t ParseHeader(ref byte[] headerData)
        {
            fixed (byte* pData = &headerData[0]) {
                return *(parrot_video_encapsulation_t*)pData;
            } 
        }

       /// <summary>
       /// Reads from a stream until the given byte-Array is full.
       /// </summary>
       /// <param name="buffer">The byte-Array that should be filled</param>
       /// <param name="networkStream">The stream from which the data should be read</param>
        private void ReadUntilBufferIsFull(ref byte[] buffer, ref NetworkStream networkStream) {
            int offset = networkStream.Read(buffer, 0, buffer.Length);

            while (offset < buffer.Length && !IsStopRequested())
            {
                offset += networkStream.Read(buffer, offset, buffer.Length - offset);
               // Console.WriteLine(string.Format("Ich hänge in der Schleife fest! Offset: {0}, Buffer: {1}", offset, buffer.Length));
            }
        }

        internal override void OnStopRequest()
        {
           // Not needed now
        }

        protected override void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!_disposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    //Nothing todo here, yet
                }
                _disposed = true;
            }
            base.Dispose(isSaveToRemoveManagedObjects);
        }
    }
}
