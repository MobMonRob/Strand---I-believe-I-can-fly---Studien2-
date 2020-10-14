using AR.Drone.Util;
using AR.Drone.Video.Data;
using FFmpeg.AutoGen;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.Video
{
    internal unsafe class VideoFrameDecoder : Worker
    {
        private bool _disposed = false;

        private const int BLOCKING_QUEUE_TIMEOUT = 100;

        //FFMPEG Decoding
        private const AVCodecID CODEC_ID = AVCodecID.AV_CODEC_ID_H264;
        private readonly AVPixelFormat DESTINATION_IMG_FORMAT = AVPixelFormat.AV_PIX_FMT_BGR24;
        private AVCodecContext* _pDecodingContext;
        private AVFrame* _sourceFrame, _destinationFrame;
        private SwsContext* _swsContext;

        // buffer for the destionation frame
        private byte* _destinationFrameBuffer;

        private int _currentHeight = 0;
        private int _currentWidth = 0;

        private BlockingCollection<VideoFrame> _videoFrameCollection;

        private CancellationTokenSource _cancellellationTokenSource;

        // Events
        internal delegate void FrameReadyEventHandler(object sender, VideoFrameReadyEventArgs e);
        internal event FrameReadyEventHandler FrameReady;

        internal VideoFrameDecoder(BlockingCollection<VideoFrame> videoFrameCollection)
            :base("VideoFrameDecoder")
        {

            this._videoFrameCollection = videoFrameCollection;
            _cancellellationTokenSource = new CancellationTokenSource();
        }

        /// <summary>
        /// Registers all Codecs for FFMPEG
        /// </summary>
        static VideoFrameDecoder()
        {
            string ffmpegPath = string.Format(DroneConstants.FFMPEG_PATH, Environment.Is64BitProcess ? "x64" : "x86");
            NativeMethods.SetDllDirectory(ffmpegPath); 
            FFmpegInvoke.avcodec_register_all();
        }

        /// <summary>
        /// In this Method videoFrames are taken out of the blockingQueue for decoding.
        /// </summary>
        internal override void DoWork()
        {
            try
            {
                InitializeAndDecoderLoop();
            }
            catch (Exception e)
            {
                resultState.successfull = false;
                resultState.exception = e;
            }
            
        }

        /// <summary>
        /// Initializes FFmpeg Decoder and runs the decoding loop
        /// </summary>
        private void InitializeAndDecoderLoop()
        {
            InitializeVideoFrameDecoder();

            while (!IsStopRequested())
            {
                try
                {
                    VideoFrame frame = this._videoFrameCollection.Take(_cancellellationTokenSource.Token); //to be able to exit blocking mode we have to define a token.

                    if (frame != null)
                    {
                        DecodePackage(ref frame);
                        ProcessDecodedPackage();
                    }
                }
                catch (OperationCanceledException)
                {
                    // cancellation token received
                    resultState.successfull = true;
                    return;
                }
            }
        }

        /// <summary>
        /// Checks if the image dimension has changed and therefor reallocates the needed resources
        /// </summary>
        private void ReserveRessourcesIfNeeded()
        {
            // did the frame dimension change?
            if (_currentHeight != _sourceFrame->height || _currentWidth != _sourceFrame->width)
            {
                _currentHeight = _sourceFrame->height;
                _currentWidth = _sourceFrame->width;

                CreateSWSContext();
                CreateDestionationFrame();
            }
        }

        /// <summary>
        /// Allocates memory for the destination frame according to the current width and height.
        /// Should only be called if the width or height of the source frame has changed
        /// </summary>
        private void CreateDestionationFrame()
        {
            // calculate the required bytes
            int numBytes = FFmpegInvoke.avpicture_get_size(DESTINATION_IMG_FORMAT, _currentWidth, _currentHeight);

            if (_destinationFrameBuffer != null)
            {
                FFmpegInvoke.av_free(_destinationFrameBuffer);
            }

            // reserve memory and give it to the destionation frame
            _destinationFrameBuffer = (byte*)FFmpegInvoke.av_malloc((uint)numBytes);
            FFmpegInvoke.avpicture_fill((AVPicture*)_destinationFrame, _destinationFrameBuffer, DESTINATION_IMG_FORMAT, _currentWidth, _currentHeight);
        }

        /// <summary>
        /// A new sws context is created and the old one is removed.
        /// Therefor the current width and height is used
        /// </summary>
        private void CreateSWSContext()
        {
            // is the context initialized?
            if (_swsContext != null)
            {
                // free memory
                FFmpegInvoke.sws_freeContext(_swsContext);
            }

            _swsContext = FFmpegInvoke.sws_getContext(_currentWidth, _currentHeight, _pDecodingContext->pix_fmt, _currentWidth, _currentHeight,
                DESTINATION_IMG_FORMAT, FFmpegInvoke.SWS_FAST_BILINEAR, null, null, null);
        }


        /// <summary>
        /// Method to process the decoded Video Frame.
        /// This method is called after a frame was decoded.
        /// </summary>
        private void ProcessDecodedPackage()
        {
            ReserveRessourcesIfNeeded();
            byte** src = &_sourceFrame->data_0;
            byte** dest = &_destinationFrame->data_0;

            FFmpegInvoke.sws_scale(_swsContext, src, _sourceFrame->linesize, 0, _currentHeight,
                dest, _destinationFrame->linesize);

            byte* destFrame = _destinationFrame->data_0;
            var pIimageBuffer = new IntPtr(destFrame);

            if (null == pIimageBuffer)
            {
                // should only happen while disposing this thread
                Console.WriteLine("VideoFrameDecoder: imageBuffer is null. Is Thread disposing?");
                return;
            }

            var frame = new DecodedVideoFrame(_currentWidth, _currentHeight, _destinationFrame->linesize[0], pIimageBuffer);
            
            // Fire frameReady Event
            if (null != FrameReady)
            {
                FrameReady(this, new VideoFrameReadyEventArgs(frame));
            }


           // Marshal.FreeHGlobal(pIimageBuffer);
        }

        /// <summary>
        /// Method for processing a new bitmap, which was generated from a frame.
        /// It is called, when a new bitmap is available
        /// </summary>
        /// <param name="bitmap"></param>
        private void ProcessBitmap(Bitmap bitmap)
        {
            // TODO do something with the bitmap
            /*using (var stream = new MemoryStream())
            {
                bitmap.Save(stream, ImageFormat.Bmp);
                byte[] data = stream.ToArray();
                if(this._socketHandler.droneClient != null)
                    this._socketHandler.droneClient.Send(data);
            }*/
            //_processBitmapAction(bitmap);
        }

        /// <summary>
        /// Ovverwritten method On StopRequest. 
        /// Within this method the Cancellation Token is set to Cancel to stop blocking the queue.
        /// </summary>
        internal override void OnStopRequest()
        {
            _cancellellationTokenSource.Cancel();
        }

        /// <summary>
        /// Decodes the VideoFrame received from the Drones Video Stream
        /// </summary>
        /// <param name="frame">The VideoFrame to be decoded</param>
        private unsafe void DecodePackage(ref VideoFrame video_frame)
        {
            AVPacket avPacket = new AVPacket();

            fixed (byte* data = &video_frame.payload[0])
            {
                avPacket.size = video_frame.payload.Length;
                avPacket.data = data;

                int hasPicture;
                int decodedSize = FFmpegInvoke.avcodec_decode_video2(_pDecodingContext, _sourceFrame, &hasPicture, &avPacket);
                if (decodedSize < 0)
                {
                    //TODO: implement appropriate handling for decoding error
                }
            }
        }

        /// <summary>
        /// Initializes relevant stuff for FFMPEG Decoding
        /// </summary>
        private unsafe void InitializeVideoFrameDecoder() {
            _sourceFrame = FFmpegInvoke.av_frame_alloc();
            _destinationFrame = FFmpegInvoke.av_frame_alloc();

            AVCodec* codec = FFmpegInvoke.avcodec_find_decoder(CODEC_ID);

            if (codec == null)
            {
                throw new FFmpegException("Invalid Codec ID " + CODEC_ID);
            }                

            _pDecodingContext = FFmpegInvoke.avcodec_alloc_context3(codec);


            if (FFmpegInvoke.avcodec_open2(_pDecodingContext, codec, null) < 0)
            {
                throw new FFmpegException("Could not open Codec");
            }
        }

        /// <summary>
        /// Frees the given frame
        /// </summary>
        /// <param name="frame"></param>
        private unsafe void FreeFrame(AVFrame* frame)
        {
            AVFrame* frameOnStack = frame;
            FFmpegInvoke.av_frame_free(&frame);
        }

        protected override void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!_disposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    // terminate thread first
                    RequestStop();
                    this.Join();

                    _cancellellationTokenSource.Dispose();
                }

                FFmpegInvoke.avcodec_close(_pDecodingContext);

                if (_destinationFrameBuffer != null)
                {
                    FFmpegInvoke.av_free(_destinationFrameBuffer);
                }

                if (_swsContext != null)
                {
                    FFmpegInvoke.sws_freeContext(_swsContext);
                }

                FreeFrame(_sourceFrame);
                FreeFrame(_destinationFrame);

                _disposed = true;
            }
            base.Dispose(isSaveToRemoveManagedObjects);
        }
    }
}
