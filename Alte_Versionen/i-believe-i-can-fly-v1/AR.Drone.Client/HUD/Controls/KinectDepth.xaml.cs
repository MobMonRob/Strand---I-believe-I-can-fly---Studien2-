using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.IO;

namespace AR.Drone.Client.HUD.Controls
{
    /// <summary>
    /// Interaktionslogik für KinectDepth.xaml
    /// </summary>
    public partial class KinectDepth : UserControl
    {
        private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        private KinectSensor sensor;
        private DepthImagePixel[] pixelData;
        private DepthImageFormat lastImageFormat;
        private byte[] depthFrame32;
        private WriteableBitmap outputBitmap;
        private DepthColorizer colorizer = new DepthColorizer();

        public KinectDepth()
        {
            InitializeComponent();
        }

        private void DepthImageReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            int imageWidth = 0;
            int imageHeight = 0;
            bool haveNewFormat = false;
            int minDepth = 0;
            int maxDepth = 0;

            using (DepthImageFrame imageFrame = e.OpenDepthImageFrame())
            {
                if (imageFrame != null)
                {
                    imageWidth = imageFrame.Width;
                    imageHeight = imageFrame.Height;

                    haveNewFormat = this.lastImageFormat != imageFrame.Format;

                    if (haveNewFormat)
                    {
                        this.pixelData = new DepthImagePixel[imageFrame.PixelDataLength];
                        this.depthFrame32 = new byte[imageFrame.Width * imageFrame.Height * Bgr32BytesPerPixel];
                        this.lastImageFormat = imageFrame.Format;
                    }

                    imageFrame.CopyDepthImagePixelDataTo(this.pixelData);
                    minDepth = imageFrame.MinDepth;
                    maxDepth = imageFrame.MaxDepth;
                }
            }

            // Did we get a depth frame?
            if (imageWidth != 0)
            {
                colorizer.ConvertDepthFrame(this.pixelData, minDepth, maxDepth, this.depthFrame32);

                this.Dispatcher.BeginInvoke((Action)(() =>
                {
                    if (haveNewFormat)
                    {
                        this.outputBitmap = new WriteableBitmap(
                            imageWidth,
                            imageHeight,
                            96, // DpiX
                            96, // DpiY
                            PixelFormats.Bgra32,
                            null);

                        this.kinectDepthImage.Source = this.outputBitmap;
                    }

                    this.outputBitmap.WritePixels(
                        new Int32Rect(0, 0, imageWidth, imageHeight),
                        this.depthFrame32,
                        imageWidth * Bgr32BytesPerPixel,
                        0);
                }));
            }
        }

        internal void Destruct() 
        {
            if (null != this.sensor) this.sensor.DepthFrameReady -= this.DepthImageReady;
        }

        ~KinectDepth() 
        {
            Console.WriteLine("KinectDepth terminated gracefully!");
        }

        internal void Initialize(KinectSensor sensor)
        {
            if (null != sensor)
            {
                this.sensor = sensor;
            }

            if (null != this.sensor)
            {
                this.sensor.DepthFrameReady += this.DepthImageReady;

                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }
        }

    }
}
