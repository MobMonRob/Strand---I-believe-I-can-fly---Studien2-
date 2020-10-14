using AR.Drone.Control;
using AR.Drone.Headtracker;
using AR.Drone.Kinect;
using AR.Drone.Video;
using AR.Drone.Video.Data;
using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace AR.Drone.Client
{
    /// <summary>
    /// Interaktionslogik für DroneCockpit.xaml
    /// </summary>
    public partial class DroneCockpit : Page
    {
        private DroneController _droneController;
        private WriteableBitmap _bitmap;
        private Int32Rect _bitmapSourceRect;
        private KinectController _kinectController;
        private HeadTrackerController _headtrackerController;
        private MainWindow _window;

        public DroneCockpit(MainWindow window)
        {
            InitializeComponent();
            this._window = window;
            this._droneController = window.droneController;
            this._kinectController = window.kinectController;

            this._kinectController.NavigateBack += kinectController_NavigateBack;
        }

        private void kinectController_NavigateBack(object sender, EventArgs e)
        {
            this.NavigateBack();
        }


        internal void CloseCockpit() 
        {
            if (null != this._headtrackerController)
            {
                this._headtrackerController.RequestStop();
            }
            if (null != this._droneController)
            {
                this._droneController.Dispose();
            }
        }

        internal void InitCockpit()
        {
            InitializeDataBinding();

            InitializeHeadtracker();

            HudViewer.SetKinectSensor(_kinectController.sensor);

            _droneController.FrameReady += FrameReadyHandler;

            try
            {
                // ensure that emergency mode is deactivated
                _droneController.RecoverFromEmergency();
            }
            catch (TimeoutException e) 
            {
                Console.WriteLine(e.Message);
            }
           
        }

        private void Cockpit_Loaded(object sender, RoutedEventArgs e)
        {
            System.GC.Collect();
            this._kinectController.onDroneCockpit = true;
            InitCockpit();
        }

        /// <summary>
        /// Initializes the data binding for the UI elements
        /// </summary>
        private void InitializeDataBinding()
        {
            HudViewer.DataContext = _droneController.Navdata;
        }

        private void InitializeHeadtracker()
        {
            if (null == _headtrackerController) 
            {
                _headtrackerController = new HeadTrackerController();
            }

            _headtrackerController.InitializeTracker();
            if (!_headtrackerController.errorState.hasError)
            {
                _headtrackerController.StartAsThread();
            }
        }

        /// <summary>
        /// Writes a new a video frame onto the image element
        /// </summary>
        /// <param name="sender">The sender of the frame</param>
        /// <param name="frame">The decoded video frame</param>
        private void FrameReadyHandler(object sender, VideoFrameReadyEventArgs args)
        {
            DecodedVideoFrame frame = args.frame;

            Dispatcher.BeginInvoke((Action)(() =>
            {
                if (this._bitmap == null ||
                this._bitmap.PixelHeight != frame.height || this._bitmap.PixelWidth != frame.width)
                {
                    _bitmap = new WriteableBitmap(frame.width, frame.height, 96, 96, PixelFormats.Bgr24, null);
                    _bitmapSourceRect = new Int32Rect(0, 0, frame.width, frame.height);

                    this.Image.Source = _bitmap;
                }

                _bitmap.WritePixels(
                    _bitmapSourceRect,
                    frame.pixelData,
                    frame.stride,
                    0);

            }));
        }

        ~DroneCockpit() 
        {
            Console.WriteLine("DroneCockpit terminated gracefully!");
        }

        internal void NavigateBack()
        {
            NavigationService.Navigate(_window.mainMenu);
        }

        private void Page_Unloaded(object sender, RoutedEventArgs e)
        {
            if (_droneController != null) _droneController.FrameReady -= FrameReadyHandler;
            if (_kinectController != null) _kinectController.NavigateBack -= kinectController_NavigateBack;
            HudViewer.DataContext = null;
            HudViewer.Destruct();

            HudViewer = null;
            _bitmap = null;

            _headtrackerController.RequestStop();
            _headtrackerController.Join();

            System.GC.Collect();
        }
    }
}
