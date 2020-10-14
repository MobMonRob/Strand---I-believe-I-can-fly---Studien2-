using AR.Drone.Control;
using AR.Drone.Headtracker;
using AR.Drone.Kinect;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using System;
using System.Windows;
using System.Windows.Input;

namespace AR.Drone.Client
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        internal KinectController kinectController { get; set; }
        internal DroneController droneController { get; set; }
        internal KinectSensorChooser kinectSensorChooser { get; set; }
        internal bool IsDroneConnected { get; set; }
        internal MainMenu mainMenu;

        private int _droneSensitivity;
        internal int droneSensitivity
        {
            get 
            {
                return _droneSensitivity;
            }
            set
            {
                _droneSensitivity = value;
                if (null != droneController)
                    droneController.SetSensivity(value);
            }
        }

        public MainWindow()
        {
            InitializeComponent();
            droneSensitivity = 5;
        }

        private void InitializeControllers() 
        {
            kinectController = new KinectController();
            kinectSensorChooser = new KinectSensorChooser();

            kinectSensorChooser.KinectChanged += SensorChooserOnKinectChanged;
            kinectSensorChooser.Start();
        }

        private void Cockpit_Closed(object sender, EventArgs e)
        {
            SaveShutdown();
        }

        private void Cockpit_Loaded(object sender, RoutedEventArgs e)
        {
            InitializeControllers();
            mainMenu = new MainMenu(this);
            this.mainFrame.Navigate(mainMenu);
        }

        /// <summary>
        /// Frees the kinect and the drone controller to ensure a safe application shutdown
        /// </summary>
        private void SaveShutdown()
        {
            if (null != kinectController)
            {
                kinectController.StopKinect();
            }

            if (null != droneController && !droneController.isDisposed)
            {
                droneController.Dispose();
            }
        }

        
        private void SensorChooserOnKinectChanged(object sender, KinectChangedEventArgs args)
        {
            bool error = false;
            if (args.OldSensor != null)
            {
                try
                {
                    args.OldSensor.DepthStream.Range = DepthRange.Default;
                    args.OldSensor.SkeletonStream.EnableTrackingInNearRange = false;
                    args.OldSensor.DepthStream.Disable();
                    args.OldSensor.SkeletonStream.Disable();
                }
                catch (InvalidOperationException)
                {
                    // KinectSensor might enter an invalid state while enabling/disabling streams or stream features.
                    // E.g.    sensor might be abruptly unplugged.
                    error = true;
                }
            }

            if (args.NewSensor != null)
            {
                try
                {
                    args.NewSensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                    args.NewSensor.SkeletonStream.Enable();
                    args.NewSensor.ColorStream.Enable();

                    try
                    {
                        args.NewSensor.DepthStream.Range = DepthRange.Near;
                        args.NewSensor.SkeletonStream.EnableTrackingInNearRange = true;
                        args.NewSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                    }
                    catch (InvalidOperationException)
                    {
                        // Non Kinect for Windows devices do not support Near mode, so reset back to default mode.
                        args.NewSensor.DepthStream.Range = DepthRange.Default;
                        args.NewSensor.SkeletonStream.EnableTrackingInNearRange = false;
                        error = true;
                    }
                }
                catch (InvalidOperationException)
                {
                    error = true;
                    // KinectSensor might enter an invalid state while enabling/disabling streams or stream features.
                    // E.g.    sensor might be abruptly unplugged.
                }
            }

            if (!error)
            {
                kinectController.Initialize(args.NewSensor);
            }
        }

        private void mainFrame_Navigated(object sender, System.Windows.Navigation.NavigationEventArgs e)
        {
            mainFrame.NavigationService.RemoveBackEntry();
        }

        
    }

}
