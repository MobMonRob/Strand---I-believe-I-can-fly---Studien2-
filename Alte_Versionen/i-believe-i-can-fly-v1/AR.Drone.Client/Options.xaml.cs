using AR.Drone.Control;
using Microsoft.Kinect.Toolkit.Controls;
using System;
using System.Windows;
using System.Windows.Controls;

namespace AR.Drone.Client
{
    /// <summary>
    /// Interaktionslogik für Options.xaml
    /// </summary>
    public partial class Options : Page
    {
        private KinectCircleButton currentSensitivity;
        private MainWindow _window;

        public Options(MainWindow window)
        {
            InitializeComponent();
            this._window = window;

            kinectRegion.KinectSensor = window.kinectController.sensor;
            this.currentSensitivity = (KinectCircleButton)this.FindName("Button" + _window.droneSensitivity );
            this.currentSensitivity.IsEnabled = false;
            
        }

        private void btnBackClicked(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(_window.mainMenu);
        }

        private void btnSensitivityClicked(object sender, RoutedEventArgs e)
        {
            this.currentSensitivity.IsEnabled = true;
            this.currentSensitivity = (KinectCircleButton) sender;
            this.currentSensitivity.IsEnabled = false;
            int sensitivity = int.Parse((string)currentSensitivity.Content);

            _window.droneSensitivity = sensitivity;
        }

        ~Options() 
        {
            Console.WriteLine("Options terminated gracefully!");
        }

        private void Page_Loaded(object sender, RoutedEventArgs e)
        {
            System.GC.Collect();
        }

        private void Page_Unloaded(object sender, RoutedEventArgs e)
        {
            kinectRegion.KinectSensor = null;
            kinectRegion = null;
            System.GC.Collect();
        }

        private void KinectCircleButton_Click(object sender, RoutedEventArgs e)
        {
            System.GC.Collect();
        }
    }
}
