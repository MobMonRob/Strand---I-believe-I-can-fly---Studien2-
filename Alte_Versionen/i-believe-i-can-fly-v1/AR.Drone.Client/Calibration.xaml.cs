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
using System.Windows.Navigation;
using System.Windows.Shapes;
using AR.Drone.Kinect;
using AR.Drone.Client;
using System.Windows.Forms;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using AR.Drone.Client.Ressources;
using System.Windows.Media.Animation;

namespace AR.Drone.Client.CalibrationBinding
{
    /// <summary>
    /// Interaktionslogik für Calibration.xaml
    /// </summary>
    public partial class Calibration : Page, IDisposable
    {
        private bool _disposed = false;
        private KinectController _kinectController;
        private Timer aTimer;
        private String armLengthStatus;
        private String backTiltStatus;
        private MainWindow _window;
        private bool _showCalibrationNeededPopup;

        public Calibration(MainWindow window, bool showCalibrationNeededPopup)
        {
            InitializeComponent();
            this._window = window;
            this._showCalibrationNeededPopup = showCalibrationNeededPopup;
        }

        private void btnBackClicked(object sender, RoutedEventArgs e)
        {
            aTimer.Stop();
            this._kinectController.DeactivateCalibration();
            this.NavigationService.Navigate(_window.mainMenu);
        }

        private void btnRefreshClicked(object sender, RoutedEventArgs e)
        {
            this._kinectController.ActivateCalibration();
            BtnRefresh.Visibility = Visibility.Hidden;
            kinectDepth.Background.Opacity = 100;
            _kinectController.calibrationDataSource.ArmLengthStatus = Status.Initialized.ToString();
            _kinectController.calibrationDataSource.BackTiltStatus = Status.Initialized.ToString();
        }

        private void Page_Loaded(object sender, RoutedEventArgs e)
        {
            
            this._kinectController = _window.kinectController;

            this.lblCalibrationNeeded.Content = CommonRessource.CalibrationNeeded;

            if (this._showCalibrationNeededPopup) {
                 this.CalibrationNeededPopup.Visibility = Visibility.Visible;

                 ((Storyboard)FindResource("animateCalibrationNeeded")).Begin(this.CalibrationNeededPopup);

                 // Hide Calibrationpopup after 2 Seconds
                 /*Task.Delay(2000).ContinueWith(_ =>
                 {
                     Dispatcher.Invoke(() =>
                     {
                         this.CalibrationNeededPopup.Visibility = Visibility.Hidden;
                     });
                 });*/
            }

            kinectRegion.KinectSensor = this._kinectController.sensor;
            kinectDepth.Initialize(this._kinectController.sensor);

            // Essential because of unforeseenable polling behaviour
            armLengthStatus = ArmLengthStatus.Status;
            backTiltStatus = BackTiltStatus.Status;

            // Initialize Polling Timer
            aTimer = new Timer();
            aTimer.Tick += new EventHandler(OnTimedEvent);
            aTimer.Interval = 100;

            aTimer.Start();
            this._kinectController.ActivateCalibration();
        }

        private void OnTimedEvent(object sender, EventArgs e)
        {
            if (_kinectController.HasCalibrationStarted())
                kinectDepth.Background.Opacity = 0;

            if (_kinectController.HasCalibrationFinished())
            {
                _window.mainMenu._isCalibrated = true;
                _kinectController.DeactivateCalibration();
                BtnRefresh.Visibility = Visibility.Visible;
            }
            String pollingStatus = _kinectController.calibrationDataSource.ArmLengthStatus;
            if (pollingStatus != null)
                if (!armLengthStatus.Equals(pollingStatus))
                {
                    ArmLengthStatus.Status = pollingStatus;
                    armLengthStatus = pollingStatus;
                }

            pollingStatus = _kinectController.calibrationDataSource.BackTiltStatus;
            if (pollingStatus != null)
                if (!backTiltStatus.Equals(pollingStatus))
                {
                    BackTiltStatus.Status = pollingStatus;
                    backTiltStatus = pollingStatus;
                }
        }

        private void Page_Unloaded(object sender, RoutedEventArgs e)
        {
            aTimer.Stop();
            aTimer.Dispose();

            kinectRegion.KinectSensor = null;
            kinectRegion = null;
            this.kinectDepth.Destruct();

            System.GC.Collect();
        }

        ~Calibration()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!_disposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    aTimer.Dispose();
                }
                _disposed = true;
            }
        }
    }
}
