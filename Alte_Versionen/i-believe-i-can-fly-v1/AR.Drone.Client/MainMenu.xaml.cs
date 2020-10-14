using AR.Drone.Client.CalibrationBinding;
using AR.Drone.Client.Ressources;
using AR.Drone.Control;
using AR.Drone.Headtracker;
using AR.Drone.Kinect;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using System;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Navigation;

namespace AR.Drone.Client
{
    /// <summary>
    /// Interaktionslogik für MainMenu.xaml
    /// </summary>
    public partial class MainMenu : Page, IDisposable
    {
        private bool _disposed = false;
        private bool _isConnecting = false;
        internal bool _isCalibrated = false;
        private MainWindow _window { get; set; }

        private KinectController _kinectController;
        private DroneController _droneController;

        private bool _isConnected { 
            get 
            {
                return _window.IsDroneConnected;
            }
            set 
            {
                _window.IsDroneConnected = value;
            }
        }

        public MainMenu(MainWindow window)
        {
            InitializeComponent();
            _window = window;
            _kinectController = _window.kinectController;

        }

        private void kinectController_SensorChanged(object sender, SensorChangedEventArgs e)
        {
            kinectRegion.KinectSensor = e.sensor;

            VisualizeConnectionState();
        }

        /// <summary>
        /// Visualizes the current connection state according to the provided params
        /// </summary>
        /// <param name="IsConnecting">Wether the program is trying to establish a connection to the drone at the moment</param>
        private void VisualizeConnectionState(bool IsConnecting)
        {
            this._isConnecting = IsConnecting;
            VisualizeConnectionState();
        }

        /// <summary>
        /// Visualizes the current connection state according to the provided params
        /// </summary>
        private void VisualizeConnectionState()
        {
            if (_isConnected)
            {
                startFlyingTile.Label = CommonRessource.StartFlying;
                startFlyingTile.IsEnabled = true;

                refreshCtrl.Visibility = System.Windows.Visibility.Hidden;
                refreshCtrl.IsSpinning = false;

                // Disable Button when Kinect is not available
                if (null == kinectRegion.KinectSensor || KinectStatus.Connected != kinectRegion.KinectSensor.Status)
                {
                    startFlyingTile.IsEnabled = false;
                }
            }
            else
            {
                refreshCtrl.Visibility = System.Windows.Visibility.Visible;
                if (_isConnecting)
                {
                    startFlyingTile.IsEnabled = false;
                    startFlyingTile.Label = CommonRessource.Connecting;
                    refreshCtrl.IsSpinning = true;
                }
                else
                {
                    startFlyingTile.IsEnabled = true;
                    startFlyingTile.Label = CommonRessource.RetryConnection;
                    refreshCtrl.IsSpinning = false;
                }
            }
        }

        /// <summary>
        /// Initializes the DroneController and sets it on all other Controllers
        /// </summary>
        /// <param name="OnFinished"></param>
        private void InitDroneController(Action OnFinished)
        {
            // check if old event handlers need the be removed
            if (null != _droneController && _droneController.isDisposed)
            {
                _droneController.errorStateData.PropertyChanged -= errorStateData_PropertyChanged;
            }

            if (null == _droneController || _droneController.isDisposed)
            {

                _droneController = new DroneController();
                _window.droneController = _droneController;

                _droneController.errorStateData.PropertyChanged += errorStateData_PropertyChanged;
            }

            if (_droneController.IsConnected)
            {
                Dispatcher.BeginInvoke(OnFinished);
                return;
            }

            VisualizeConnectionState(true);

            Task.Factory.StartNew(() =>
            {
                _droneController.Initialize();
                if (_droneController.IsConnected)
                {
                    if (null != _kinectController)
                    {
                        _kinectController.SetDroneController(_droneController);
                    }

                }

                Dispatcher.BeginInvoke(OnFinished);
            });
        }

        private void errorStateData_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == ErrorStateData.DRONE_CONNECTED)
            {
                ErrorStateData errorState = (ErrorStateData)sender;
                if (errorState.droneConnected)
                {
                    _isConnected = true;
                    _droneController.SetSensivity(_window.droneSensitivity);

                    Dispatcher.BeginInvoke((Action)(() =>
                    {
                        VisualizeConnectionState(false);
                    }));

                }
                else
                {
                    _isConnected = false;
                    Dispatcher.BeginInvoke((Action)(() =>
                    {
                        VisualizeConnectionState(false);
                    }));
                }
            }
        }

        private void Page_Loaded(object sender, RoutedEventArgs e)
        {
            _window.KeyDown += Window_KeyDown;
            _kinectController.SensorChanged += kinectController_SensorChanged;

            this.sensorChooserUi.KinectSensorChooser = _window.kinectSensorChooser;
            kinectRegion.KinectSensor = _kinectController.sensor;

            if (_droneController != null && !_droneController.isDisposed) 
            {
                _droneController.errorStateData.PropertyChanged += errorStateData_PropertyChanged;
            }

            EnsureDroneConnection();
        }


        private void EnsureDroneConnection() 
        {
            if (!this._isConnected)
            {
                AsyncInitDroneControllerAndVisualizeResult();
            }
            else
            {
                VisualizeConnectionState(false);
            }
        }
        /// <summary>
        /// Initializes the DroneController and Visualizes the result
        /// </summary>
        private void AsyncInitDroneControllerAndVisualizeResult()
        {
            VisualizeConnectionState(true);
            InitDroneController(() =>
            {
                VisualizeConnectionState(false);
            });
        }

        private void BtnStartFlyingClicked(object sender, RoutedEventArgs e)
        {
            if (_isConnected)
            {
                if (!_isCalibrated)
                {
                    // Navigate to navigation window
                    this.NavigationService.Navigate(new Calibration(_window, true));
                }
                this.NavigationService.Navigate(new DroneCockpit(_window));
            }
            else
            {
                AsyncInitDroneControllerAndVisualizeResult();
            }
        }

        private void btnCalibrationClicked(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(new Calibration(_window, false));
        }

        private void btnOptionsClicked(object sender, RoutedEventArgs e)
        {
            this.NavigationService.Navigate(new Options(_window));
        }

        private void btnExitClicked(object sender, RoutedEventArgs e)
        {
            Application.Current.Shutdown();
            Environment.Exit(0);
        }

        private void Page_Unloaded(object sender, RoutedEventArgs e)
        {
            _window.KeyDown -= Window_KeyDown;
            _kinectController.SensorChanged -= kinectController_SensorChanged;
            if (_droneController != null) _droneController.errorStateData.PropertyChanged -= errorStateData_PropertyChanged;

            System.GC.Collect();
        }

        private void Window_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (Keyboard.Modifiers == ModifierKeys.Control && e.Key == Key.N)
            {
                if (null != _droneController && !_droneController.isDisposed)
                {
                    _droneController.Disconnect();
                    this.AsyncInitDroneControllerAndVisualizeResult();
                }
            }
        }

        ~MainMenu()
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
                    if (!_droneController.isDisposed) _droneController.Dispose();
                }
                _disposed = true;
            }
        }    }
}