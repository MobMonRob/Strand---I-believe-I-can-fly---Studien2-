using AR.Drone.Configuration;
using AR.Drone.Control.Command;
using AR.Drone.Control.Command.Mode;
using AR.Drone.NavData;
using AR.Drone.NavData.Data;
using AR.Drone.NavData.Wifi;
using AR.Drone.Util;
using AR.Drone.Video;
using AR.Drone.Video.Data;
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.Control
{
    public class DroneController : IDisposable
    {
        public bool isDisposed { get; private set; }
        private Object lockObject = new Object();
        internal bool shutingDown { get; private set; }
        public ErrorStateData errorStateData { get; internal set; }

        internal WifiReceiver wifiReceiver { get; private set; }
        private CommandSender _commandSender;
        private ConfigurationReceiver _configReceiver;
        private bool _configReady = false;
        private NavdataAcquisition _navdataReceiver;
        public NavigationData Navdata { get; private set; }

        private Object _emergencyLock = new Object();
        private bool _IsRecoveringFromEmergency;

        private VideoReceiver _videoReceiver;
        private VideoFrameDecoder _videoFrameDecoder;
        private BlockingCollection<VideoFrame> _videoFrameQueue;

        public bool IsConnected { 
            get {
                return this.errorStateData.droneConnected;
            }
        }

        public DroneConfiguration droneConfiguration { get; private set; }

        // Events
        public delegate void FrameReadyEventHandler(object sender, VideoFrameReadyEventArgs e);
        public event FrameReadyEventHandler FrameReady;

        public DroneController()
        {
            this.errorStateData = new ErrorStateData();
            this.isDisposed = false;
            //WifiReceiver.ConnectToWifiNetwork();
        }

        public void Initialize()
        {
            try
            {
                EnsureActiveWifiReceiver();  

                EnsureValidCommandSender();
                SendInitCommands();

                // Init configuration
                EnsureDroneConfiguration();
                EnsureValidConfigReceiver();
                SyncConfigurationFetch();

                // Init Navigation data          
                EnsureNavigationDataStream();

                EnsureActiveVideoStream();

                // Ensure no Emergency mode is active
                RecoverFromEmergency();

                this.errorStateData.droneConnected = true;
            }
            catch (TimeoutException e)
            {
                Console.Write("Drone Connection Timeout: " + e.Message);
                this.HandleDroneConnectionLost();
            }

        }

        private void EnsureDroneConfiguration()
        {
            if (null == droneConfiguration)
            {
                droneConfiguration = new DroneConfiguration(ref this._commandSender);
            }
        }

        /// <summary>
        /// Ensures a valid WifiReceiver instance
        /// </summary>
        private void EnsureActiveWifiReceiver()
        {
            if (!IsValidWorker(wifiReceiver))
            {
                wifiReceiver = new WifiReceiver(this);
                wifiReceiver.StartAsThread();
                //wifiReceiver.TryConnectToNetwork();
            }
            
        }

        /// <summary>
        /// Sends the commands for initializing the drone
        /// </summary>
        private void SendInitCommands()
        {
            _commandSender.AddCommand(new MISCCommand());
            _commandSender.AddCommand(BasicCommand.LAND_COMMAND);
        }

        /// <summary>
        /// Ensures a valid command sender instance.
        /// If no instance exists a new instance is created
        /// </summary>
        private void EnsureValidCommandSender()
        {
            if (!IsValidWorker(_commandSender))
            {
                _commandSender = new CommandSender(DroneConstants.IP_ADDRESS, DroneConstants.COMMAND_PORT);
                _commandSender.WorkFinished += _commandSender_WorkFinished;
                _commandSender.StartAsThread();
            }
        }

        /// <summary>
        /// HAndler for the Workfinish-Event of the command Sender
        /// Called when the Work of the CommandSender is finished 
        /// </summary>
        /// <param name="sender">The sender</param>
        /// <param name="e">Workfinished Events holding the possible Error</param>
        void _commandSender_WorkFinished(object sender, WorkFinishedEventArgs e)
        {
            if (!e.successfull && null != e.exception)
            {
                if (e.exception.GetType() == typeof(SocketException))
                {
                    // Drone connection lost
                    HandleDroneConnectionLost();
                }
                else
                {
                    // something else happened -> Try again
                    EnsureValidCommandSender();
                }
            }
        }

        private void EnsureNavigationDataStream()
        {
            if (!IsValidWorker(_navdataReceiver))
            {
                _navdataReceiver = new NavdataAcquisition(this);
                _navdataReceiver.WorkFinished += _navdataReceiver_WorkFinished;
                _navdataReceiver.StartAsThread();

                Navdata = _navdataReceiver.GetNavigationData();

                droneConfiguration.general.navdata_demo = "TRUE";
            }
        }

        /// <summary>
        /// Handler for the WorkFinish-Event of the NavdataReceiver.
        /// Called when the work of the NavdataReceiver is finished.
        /// </summary>
        /// <param name="sender">The sender</param>
        /// <param name="e"></param>
        void _navdataReceiver_WorkFinished(object sender, WorkFinishedEventArgs e)
        {
            if (!e.successfull && null != e.exception)
            {
                Console.WriteLine(e.exception);
                if (e.exception.GetType() == typeof(SocketException))
                {
                    // Drone connection lost
                    HandleDroneConnectionLost();
                }
                else
                {
                    // something else happened -> Try again
                    errorStateData.navigationDataLost = true;
                    EnsureNavigationDataStream();
                }
            }
        }

        /// <summary>
        /// Ensures a valid config receiver instance.
        /// If no instance exists a new instance is created
        /// </summary>
        private void EnsureValidConfigReceiver()
        {
            if (!IsValidWorker(_configReceiver))
            {
                _configReceiver = new ConfigurationReceiver(_commandSender, droneConfiguration);
                _configReceiver.WorkFinished += _configReceiver_WorkFinished;
                _configReceiver.StartAsThread();
            }
        }

        /// <summary>
        /// Handler for the WorkFinish-Event of the ConfigReceiver.
        /// Called when the work of the ConfigReceiver is finished.
        /// </summary>
        /// <param name="sender">The sender</param>
        /// <param name="e"></param>
        void _configReceiver_WorkFinished(object sender, WorkFinishedEventArgs e)
        {
            if (!e.successfull && null != e.exception)
            {
                Console.WriteLine(e.exception);
                if (e.exception.GetType() == typeof(SocketException))
                {
                    // Drone connection lost
                    HandleDroneConnectionLost();
                }
                else
                {
                   // something else happened -> Try again
                    EnsureValidConfigReceiver();
                }
            }
        }

        /// <summary>
        /// This method should be called, when the connection to the drone is lost.
        /// Results in disposing the DroneController
        /// </summary>
        private void HandleDroneConnectionLost()
        {
            lock (lockObject)
            {
                if (shutingDown) { return; }
                shutingDown = true;
            }

            errorStateData.droneConnected = false;
            this.Dispose(true);
        }

        /// <summary>
        /// Checks if the given worker object is valid. (This means not null, alive and no stop is requested)
        /// </summary>
        /// <param name="worker">The worker to be check</param>
        /// <returns>Whether the worker is valid or not</returns>
        private bool IsValidWorker(Worker worker)
        {
            return (null != worker && worker.IsAlive() && !worker.IsStopRequested());
        }

        /// <summary>
        /// Activates the video stream
        /// </summary>
        public void EnsureActiveVideoStream()
        {
            droneConfiguration.video.video_codec = (int)VideoCodec.H264_720P_CODEC;
            if (null == _videoFrameQueue)
            {
                _videoFrameQueue = new BlockingCollection<VideoFrame>();
            }

            if (!IsValidWorker(_videoReceiver))
            {
                _videoReceiver = new VideoReceiver(_videoFrameQueue);
                _videoReceiver.WorkFinished += _videoReceiver_WorkFinished;
                _videoReceiver.StartAsThread();
            }

            if (!IsValidWorker(_videoFrameDecoder))
            {
                _videoFrameDecoder = new VideoFrameDecoder(_videoFrameQueue);
                _videoFrameDecoder.FrameReady += ForwardFrameReady;
                _videoFrameDecoder.WorkFinished += _videoFrameDecoder_WorkFinished;
                _videoFrameDecoder.StartAsThread();
            }
        }

        /// <summary>
        /// Handler for the WorkFinish-Event of the VideoFrameDecoder.
        /// Called when the work of the VideoFrameDecoder is finished.
        /// </summary>
        /// <param name="sender">The sender</param>
        /// <param name="e"></param>
        void _videoFrameDecoder_WorkFinished(object sender, WorkFinishedEventArgs e)
        {
            if (!e.successfull && null != e.exception)
            {
                Console.WriteLine(e.exception);
                if (e.exception.GetType() == typeof(FFmpegException))
                {
                    errorStateData.ffmpegError = true;
                    errorStateData.ffmpegErrorString = e.exception.Message;
                }
                else
                {
                    // something else happened -> Try again
                    EnsureActiveVideoStream();
                }
            }
        }

        /// <summary>
        /// Handler for the WorkFinish-Event of the VideoReceiver.
        /// Called when the work of the VideoReceiver is finished.
        /// </summary>
        /// <param name="sender">The sender</param>
        /// <param name="e"></param>
        void _videoReceiver_WorkFinished(object sender, WorkFinishedEventArgs e)
        {
            if (!e.successfull && null != e.exception)
            {
                Console.WriteLine(e.exception);
                if (e.exception.GetType() == typeof(SocketException))
                {
                    // Drone connection lost
                    HandleDroneConnectionLost();
                }
                else
                {
                    // something else happened -> Try again
                    EnsureActiveVideoStream();
                }
            }
        }

        /// <summary>
        /// Forwards the FrameReady-Event from the VideoFrameDecoder to the EventListners on this object
        /// </summary>
        /// <param name="sender">The sender</param>
        /// <param name="e">The arguments</param>
        private void ForwardFrameReady(object sender, VideoFrameReadyEventArgs e)
        {
            if (null != FrameReady)
            {
                this.FrameReady(sender, e);
            }
        }

        /// <summary>
        /// Tries to recover from emergency.
        /// </summary>
        /// <exception cref="TimeoutException">The maximum waiting time exceeded</exception>
        public void RecoverFromEmergency()
        {
            if (null == Navdata) return;

            lock (_emergencyLock)
            {
                if (!Navdata.IsEmergency() || this._IsRecoveringFromEmergency)
                    return;

                _IsRecoveringFromEmergency = true;
            }

            var timeoutWatch = Stopwatch.StartNew();

            Task.Factory.StartNew(() =>
            {
                while (Navdata.IsEmergency())
                {
                    if (timeoutWatch.ElapsedMilliseconds > DroneConstants.DRONE_CONTROLLER_EMERGENCY_TIMEOUT)
                    {
                        this.HandleDroneConnectionLost();
                        //throw new TimeoutException("Could not recover from emergency");
                    }
                    _commandSender.AddCommand(BasicCommand.EMERGENCY_COMMAND);
                    Thread.Sleep(1000);
                    _IsRecoveringFromEmergency = false;
                }
            });         
        }

        /// <summary>
        /// Makes a request for fetching the current configuration and waits for the answer
        /// </summary>
        /// <exception cref="TimeoutException">The maximum waiting time exceeded</exception>
        private void SyncConfigurationFetch()
        {
            var watch = Stopwatch.StartNew();
            _configReceiver.GetConfiguration((DroneConfiguration) =>
            {
                _configReady = true;
            });

            while (!_configReady)
            {
                if (watch.ElapsedMilliseconds > DroneConstants.DRONE_CONTROLLER_CONFIG_FETCH_TIMEOUT)
                {
                    throw new TimeoutException("Config fetch timed out");
                }
                Thread.Sleep(10);
            }
        }

        /// <summary>
        /// Sends the command for take off
        /// </summary>
        public void TakeOff()
        {
            _commandSender.AddCommand(BasicCommand.TAKE_OFF_COMMAND);
        }

        /// <summary>
        /// Sends the command for landing the drone
        /// </summary>
        public void Land()
        {
            _commandSender.AddCommand(BasicCommand.LAND_COMMAND);
        }

        /// <summary>
        /// Shuts down the connection to the drone
        /// </summary>
        public void Disconnect()
        {  
            HandleDroneConnectionLost();
        }

        /// <summary>
        /// Stops the worker thread if necessary and waits for the termination
        /// </summary>
        /// <param name="worker">The worker to be stop</param>
        private void SyncStopWorkerIfNecessary(Worker worker)
        {
            if (worker != null && !worker.IsAlive())
            {
                worker.RequestStop();
                worker.Join();
            }
        }

        /// <summary>
        /// Sends the command for trimming the drone
        /// </summary>
        public void Trim()
        {
            _commandSender.AddCommand(new TrimCommand());
        }

        /// <summary>
        /// Sets the progressiv values as percentage of the total value 
        /// </summary>
        /// <param name="leftRightTilt">Percentage of the maximum inclination. A negativ value makes it fly leftward and a positiv value rightward </param>
        /// <param name="frontBackTilt">Percentage of the maximum inclination. A negativ value makes it fly forward and a positiv value backward</param>
        /// <param name="verticalSpeed">Percentage of the maximum vertical speed. A negativ value makes it go down and a positiv value makes it rise in the air</param>
        /// <param name="angularSpeed">Percentage of the maximum angular speed. A negativ value makes it spin left and a positiv makes it spin right</param>
        public void SetProgressivValues(float leftRightTilt, float frontBackTilt, float verticalSpeed, float angularSpeed)
        {
            var progressivCommand = new ProgressivCommand(ProgressiveCommandMode.PROGRESSIV)
            {
                leftRightTilt = leftRightTilt,
                frontBackTilt = frontBackTilt,
                verticalSpeed = verticalSpeed,
                angularSpeed = angularSpeed
            };
            Console.WriteLine("left: {0}; front: {1}; verticalSpeed: {2}; angularSpeed: {3}", leftRightTilt, frontBackTilt, verticalSpeed, angularSpeed);
            _commandSender.AddCommand(progressivCommand);
        }

        /// <summary>
        /// Sends the command for hovering
        /// </summary>
        public void Hover()
        {
            var progressivCommand = new ProgressivCommand(ProgressiveCommandMode.HOVER);

            _commandSender.AddCommand(progressivCommand);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="frontBackTilt"></param>
        public void SetFrontBackTilt(float frontBackTilt)
        {
            SetProgressivValues(0.0f, frontBackTilt, 0.0f, 0.0f);
        }

        /// <summary>
        /// Lets the drone fly forward with the given strength.
        /// The strength must be a value between 0 and 1. It represents the percentage of the maximum inclination
        /// </summary>
        /// <param name="strength">The percentage of the maximum inclination (between 0 and 1)</param>
        public void Forward(float strength)
        {
            SetFrontBackTilt(-1 * Math.Abs(strength));
        }

        /// <summary>
        /// Lets the drone fly backward with the given strength.
        /// The strength must be a value between 0 and 1. It represents the percentage of the maximum inclination
        /// </summary>
        /// <param name="strength"></param>
        public void Backward(float strength)
        {
            SetFrontBackTilt(Math.Abs(strength));
        }

        /// <summary>
        /// Switches the video stream to the front camera
        /// </summary>
        public void ShowFrontCamera()
        {
            droneConfiguration.video.video_channel = 0;
        }

        /// <summary>
        /// Switches the video stream to the bottom camera
        /// </summary>
        public void ShowBottomCamera()
        {
            droneConfiguration.video.video_channel = 3;
        }

        /// <summary>
        /// Sets the sensivity of the drone by setting the maxium speed values.
        /// </summary>
        /// <param name="sensivity">The new sensivity. Must be in the range of <code>DroneConstants.DRONE_SENSIVITY_MIN</code> and <code>DroneConstants.DRONE_SENSIVITY_MAX</code></param>
        public void SetSensivity(int sensivity) 
        {
            if (null == droneConfiguration) return;

            sensivity = Math.Min(sensivity, DroneConstants.DRONE_SENSIVITY_MAX);
            sensivity = Math.Max(sensivity, DroneConstants.DRONE_SENSIVITY_MIN);

            // calculate the percentage of the maximum sensivity
            float sensivityPercentage = (float) sensivity / DroneConstants.DRONE_SENSIVITY_DELTA;

            int newMaxVerticalSpeed = (int) (DroneConstants.DRONE_MAX_VERTICAL_SPEED_DELTA * sensivityPercentage + DroneConstants.DRONE_MAX_VERTICAL_SPEED_MIN);
            float newMaxYawSpeed = DroneConstants.DRONE_MAX_YAW_SPEED_DELTA * sensivityPercentage + DroneConstants.DRONE_MAX_YAW_SPEED_MIN;
            float newMaxPitchRollBendingAngle = DroneConstants.DRONE_MAX_PITCH_ROLL_BENDING_ANGLE_DELTA * sensivityPercentage + DroneConstants.DRONE_MAX_PITCH_ROLL_BENDING_ANGLE_MIN;

            droneConfiguration.control.control_vz_max = newMaxVerticalSpeed;
            droneConfiguration.control.control_yaw = newMaxYawSpeed;
            droneConfiguration.control.euler_angle_max = newMaxPitchRollBendingAngle;
        }

        ~DroneController()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!isDisposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    DisposeIfNotNull(_commandSender);
                    DisposeIfNotNull(_configReceiver);
                    DisposeIfNotNull(_navdataReceiver);
                    DisposeIfNotNull(wifiReceiver);

                    if (null != _videoFrameDecoder)
                    {
                        _videoFrameDecoder.Dispose();
                        _videoFrameDecoder.Join();
                    }
                    if (null != _videoReceiver)
                    {
                        _videoReceiver.Dispose();
                        _videoReceiver.Join();
                    }

                    DisposeIfNotNull(_videoFrameQueue);
                }
                isDisposed = true;
            }
        }

        /// <summary>
        /// Disposes the object if it is not null
        /// </summary>
        /// <param name="disposable"></param>
        private void DisposeIfNotNull(IDisposable disposable)
        {
            if (null != disposable)
            {
                disposable.Dispose();
            }
        }

        public bool IsStarted { get { return !this.Navdata.IsLanded(); } }

        public bool BatteryLow { get { return this.Navdata.BatteryLow; } }
    }
}