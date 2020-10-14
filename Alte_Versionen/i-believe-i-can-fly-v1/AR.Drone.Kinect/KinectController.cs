using AR.Drone.Client;
using AR.Drone.Client.CalibrationBinding;
using AR.Drone.Control;
using Fuzzy;
using Microsoft.Kinect;
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.Kinect
{
    public class KinectController
    {
        private DroneController _droneController;
        private Calibrater _calibrater;
        private Skeleton[] _skeletons;
        private bool _droneStarting;
        private double[] _upHistory;
        private double[] _rightHistory;
        private double[] _rotateRightHistory;
        private double[] _backHistory;
        private int _historyCounter;
        public KinectSensor sensor { get; private set; }
        public bool onDroneCockpit { get; set; }
        private bool _onCalibrationPage;
        private FuzzyController _fuzzyController;
        private Object lockObject = new Object();
        public CalibrationDataSource calibrationDataSource
        {
            get
            {
                return _calibrater.calibrationDataSource;
            }

            private set 
            { 
                // nothing todo here
            }
        }

        public delegate void NavigateBackEventHandler(object sender, EventArgs e);
        public event NavigateBackEventHandler NavigateBack;

        public delegate void SensorChangedEventHandler(object sender, SensorChangedEventArgs e);
        public event SensorChangedEventHandler SensorChanged;

        public KinectController()
        {
            this._calibrater = new Calibrater(new CalibrationDataSource());
            this._fuzzyController = new FuzzyController();
            this._fuzzyController.InitFIS();
            this._skeletons = new Skeleton[6];
            this._droneStarting = false;
            this._upHistory = new double[Constants.SMOOTH_AVERAGE_COUNT];
            this._rightHistory = new double[Constants.SMOOTH_AVERAGE_COUNT];
            this._rotateRightHistory = new double[Constants.SMOOTH_AVERAGE_COUNT];
            this._backHistory = new double[Constants.SMOOTH_AVERAGE_COUNT];
            for (int i = 0; i < Constants.SMOOTH_AVERAGE_COUNT; ++i)
            {
                this._upHistory[i] = 0;
                this._rightHistory[i] = 0;
                this._rotateRightHistory[i] = 0;
                this._backHistory[i] = 0;
            }
            this._historyCounter = 0;
            this._onCalibrationPage = false;
            this.onDroneCockpit = false;
        }

        public void Initialize(KinectSensor sensor)
        {
            //this.sensor = KinectSensor.KinectSensors.SingleOrDefault();
            this.sensor = sensor;

            if (this.sensor != null)
            {
                this.sensor.AllFramesReady += this.Sensor_AllFramesReady;
                this.sensor.Start();
                if (null != SensorChanged) SensorChanged(this, new SensorChangedEventArgs(sensor));
            }
        }

        public void SetDroneController(DroneController droneController) 
        {
            this._droneController = droneController;
        }

        public bool StopKinect()
        {
            if (null == this.sensor) 
            {
                // Kinect is not connected
                return true;
            } 
            if (this.sensor.SkeletonStream.IsEnabled)
                this.sensor.SkeletonStream.Disable();

            if (this.sensor.ColorStream.IsEnabled)
                this.sensor.ColorStream.Disable();

            if (this.sensor.IsRunning)
            {
                this.sensor.Dispose();
                return true;
            }
            return false;
        }

        private void Sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (var frame = e.OpenSkeletonFrame())
            {
                if (frame != null)
                {
                    frame.CopySkeletonDataTo(this._skeletons);

                    var users = this._skeletons.Where(s => s.TrackingState == SkeletonTrackingState.Tracked).ToList();

                    if (users.Count > 0)
                    {
                        JointCollection joints = users[0].Joints;
                        double difHands     = this.GetDifHandsSignal(joints);
                        double up           = this.GetUpSignal(joints);
                        double right        = this.GetRightSignal(joints);
                        double rotateRight  = this.GetRotateRightSignal(joints);
                        double back         = this.GetBackSignal(joints);
                        this._historyCounter++;

                        if (this._onCalibrationPage)
                        {
                            this.handleCalibrationTask(joints, up);
                        }
                        else if (this.onDroneCockpit)
                        {
                            this.handleDroneTask(difHands, up, right, rotateRight, back);
                        }                    
                    }
                }
            }
        }

        private void handleCalibrationTask(JointCollection joints, double up)
        {
            // Start calibration if user is on Calibration Page and takes calibration position
            if (Math.Abs(up) < Constants.MIN_MOVEMENT && !this._calibrater.hasCalibrationStarted)
            {
                this._calibrater.Start();
            }
            // Continue Calibration
            else if (this._calibrater.hasCalibrationStarted)
            {
                this._calibrater.DoCalibration(joints);
            }
        }

        private void handleDroneTask(double difHands, double up, double right, double rotateRight, double back)
        {

            // Start drone if hands rise
            if (!this._droneController.IsStarted && !this._droneStarting && up > SignalSmoother.SmoothArmValue(Constants.STARTING_TRESHOLD, this._calibrater))
            {
                this.StartDrone();
            }
            // Land drone if hands fall
            else if (up < SignalSmoother.SmoothArmValue(Constants.LANDING_THRESHOLD, this._calibrater))
            {
                this._droneController.Land();
            }

            // Don't send any commands while the drone is starting
            if (this._droneStarting) return;

            //Navigate to Main Menu if hands stick together
            else if (Math.Abs(difHands) < Constants.TOLERANCE_ENTER_HOVER_MODE)
            {
                this._droneController.Hover();
                this.onDroneCockpit = false;
                if (null != NavigateBack)
                    NavigateBack(this, null);
            }
            // Hover drone if default position of skeleton
            else if (Math.Abs(up) < Constants.MIN_MOVEMENT && Math.Abs(right) < Constants.MIN_MOVEMENT && Math.Abs(rotateRight) < Constants.MIN_MOVEMENT && Math.Abs(back) < Constants.MIN_MOVEMENT)
            {
                this._droneController.Hover();
            }
            // Control the drone
            else
            {
                FISResult fuzzyResult = _fuzzyController.DoInference((float)back, (float)right, (float)up, (float)rotateRight);
                
                //Console.WriteLine("--------------------");
                //Console.WriteLine(string.Format("Back: {0}, Right: {1}, Up: {2}, RotateRight: {3}", back, right, up, rotateRight));
                //Console.WriteLine(fuzzyResult);
                //Console.WriteLine("--------------------\n");

                this._droneController.SetProgressivValues(fuzzyResult.sidewardSpeed, fuzzyResult.backwardSpeed, fuzzyResult.upSpeed, fuzzyResult.rotationSpeed);
            }
        }

        private double GetDifHandsSignal(JointCollection joints) 
        {
            return joints[JointType.HandRight].Position.X - joints[JointType.HandLeft].Position.X;
        }

        private double GetUpSignal(JointCollection joints)
        {
            double rawUp = joints[JointType.HandLeft].Position.Y - joints[JointType.ShoulderLeft].Position.Y + joints[JointType.HandRight].Position.Y - joints[JointType.ShoulderRight].Position.Y;
            return SignalSmoother.SmoothArmValue(rawUp, this._calibrater);
            //this._upHistory[this._historyCounter % Constants.SMOOTH_AVERAGE_COUNT] = smoothedValue;

            //double sumHistoryValues = 0.0;
            //foreach(double value in this._upHistory) 
            //    sumHistoryValues += value;

            //return sumHistoryValues / Constants.SMOOTH_AVERAGE_COUNT;
        }

        private double GetRightSignal(JointCollection joints)
        {
            double rawRight = joints[JointType.HandLeft].Position.Y - joints[JointType.HandRight].Position.Y;
            return SignalSmoother.SmoothArmValue(rawRight, this._calibrater);
            //this._rightHistory[this._historyCounter % Constants.SMOOTH_AVERAGE_COUNT] = smoothedValue;

            //double sumHistoryValues = 0.0;
            //foreach (double value in this._rightHistory)
            //    sumHistoryValues += value;

            //return sumHistoryValues / Constants.SMOOTH_AVERAGE_COUNT;
        }

        private double GetRotateRightSignal(JointCollection joints)
        {
            double rawRotateRight = joints[JointType.HandRight].Position.Z - joints[JointType.HandLeft].Position.Z;
            return SignalSmoother.SmoothArmValue(rawRotateRight, this._calibrater);
            //this._rotateRightHistory[this._historyCounter % Constants.SMOOTH_AVERAGE_COUNT] = smoothedValue;

            //double sumHistoryValues = 0.0;
            //foreach (double value in this._rotateRightHistory)
            //    sumHistoryValues += value;

            //return sumHistoryValues / Constants.SMOOTH_AVERAGE_COUNT;
        }

        private double GetBackSignal(JointCollection joints)
        {
            double rawBack = joints[JointType.ShoulderCenter].Position.Z - joints[JointType.HipCenter].Position.Z;
            return SignalSmoother.SmoothTiltValue(rawBack, this._calibrater);

            //double smoothedValue = backSignal;
            //this._backHistory[this._historyCounter % Constants.SMOOTH_AVERAGE_COUNT] = smoothedValue;

            //double sumHistoryValues = 0.0;
            //foreach (double value in this._backHistory)
            //    sumHistoryValues += value;

            //return sumHistoryValues / Constants.SMOOTH_AVERAGE_COUNT;
        }

        private void StartDrone()
        {
            lock (lockObject) { 
                if (this._droneStarting || this._droneController.BatteryLow) return;
            }
            Stopwatch watch = new Stopwatch();
            watch.Start();

            this._droneStarting = true;
            
            this._droneController.RecoverFromEmergency();
            
            this._droneController.Trim();

            Task.Factory.StartNew(() =>
            {
                while (!_droneController.IsStarted && watch.ElapsedMilliseconds < 1000)
                {
                    _droneController.TakeOff();
                    Thread.Sleep(100);
                }
                watch.Stop();

                // wait until the drone has reached its startup height
                if (_droneController.IsStarted) Thread.Sleep(5000);
                
                this._droneStarting = false;
            });
           
        }

        public void ActivateCalibration()
        {
            this._onCalibrationPage = true;
        }

        public void DeactivateCalibration()
        {
            this._onCalibrationPage = false;
            this._calibrater.hasCalibrationFinished = false;
        }

        public bool HasCalibrationFinished()
        {
            return this._calibrater.hasCalibrationFinished;
        }

        public bool HasCalibrationStarted()
        {
            return this._calibrater.hasCalibrationStarted;
        }
    }
}