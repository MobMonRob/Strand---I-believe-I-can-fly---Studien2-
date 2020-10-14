using AR.Drone.Control;
using System;
using System.Text;
using System.Threading;

namespace AR.Drone.Headtracker
{
    /// <summary>
    /// The Headtrackercontroller provides an interface for the Dronecontroller to handle the information received
    /// from the cinemizer Headtracker
    /// </summary>
    public class HeadTrackerController
    {
        private bool _stopRequested = false;
        private bool _trackerInitiliazed = false;
        private USED_CAMERA _usedCamera = USED_CAMERA.INITIAL;

        private Thread _trackerThread; 
        private DroneController _droneController;

        public HeadTrackerErrorState errorState { get; internal set; } 

        /// <summary>
        /// Constructor for the HeadtrackerController
        /// </summary>
        /// <param name="droneController">reference to the Drone Controller</param>
        public HeadTrackerController()
        {
            this.errorState = new HeadTrackerErrorState(false, "");
        }

        /// <summary>
        /// Request to stop receiving Headtracking data and to shutdown the Headtracker Thread.
        /// </summary>
        public void RequestStop()
        {
            this._stopRequested = true;
        }

        /// <summary>
        /// Initialize Headtracker and configure basic settings
        /// </summary>
        /// <returns>true if initialization succeedes, false otherwise. The errorState object contains occured errors</returns>
        public bool InitializeTracker()
        {
            if (this._trackerInitiliazed)
            {
                this.errorState.hasError = false;
                this.errorState.stateText = "";
                return true;
            }
            StringBuilder errorString = new StringBuilder();
            var bResult = Tracker.Init();
            var iResult = Tracker.GetLastError();
            errorString.Append(Tracker.GetErrorString(errorString, iResult));
            if (!bResult)
            {
               
                this.errorState.hasError = true;
                this.errorState.stateText = errorString.ToString();
                return false;
            }

            Tracker.SetBootloaderMode(false);          

            this._trackerInitiliazed = true;
            if (!errorState.hasError)
            {
                this.errorState.hasError = false;
                this.errorState.stateText = "";
            }
            // Turn of mouse emulation
            Tracker.SetMouseSpeed(0);
            //Tracker.CinemizerApplyAutoRollAdjust();
            Tracker.CinemizerApplyAutoYawAdjust();            
            return true;
        }

        /// <summary>
        /// Start thread for receiving and processing the headtracking data
        /// </summary>
        public void StartAsThread()
        {
            this._stopRequested = false;
            _trackerThread = new Thread(TrackerReceiver);
            _trackerThread.Start();
        }

        /// <summary>
        /// destructor to release the Headtracker hardware
        /// </summary>
         ~HeadTrackerController()
        {
            if (this._trackerInitiliazed)
            {
                var result = Tracker.Release();
                if (result) this._trackerInitiliazed = false;
            }
        }

        /// <summary>
        /// Thread Method for receiving continous Headtracking data and processing them.
        /// </summary>
        private void TrackerReceiver() {
            
            while (!this._stopRequested)
            {
                if (Tracker.WaitNextFrame())
                {
                    Thread.Sleep(10);
                   
                }
                Tracker.Frame frame = new Tracker.Frame();
                Tracker.Euler euler = new Tracker.Euler();
                Tracker.Frame cinemizerFrame = new Tracker.Frame();
                Tracker.Euler cinemizerEuler = new Tracker.Euler();
                bool gotFrame;
                if (gotFrame = Tracker.GetFrame(ref frame))
                {
                    Tracker.QuatGetEuler(ref euler, frame.Rot);
                    cinemizerFrame = frame;
                    Tracker.GetCinemizerRotatedFrame(ref cinemizerFrame);
                    Tracker.QuatGetEuler(ref cinemizerEuler, cinemizerFrame.Rot);
                    ProcessPitchRotation(cinemizerFrame);
                    Tracker.CinemizerApplyAutoYawAdjust(); 
                }

                int State = 0;
                Tracker.GetState(ref State);
                bool isConnected = gotFrame && State == 2;
                if (!isConnected)
                {
                    break;
                }
                
            }
            var result = Tracker.Release();
            if (result) this._trackerInitiliazed = false;
        }

        /// <summary>
        /// Method for proseccing the Pitch Rotation of the HEadtracker to change the used Camera of the drone.
        /// </summary>
        /// <param name="frame"></param>
        private void ProcessPitchRotation(Tracker.Frame frame)
        {

            //Console.WriteLine(String.Format("X:{0:0.00}, Y:{1:0.00}, Z:{2:0.00}", frame.Rot.v.x, frame.Rot.v.y, frame.Rot.v.z));

            //TODO: extract constant
            //TODO: add a variable for bottom / front, to prevent the camera from jumping around when the angle is near to -0.30
            if (frame.Rot.v.x < -0.20)
            {
                if (_usedCamera != USED_CAMERA.BOTTOM_CAMERA)
                {
                    this._droneController.ShowBottomCamera();
                    this._usedCamera = USED_CAMERA.BOTTOM_CAMERA;
                    //Console.WriteLine("BOTTOM");
                }                
            }
            else
            {
                if (_usedCamera != USED_CAMERA.FRONT_CAMERA)
                {
                    this._droneController.ShowFrontCamera();
                    this._usedCamera = USED_CAMERA.FRONT_CAMERA;
                    //Console.WriteLine("FRONT");
                }
                
            }
        }

        /// <summary>
        /// Waits for the Thread to end
        /// </summary>
        /// <param name="millis"> time to wait in milliseconds</param>
        /// <returns>true if the Thread has ended within the given time, false otherwise</returns>
        public bool Join(int millis)
        {
            if (null != _trackerThread) return this._trackerThread.Join(millis);
            return true;
        }

        /// <summary>
        /// Waits for the Thread to end
        /// </summary>
        public void Join()
        {
            if (null != _trackerThread) this._trackerThread.Join();
        }

        public void SetDroneController(DroneController droneController)
        {
            this._droneController = droneController;
        }
    }
}
