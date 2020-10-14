using AR.Drone.Control;
using System;
using System.Text;
using System.Threading;

namespace AR.Drone.Headtracker
{
    class Program
    {
        private static bool _ThreadInterupted = false;
        private static Thread _trackerThread;

        private static void TrackerThread()
        {
            Console.WriteLine("Thread running");
            // Turn of mouse emulation
            Tracker.SetMouseSpeed(0);
            while (!_ThreadInterupted)
            {
                if (Tracker.WaitNextFrame()) { 
                    Thread.Sleep(10); 
                }

                Tracker.Frame frame = new Tracker.Frame();
                Tracker.Euler euler = new Tracker.Euler();

                if (Tracker.GetFrame(ref frame))
                {
                    Tracker.QuatGetEuler(ref euler, frame.Rot);
                    Console.WriteLine(String.Format("{0:0.0}", euler.Yaw / Math.PI * 180));
                }
            }
            Console.WriteLine("Thread terminated gracefully");
        }


        private static void InitializeTracker() 
        {
            // From C# example of Inreal Technologies

            StringBuilder trackerErrorString = new StringBuilder(); ;

            Console.WriteLine("TrackerInit ... ");
            var bResult = Tracker.Init();
            var iResult = Tracker.GetLastError();
            Tracker.GetErrorString(trackerErrorString, iResult);
            Console.WriteLine(String.Format(" [{0}] [{1}]\n", bResult, trackerErrorString));
            if (!bResult) return;

            Console.WriteLine("TrackerSetBootloaderMode(false) ... ");
            bResult = Tracker.SetBootloaderMode(false);
            iResult = Tracker.GetLastError();
            Tracker.GetErrorString(trackerErrorString, iResult);
            Console.WriteLine(String.Format(" [{0}] [{1}]\n", bResult, trackerErrorString));

            _trackerThread = new Thread(TrackerThread);
            _trackerThread.Start();
        }

        static void Main(string[] args)
        {
            /*InitializeTracker();
            
            Thread.Sleep(40000);

            _ThreadInterupted = true;
            if (null != _trackerThread)
            {
                _trackerThread.Interrupt();
                _trackerThread.Join();
            }

            Tracker.Release();*/

            DroneController c = new DroneController();
            HeadTrackerController h = new HeadTrackerController();
            h.InitializeTracker();
            Console.WriteLine(h.errorState.hasError + ": " + h.errorState.stateText);
          
                h.StartAsThread();
                if (h.Join(1000))
                {
                    Console.WriteLine("Fehler bei der initialisierung");
                    Console.WriteLine(h.errorState.hasError + ": " + h.errorState.stateText);
                }
                Console.ReadKey();
                h.RequestStop();
                h.Join();
            
        }
    }
}