
using AR.Drone.Configuration;
using AR.Drone.Control;
using AR.Drone.NavData.Wifi;
using AR.Drone.Util;
using AR.Drone.Video;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone
{
    class Program
    {
        /*public static void ExceptionDemo()
        {
            var sender = new CommandSender("192.168.1.1", 1234);
            var confReceiver = new ConfigurationReceiver(sender);

            confReceiver.StartAsThread();
            confReceiver.WorkFinished += confReceiver_WorkFinished;
            confReceiver.GetConfiguration((DroneConfiguration) =>
            {
                Console.WriteLine("Config received. This should not happen");
            });
            Console.ReadKey();
        }*/

        static void confReceiver_WorkFinished(object sender, WorkFinishedEventArgs e)
        {
            Console.WriteLine(e.successfull);
            Console.WriteLine(e.exception);
        }

       /* public static void CombinedDemo()
        {
            var droneController = new DroneController();

            // set some config
            var droneConfig = droneController.droneConfiguration;
            droneConfig.control.outdoor = "FALSE";
            droneConfig.control.flight_without_shell = "FALSE";

            Thread.Sleep(100);
            droneController.Trim();

            var kinectController = new KinectController(droneController);

            Console.WriteLine("Program running!");
            bool run = true;

            while (run)
            {
                switch (Console.ReadKey().KeyChar)
                {
                    case 'f':
                        droneController.ShowFrontCamera();
                        break;

                    case 'b':
                        droneController.ShowBottomCamera();
                        break;

                    case 'c':
                        run = false;
                        break;
                }
            }

            droneController.Dispose();
            kinectController.StopKinect();
        }*/

        public static void NavdataTest()
        {
            DroneController controller = new DroneController();
        }

        static void Main(string[] args)
        {
            //CombinedDemo();
            //NavdataTest();
            //WifiTest();
            using (var droneController = new DroneController())
            {
                droneController.Initialize();

                droneController.TakeOff();
                droneController.TakeOff();

                Thread.Sleep(2000);
                droneController.Land();
                droneController.Land();

            }
            Thread.Sleep(3000);
        }

    }
}
