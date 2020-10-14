using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AR.Drone.Kinect
{
    public class SensorChangedEventArgs
    {
        public KinectSensor sensor { get; private set; }

        public SensorChangedEventArgs(KinectSensor sensor)
        {
            this.sensor = sensor;
        }
    }
}
