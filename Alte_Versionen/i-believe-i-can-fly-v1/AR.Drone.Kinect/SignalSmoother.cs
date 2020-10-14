using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Kinect
{
    class SignalSmoother
    {
        public static double SmoothArmValue(double value, Calibrater cali) 
        {
            // Standardize values for every arm length
            value /= cali.armLength;
            return GetReturnValue(value);
        }

        public static double SmoothTiltValue(double value, Calibrater cali)
        {
            // Standardize for back tilt
            value -= cali.backTilt;
            return GetReturnValue(value);
        }

        private static double GetReturnValue(double value)
        {

            if (value <= -1.0)
                return -1.0;

            if (value >= 1.0)
                return 0.99;

            return value;
        }
    }
}
