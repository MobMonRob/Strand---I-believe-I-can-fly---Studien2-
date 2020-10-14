﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media;

namespace AR.Drone.Kinect
{
    class Constants
    {
        /// <summary>
        /// IP for Kinect socket
        /// </summary>
        public const string KINECT_SOCKET_IP = "127.0.0.1";

        /// <summary>
        /// Port for Kinect socket
        /// </summary>
        public const string KINECT_SOCKET_PORT = "8181";

        /// <summary>
        /// IP for Drone socket
        /// </summary>
        public const string DRONE_SOCKET_IP = "127.0.0.1";

        /// <summary>
        /// Port for Drone socket
        /// </summary>
        public const string DRONE_SOCKET_PORT = "8282";

        /// <summary>
        /// DPI.
        /// </summary>
        public const double DPI = 96.0;

        /// <summary>
        /// Default name for temporary color files.
        /// </summary>
        public const string CAPTURE_FILE_COLOR = "Capture_Color.jpg";

        /// <summary>
        /// The pixel format.
        /// </summary>
        public static readonly PixelFormat PIXEL_FORMAT = PixelFormats.Bgra32;

        /// <summary>
        /// The upper threshold for hovering
        /// </summary>
        public const double MIN_MOVEMENT = 0.05;

        /// <summary>
        /// The upper threshold for for movement signaling
        /// </summary>
        public const double SMOOTH_LIMIT = 1.0;

        /// <summary>
        /// The start point of the smoothing function
        /// </summary>
        public const double SMOOTH_START = 0.01;

        /// <summary>
        /// The lower threshold for hovering
        /// </summary>
        public const int SMOOTH_FACTOR = 6;

        /// <summary>
        /// The average of n values of signals (n::= SMOOTH_AVERAGE_COUNT)
        /// </summary>
        public const int SMOOTH_AVERAGE_COUNT = 10;

        /// <summary>
        /// The lower threshold for landing
        /// </summary>
        public const double LANDING_THRESHOLD = -0.8;

        /// <summary>
        /// The lower threshold for starting
        /// </summary>
        public const double STARTING_TRESHOLD = 0.2;

        /// <summary>
        /// The lower threshold for starting
        /// </summary>
        public const double CALIBRATE_LIMIT = 100;

        /// <summary>
        /// The tolerance area for good arm measurement
        /// </summary>
        public const double TOLERANCE_CALIBRATING = 0.2;

        /// <summary>
        /// The maximal distance for the hands to enter hover mode if hands stick together
        /// </summary>
        public const double TOLERANCE_ENTER_HOVER_MODE = 0.05;

        /// <summary>
        /// The tolerance area for starting calibration
        /// </summary>
        public const double TOLERANCE_START_CALIBRATION = 0.3;

        /// <summary>
        /// The minimal threshold for exiting hover mode
        /// </summary>
        public const double TOLERANCE_EXIT_HOVER = 0.2;

        /// <summary>
        /// The factor for strengthen the tilt signal
        /// </summary>
        public const double TILT_FACTOR = 5.0;

        /// <summary>
        /// The factor for strengthen the back tilt signal when tilting back, because it is harder than tilting front
        /// </summary>
        public const double BACK_TILT_FACTOR = 4.0;

        /// <summary>
        /// The upper threshold for sensitivity calibration
        /// </summary>
        public const double SENSITIVITY_MAX = 15;

        /// <summary>
        /// The lower threshold for sensitivity calibration
        /// </summary>
        public const double SENSITIVITY_MIN = 1;

        /// <summary>
        /// Miliseconds to wait for next action
        /// </summary>
        public const double MIN_WAIT_TIME = 400;

        /// <summary>
        /// The tolerance area for good arm measurement when calibrating sensitivity
        /// </summary>
        public const double SENSITIVITY_TOLERANCE = 0.2;
    }
}
