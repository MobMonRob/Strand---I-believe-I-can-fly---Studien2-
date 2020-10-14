using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone
{
    public class DroneConstants
    {
        /*------------------Network Configuration------------------*/
        public const int NAVDATA_PORT = 5554;
        public const int COMMAND_PORT = 5556;
        public const int CONTROL_PORT = 5559;
        public const int VIDEO_STREAM_PORT = 5555;
        public const int VIDEO_RECORDING_PORT = 5553;

        public const string IP_ADDRESS = "192.168.1.1";
        public const string IP_ADDRESS_MOCK = "127.0.0.1";

        /*-------------------Config Receiver ---------------------*/
        public const int CONFIG_RECEIVER_MAX_BUFFER_SIZE = 2048;
        public const int CONFIG_RECEIVER_MAX_CONNECTION_ATTEMPS = 10;
        public const int CONFIG_RECEIVER_CONNECTION_TIMEOUT = 1;
        public const long CONFIG_RECEIVER_TIMEOUT = 10000;
        public const int CONFIG_RECEIVER_EOF = 0x00;
        public const int CONFIG_RECEIVER_STREAM_TIMEOUT = 500;

        /*------------------Drone Configuration --------------------*/
        public const string DRONE_CONFIGURATION_CONFIG_PATTERN = "([a-zA-Z]*):([a-zA-Z_0-9]*) = ([^\n]*)[\n]";

        /*------------------Navigation Data ------------------------*/
        public const int NAVDATA_TIMEOUT = 2000;
        public const int NAVDATA_KEEPALIVETIMEOUT = 50;
        public const int NAVDATA_PROTOCOLTIMEOUT = 200;
        public const int NAVDATA_HEADER = 0x55667788;
        public const int NAVDATA_INIT_TIMEOUT = 4000;

        /*------------------Command Sender ------------------------*/
        public const int COMMAND_SENDER_CONNECTION_TIMEOUT = 1500;

        /*------------------Drone Controller ------------------------*/
        public const int DRONE_CONTROLLER_CONFIG_FETCH_TIMEOUT = 4000;
        public const int DRONE_CONTROLLER_EMERGENCY_TIMEOUT = 4000;
        public const int DRONE_SENSIVITY_MIN = 1;
        public const int DRONE_SENSIVITY_MAX = 9;
        public const int DRONE_SENSIVITY_DELTA = DRONE_SENSIVITY_MAX - DRONE_SENSIVITY_MIN + 1;

        /*------------------FFMPEG_PATH ------------------------*/
        public const string FFMPEG_PATH = @"../../../FFmpeg.AutoGen/FFmpeg/bin/windows/{0}";

        /*------------------WIFI ------------------------------------*/
        public const int WIFI_TIME_TO_WAIT = 2000;
        public const string WIFI_NETWORK_NAME = "ardrone2_101186";

        /*------------------ Drone Configuration Values -----------------------*/
        // in millimeters (Configid: CONTROL:altitude_max)
        public const int DRONE_MAX_ALTITUDE_MIN = 500;
        public const int DRONE_MAX_ALTITUDE_MAX = 5000;
        public const int DRONE_MAX_ALTITUDE_DELTA = DRONE_MAX_ALTITUDE_MAX - DRONE_MAX_ALTITUDE_MIN;

        // in millimeters per second (Configid: CONTROL:control_vz_max)
        public const int DRONE_MAX_VERTICAL_SPEED_MIN = 400;
        public const int DRONE_MAX_VERTICAL_SPEED_MAX = 2000;
        public const int DRONE_MAX_VERTICAL_SPEED_DELTA = DRONE_MAX_VERTICAL_SPEED_MAX - DRONE_MAX_VERTICAL_SPEED_MIN;

        // in radians per second (Configid: CONTROL:control_yaw)
        public const float DRONE_MAX_YAW_SPEED_MIN = 0.7f;
        public const float DRONE_MAX_YAW_SPEED_MAX = 4f; //6.11f;
        public const float DRONE_MAX_YAW_SPEED_DELTA = DRONE_MAX_YAW_SPEED_MAX - DRONE_MAX_YAW_SPEED_MIN;

        // in raidian (Configid: CONTROL:euler_angle_max)
        public const float DRONE_MAX_PITCH_ROLL_BENDING_ANGLE_MIN = 0f;
        public const float DRONE_MAX_PITCH_ROLL_BENDING_ANGLE_MAX = 0.52f;
        public const float DRONE_MAX_PITCH_ROLL_BENDING_ANGLE_DELTA = DRONE_MAX_PITCH_ROLL_BENDING_ANGLE_MAX - DRONE_MAX_PITCH_ROLL_BENDING_ANGLE_MIN;

    }
}
