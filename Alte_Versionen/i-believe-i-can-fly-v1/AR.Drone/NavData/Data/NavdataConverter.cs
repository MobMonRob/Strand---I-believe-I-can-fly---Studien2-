using AR.Drone.Control;
using AR.Drone.Data.NavData;
using AR.Drone.NavData.Data.States;
using AR.Drone.NavData.Wifi;
using AR.Drone.Util;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.NavData.Data
{
    /// <summary>
    /// The NavdataConverter Class takes a Navdata Packet and Converts the information to the relevant Navigation Data.
    /// </summary>
    public static class NavdataConverter
    {
        //helper constant for converting degree to radian
        private const float RADIAN = (float) (System.Math.PI/180.0f);

        /// <summary>
        /// This methods converts the navdata information received from the drone to the Information needed for further processing
        /// </summary>
        /// <param name="navdata">Reference to the Navigation Data to be filled</param>
        /// <param name="packet">the Information received from the Drone</param>
        public static void ConvertNavdataPacketToNavigationData(ref NavigationData navdata, NavdataPacket packet, ref DroneController droneController)
        {
            if (navdata == null)
            {
                navdata = new NavigationData();
            }
            else
            {
                navdata.State = 0;
            }            

            var ardroneState = (def_ardrone_state_mask_t)packet.drone_state;
            UpdateState(ardroneState, ref navdata.State);

            var ctrlState = (CTRL_STATES)(packet.demo.ctrl_state << 0x10);
            UpdateState(ctrlState, ref navdata.State);

            navdata.Yaw = (packet.demo.psi / 1000.0f);
            navdata.Pitch = (packet.demo.theta / 1000.0f);
            navdata.Roll = (packet.demo.phi / 1000.0f);

            navdata.Altitude = packet.demo.altitude / 1000.0f;

            navdata.Time = packet.time.time;

            Vector3 velocity = new Vector3();
            velocity.X = packet.demo.vx / 1000.0f;
            velocity.Y = packet.demo.vy / 1000.0f;
            velocity.Z = packet.demo.vz / 1000.0f;
            navdata.Velocity = velocity;

            Battery battery = new Battery();
            battery.Low = ardroneState.HasFlag(def_ardrone_state_mask_t.ARDRONE_VBAT_LOW);
            battery.Percentage = packet.demo.vbat_flying_percentage;
            battery.Voltage = packet.raw_measures.vbat_raw / 1000.0f;
            navdata.Battery = battery;

            Magneto magneto = new Magneto();
            magneto.Rectified.X = packet.magneto.magneto_rectified.x;
            magneto.Rectified.Y = packet.magneto.magneto_rectified.y;
            magneto.Rectified.Z = packet.magneto.magneto_rectified.z;
            magneto.Offset.X = packet.magneto.magneto_offset.x;
            magneto.Offset.Y = packet.magneto.magneto_offset.y;
            magneto.Offset.Z = packet.magneto.magneto_offset.z;
            navdata.Magneto = magneto;

            Video video = new Video();
            video.FrameNumber = packet.video_stream.frame_number;
            navdata.Video = video;

            Wifi wifi = new Wifi();
            wifi.LinkQuality = (float)(droneController.wifiReceiver.WifiStrength) / 100.0f;
            //wifi.LinkQuality = 1.0f - Helper.ToSingle(packet.wifi.link_quality);
            navdata.Wifi = wifi;
        }

        

        /// <summary>
        /// this method is for Filling the state of the NavigationData
        /// </summary>
        /// <param name="ctrl_states">The state flags received by the drone</param>
        /// <param name="nav_state">reference to the Navigation State of the NavigationData Class</param>
        private static void UpdateState(CTRL_STATES ctrl_states, ref NAVIGATION_STATE nav_state)
        {
            switch (ctrl_states)
            {
                case CTRL_STATES.CTRL_TRANS_TAKEOFF:
                    nav_state |= NAVIGATION_STATE.Takeoff;
                    break;
                case CTRL_STATES.CTRL_TRANS_LANDING:
                    nav_state |= NAVIGATION_STATE.Landing;
                    break;
                case CTRL_STATES.CTRL_HOVERING:
                    nav_state |= NAVIGATION_STATE.Hovering;
                    break;
            }
        }

        /// <summary>
        /// this method is for Filling the state of the NavigationData
        /// </summary>
        /// <param name="ardrone_state">The state flags received by the drone</param>
        /// <param name="nav_state">reference to the Navigation State of the NavigationData Class</param>
        private static void UpdateState(def_ardrone_state_mask_t ardrone_state, ref NAVIGATION_STATE nav_state)
        {
            if (ardrone_state.HasFlag(def_ardrone_state_mask_t.ARDRONE_NAVDATA_BOOTSTRAP))
                nav_state |= NAVIGATION_STATE.Bootstrap;

            if (ardrone_state.HasFlag(def_ardrone_state_mask_t.ARDRONE_FLY_MASK))
                nav_state |= NAVIGATION_STATE.Flying;
            else
                nav_state |= NAVIGATION_STATE.Landed;

            if (ardrone_state.HasFlag(def_ardrone_state_mask_t.ARDRONE_WIND_MASK))
                nav_state |= NAVIGATION_STATE.Wind;

            if (ardrone_state.HasFlag(def_ardrone_state_mask_t.ARDRONE_EMERGENCY_MASK))
                nav_state |= NAVIGATION_STATE.Emergency;

            if (ardrone_state.HasFlag(def_ardrone_state_mask_t.ARDRONE_COMMAND_MASK))
                nav_state |= NAVIGATION_STATE.Command;

            if (ardrone_state.HasFlag(def_ardrone_state_mask_t.ARDRONE_CONTROL_MASK))
                nav_state |= NAVIGATION_STATE.Control;

            if (ardrone_state.HasFlag(def_ardrone_state_mask_t.ARDRONE_COM_WATCHDOG_MASK))
                nav_state |= NAVIGATION_STATE.Watchdog;
        }
    }
}
