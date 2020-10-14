using AR.Drone.Data.NavData;
using AR.Drone.NavData.Data.Options;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.NavData.Data
{
    public static class NavigationPacketParser
    {
        /// <summary>
        /// This method is for trying to parse the received navigation data from the drone.
        /// </summary>
        /// <param name="data">reference to the received data</param>
        /// <param name="packet">exporting parameter for the navigation Data Packet</param>
        /// <returns>returns true on success, false otherwise</returns>
        public static unsafe bool TryToParseNavdata(ref byte[] data, out NavdataPacket packet)
        {
            packet = new NavdataPacket();

            //Only Process data, if at least the status was send from the drone.
            if (data.Length < sizeof(navdata_t))
            {
                return false;
            }

            fixed (byte* pData = &data[0])
            {
                navdata_t navdata = *(navdata_t*)pData;
                if (navdata.header == DroneConstants.NAVDATA_HEADER)
                {
                    packet.drone_state = navdata.ardrone_state;

                    //Process Different Options
                    int offset = sizeof(navdata_t);
                    while (offset < data.Length)
                    {
                        var option = (navdata_option_t*)(pData + offset);
                        ProcessOption(option, ref packet);
                        offset += option->size;
                    }
                    //Process Checksum
                    uint dataChecksum = CalculateChecksum(data);
                    if (packet.cks.cks == dataChecksum){
                        return true;
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// Based on the type of the Data, this method parses the correct Option of the message in the right field of the navdataPackage
        /// </summary>
        /// <param name="option">the option which should be filled</param>
        /// <param name="packet"> a reference to the navdata Packet, in which the information should be filled</param>
        private static unsafe void ProcessOption(navdata_option_t* option, ref NavdataPacket packet)
        {
            var tag = (navdata_tag_t)option->tag;

            switch (tag)
            {
                case navdata_tag_t.NAVDATA_DEMO_TAG:
                    packet.demo = *(navdata_demo_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_TIME_TAG:
                packet.time = *(navdata_time_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_RAW_MEASURES_TAG:
                    packet.raw_measures = *(navdata_raw_measures_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_PHYS_MEASURES_TAG:
                    packet.phys_measures = *(navdata_phys_measures_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_GYROS_OFFSETS_TAG:
                    packet.gyros_offsets = *(navdata_gyros_offsets_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_EULER_ANGLES_TAG:
                    packet.euler_angles = *(navdata_euler_angles_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_REFERENCES_TAG:
                    packet.references = *(navdata_references_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_TRIMS_TAG:
                    packet.trims = *(navdata_trims_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_RC_REFERENCES_TAG:
                    packet.rc_references = *(navdata_rc_references_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_PWM_TAG:
                    packet.pwm = *(navdata_pwm_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_ALTITUDE_TAG:
                    packet.altitude = *(navdata_altitude_t*) option;
                    break;
                case navdata_tag_t.NAVDATA_VISION_RAW_TAG:
                    packet.vision_raw = *(navdata_vision_raw_t*) option;
                    break;
                case navdata_tag_t.NAVDATA_VISION_OF_TAG:
                    packet.vision_of_tag = *(navdata_vision_of_t*) option;
                    break;
                case navdata_tag_t.NAVDATA_VISION_TAG:
                    packet.vision = *(navdata_vision_t*) option;
                    break;
                case navdata_tag_t.NAVDATA_VISION_PERF_TAG:
                    packet.vision_perf = *(navdata_vision_perf_t*) option;
                    break;
                case navdata_tag_t.NAVDATA_TRACKERS_SEND_TAG:
                    packet.trackers_send = *(navdata_trackers_send_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_VISION_DETECT_TAG:
                    packet.vision_detect = *(navdata_vision_detect_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_WATCHDOG_TAG:
                    packet.watchdog = *(navdata_watchdog_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_ADC_DATA_FRAME_TAG:
                    packet.adc_data_frame = *(navdata_adc_data_frame_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_VIDEO_STREAM_TAG:
                    packet.video_stream = *(navdata_video_stream_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_GAMES_TAG:
                    packet.games = *(navdata_games_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_PRESSURE_RAW_TAG:
                    packet.pressure_raw = *(navdata_pressure_raw_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_MAGNETO_TAG:
                    packet.magneto = *(navdata_magneto_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_WIND_TAG:
                    packet.wind_speed = *(navdata_wind_speed_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_KALMAN_PRESSURE_TAG:
                    packet.kalman_pressure = *(navdata_kalman_pressure_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_HDVIDEO_STREAM_TAG:
                    packet.hdvideo_stream = *(navdata_hdvideo_stream_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_WIFI_TAG:
                    packet.wifi = *(navdata_wifi_t*)option;
                    break;
                case navdata_tag_t.NAVDATA_ZIMMU_3000_TAG:
                    // do nothing
                    break;
                case navdata_tag_t.NAVDATA_NUM_TAGS:
                    // do nothing
                    break;
                case navdata_tag_t.NAVDATA_CKS_TAG:
                    packet.cks = *(navdata_cks_t*)option;
                    break;
                default:
                    // skip uknown options
                    //throw new ArgumentOutOfRangeException();
                    break;
            }

        }

        /// <summary>
        /// Simple method to calculate the checksum of the message.
        /// For this it will just sum up all the bytes without the checksum bytes
        /// </summary>
        /// <param name="buffer">the data to calculate the checksum of</param>
        /// <returns>returns a uint representing the checksum of the message</returns>
        private static uint CalculateChecksum(byte[] buffer)
        {
            uint checksum = 0;
            for (int i = 0; i < buffer.Length - 8; ++i) //go through data bytes without the checksum bytes
                checksum += buffer[i];
            return checksum;
        }
    }
}
