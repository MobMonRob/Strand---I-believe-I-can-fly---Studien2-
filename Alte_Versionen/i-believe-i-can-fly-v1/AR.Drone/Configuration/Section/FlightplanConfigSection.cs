using AR.Drone.Control.Command;
using AR.Drone.Configuration.Section;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using AR.Drone.Control;

namespace AR.Drone.Configuration.Section
{
	class FlightplanConfigSection : AbstractConfigSection
	{

		public FlightplanConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.FLIGHTPLAN, ref commandSender) { }

		internal float default_validation_radius 
 		{
			get { return GetFloatValue("default_validation_radius"); }
			set { Set("default_validation_radius", value); }
 		} 

 		internal float default_validation_time 
 		{
			get { return GetFloatValue("default_validation_time"); }
			set { Set("default_validation_time", value); }
 		} 

 		internal int max_distance_from_takeoff 
 		{
			get { return GetIntValue("max_distance_from_takeoff"); }
			set { Set("max_distance_from_takeoff", value); }
 		} 

 		internal int gcs_ip 
 		{
			get { return GetIntValue("gcs_ip"); }
			set { Set("gcs_ip", value); }
 		} 

 		internal int video_stop_delay 
 		{
			get { return GetIntValue("video_stop_delay"); }
			set { Set("video_stop_delay", value); }
 		} 

 		internal string low_battery_go_home 
 		{
			get { return GetStringValue("low_battery_go_home"); }
			set { Set("low_battery_go_home", value); }
 		} 

 		internal string automatic_heading 
 		{
			get { return GetStringValue("automatic_heading"); }
			set { Set("automatic_heading", value); }
 		} 

 		internal int com_lost_action_delay 
 		{
			get { return GetIntValue("com_lost_action_delay"); }
			set { Set("com_lost_action_delay", value); }
 		} 

 		internal float altitude_go_home 
 		{
			get { return GetFloatValue("altitude_go_home"); }
			set { Set("altitude_go_home", value); }
 		} 

 		internal string mavlink_js_roll_left 
 		{
			get { return GetStringValue("mavlink_js_roll_left"); }
			set { Set("mavlink_js_roll_left", value); }
 		} 

 		internal string mavlink_js_roll_right 
 		{
			get { return GetStringValue("mavlink_js_roll_right"); }
			set { Set("mavlink_js_roll_right", value); }
 		} 

 		internal string mavlink_js_pitch_front 
 		{
			get { return GetStringValue("mavlink_js_pitch_front"); }
			set { Set("mavlink_js_pitch_front", value); }
 		} 

 		internal string mavlink_js_pitch_back 
 		{
			get { return GetStringValue("mavlink_js_pitch_back"); }
			set { Set("mavlink_js_pitch_back", value); }
 		} 

 		internal int mavlink_js_yaw_left 
 		{
			get { return GetIntValue("mavlink_js_yaw_left"); }
			set { Set("mavlink_js_yaw_left", value); }
 		} 

 		internal int mavlink_js_yaw_right 
 		{
			get { return GetIntValue("mavlink_js_yaw_right"); }
			set { Set("mavlink_js_yaw_right", value); }
 		} 

 		internal int mavlink_js_go_up 
 		{
			get { return GetIntValue("mavlink_js_go_up"); }
			set { Set("mavlink_js_go_up", value); }
 		} 

 		internal int mavlink_js_go_down 
 		{
			get { return GetIntValue("mavlink_js_go_down"); }
			set { Set("mavlink_js_go_down", value); }
 		} 

 		internal int mavlink_js_inc_gains 
 		{
			get { return GetIntValue("mavlink_js_inc_gains"); }
			set { Set("mavlink_js_inc_gains", value); }
 		} 

 		internal int mavlink_js_dec_gains 
 		{
			get { return GetIntValue("mavlink_js_dec_gains"); }
			set { Set("mavlink_js_dec_gains", value); }
 		} 

 		internal int mavlink_js_select 
 		{
			get { return GetIntValue("mavlink_js_select"); }
			set { Set("mavlink_js_select", value); }
 		} 

 		internal int mavlink_js_start 
 		{
			get { return GetIntValue("mavlink_js_start"); }
			set { Set("mavlink_js_start", value); }
 		} 

 }}