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
	class ControlConfigSection : AbstractConfigSection
	{

		public ControlConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.CONTROL, ref commandSender) { }

		internal string accs_offset 
 		{
			get { return GetStringValue("accs_offset"); }
 		} 

 		internal string accs_gains 
 		{
			get { return GetStringValue("accs_gains"); }
 		} 

 		internal string gyros_offset 
 		{
			get { return GetStringValue("gyros_offset"); }
 		} 

 		internal string gyros_gains 
 		{
			get { return GetStringValue("gyros_gains"); }
 		} 

 		internal string gyros110_offset 
 		{
			get { return GetStringValue("gyros110_offset"); }
 		} 

 		internal string gyros110_gains 
 		{
			get { return GetStringValue("gyros110_gains"); }
 		} 

 		internal string magneto_offset 
 		{
			get { return GetStringValue("magneto_offset"); }
 		} 

 		internal float magneto_radius 
 		{
			get { return GetFloatValue("magneto_radius"); }
 		} 

 		internal float gyro_offset_thr_x 
 		{
			get { return GetFloatValue("gyro_offset_thr_x"); }
 		} 

 		internal float gyro_offset_thr_y 
 		{
			get { return GetFloatValue("gyro_offset_thr_y"); }
 		} 

 		internal float gyro_offset_thr_z 
 		{
			get { return GetFloatValue("gyro_offset_thr_z"); }
 		} 

 		internal int pwm_ref_gyros 
 		{
			get { return GetIntValue("pwm_ref_gyros"); }
 		} 

 		internal int osctun_value 
 		{
			get { return GetIntValue("osctun_value"); }
 		} 

 		internal string osctun_test 
 		{
			get { return GetStringValue("osctun_test"); }
 		} 

 		internal int altitude_max 
 		{
			get { return GetIntValue("altitude_max"); }
			set { Set("altitude_max", value); }
 		} 

 		internal int altitude_min 
 		{
			get { return GetIntValue("altitude_min"); }
			set { Set("altitude_min", value); }
 		} 

 		internal string outdoor 
 		{
			get { return GetStringValue("outdoor"); }
			set { Set("outdoor", value); }
 		} 

 		internal string flight_without_shell 
 		{
			get { return GetStringValue("flight_without_shell"); }
			set { Set("flight_without_shell", value); }
 		} 

 		internal string autonomous_flight 
 		{
			get { return GetStringValue("autonomous_flight"); }
			set { Set("autonomous_flight", value); }
 		} 

 		internal float flight_anim 
 		{
			get { return GetFloatValue("flight_anim"); }
			set { Set("flight_anim", value); }
 		} 

 		internal int control_level 
 		{
			get { return GetIntValue("control_level"); }
			set { Set("control_level", value); }
 		} 

 		internal float euler_angle_max 
 		{
			get { return GetFloatValue("euler_angle_max"); }
			set { Set("euler_angle_max", value); }
 		} 

 		internal float control_iphone_tilt 
 		{
			get { return GetFloatValue("control_iphone_tilt"); }
			set { Set("control_iphone_tilt", value); }
 		} 

 		internal float control_vz_max 
 		{
			get { return GetFloatValue("control_vz_max"); }
			set { Set("control_vz_max", value); }
 		} 

 		internal float control_yaw 
 		{
			get { return GetFloatValue("control_yaw"); }
			set { Set("control_yaw", value); }
 		} 

 		internal string manual_trim 
 		{
			get { return GetStringValue("manual_trim"); }
			set { Set("manual_trim", value); }
 		} 

 		internal float indoor_euler_angle_max 
 		{
			get { return GetFloatValue("indoor_euler_angle_max"); }
			set { Set("indoor_euler_angle_max", value); }
 		} 

 		internal float indoor_control_vz_max 
 		{
			get { return GetFloatValue("indoor_control_vz_max"); }
			set { Set("indoor_control_vz_max", value); }
 		} 

 		internal float indoor_control_yaw 
 		{
			get { return GetFloatValue("indoor_control_yaw"); }
			set { Set("indoor_control_yaw", value); }
 		} 

 		internal float outdoor_euler_angle_max 
 		{
			get { return GetFloatValue("outdoor_euler_angle_max"); }
			set { Set("outdoor_euler_angle_max", value); }
 		} 

 		internal float outdoor_control_vz_max 
 		{
			get { return GetFloatValue("outdoor_control_vz_max"); }
			set { Set("outdoor_control_vz_max", value); }
 		} 

 		internal float outdoor_control_yaw 
 		{
			get { return GetFloatValue("outdoor_control_yaw"); }
			set { Set("outdoor_control_yaw", value); }
 		} 

 		internal int flying_mode 
 		{
			get { return GetIntValue("flying_mode"); }
			set { Set("flying_mode", value); }
 		} 

 		internal int hovering_range 
 		{
			get { return GetIntValue("hovering_range"); }
			set { Set("hovering_range", value); }
 		} 

 		internal float flying_camera_mode 
 		{
			get { return GetFloatValue("flying_camera_mode"); }
			set { Set("flying_camera_mode", value); }
 		} 

 		internal string flying_camera_enable 
 		{
			get { return GetStringValue("flying_camera_enable"); }
			set { Set("flying_camera_enable", value); }
 		} 

 }}