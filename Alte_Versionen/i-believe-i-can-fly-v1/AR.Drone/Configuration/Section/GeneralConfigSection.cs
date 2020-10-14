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
	class GeneralConfigSection : AbstractConfigSection
	{

		public GeneralConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.GENERAL, ref commandSender) { }

		internal int num_version_config 
 		{
			get { return GetIntValue("num_version_config"); }
 		} 

 		internal int num_version_mb 
 		{
			get { return GetIntValue("num_version_mb"); }
 		} 

 		internal string num_version_soft 
 		{
			get { return GetStringValue("num_version_soft"); }
 		} 

 		internal string drone_serial 
 		{
			get { return GetStringValue("drone_serial"); }
 		} 

 		internal string soft_build_date 
 		{
			get { return GetStringValue("soft_build_date"); }
 		} 

 		internal float motor1_soft 
 		{
			get { return GetFloatValue("motor1_soft"); }
 		} 

 		internal float motor1_hard 
 		{
			get { return GetFloatValue("motor1_hard"); }
 		} 

 		internal float motor1_supplier 
 		{
			get { return GetFloatValue("motor1_supplier"); }
 		} 

 		internal float motor2_soft 
 		{
			get { return GetFloatValue("motor2_soft"); }
 		} 

 		internal float motor2_hard 
 		{
			get { return GetFloatValue("motor2_hard"); }
 		} 

 		internal float motor2_supplier 
 		{
			get { return GetFloatValue("motor2_supplier"); }
 		} 

 		internal float motor3_soft 
 		{
			get { return GetFloatValue("motor3_soft"); }
 		} 

 		internal float motor3_hard 
 		{
			get { return GetFloatValue("motor3_hard"); }
 		} 

 		internal float motor3_supplier 
 		{
			get { return GetFloatValue("motor3_supplier"); }
 		} 

 		internal float motor4_soft 
 		{
			get { return GetFloatValue("motor4_soft"); }
 		} 

 		internal float motor4_hard 
 		{
			get { return GetFloatValue("motor4_hard"); }
 		} 

 		internal float motor4_supplier 
 		{
			get { return GetFloatValue("motor4_supplier"); }
 		} 

 		internal string ardrone_name 
 		{
			get { return GetStringValue("ardrone_name"); }
			set { Set("ardrone_name", value); }
 		} 

 		internal int flying_time 
 		{
			get { return GetIntValue("flying_time"); }
 		} 

 		internal string navdata_demo 
 		{
			get { return GetStringValue("navdata_demo"); }
			set { Set("navdata_demo", value); }
 		} 

 		internal int com_watchdog 
 		{
			get { return GetIntValue("com_watchdog"); }
			set { Set("com_watchdog", value); }
 		} 

 		internal string video_enable 
 		{
			get { return GetStringValue("video_enable"); }
			set { Set("video_enable", value); }
 		} 

 		internal string vision_enable 
 		{
			get { return GetStringValue("vision_enable"); }
			set { Set("vision_enable", value); }
 		} 

 		internal int vbat_min 
 		{
			get { return GetIntValue("vbat_min"); }
 		} 

 		internal int localtime 
 		{
			get { return GetIntValue("localtime"); }
			set { Set("localtime", value); }
 		} 

 		internal float gps_soft 
 		{
			get { return GetFloatValue("gps_soft"); }
			set { Set("gps_soft", value); }
 		} 

 		internal float gps_hard 
 		{
			get { return GetFloatValue("gps_hard"); }
			set { Set("gps_hard", value); }
 		} 

 		internal string localtime_zone 
 		{
			get { return GetStringValue("localtime_zone"); }
			set { Set("localtime_zone", value); }
 		} 

 		internal int timezone 
 		{
			get { return GetIntValue("timezone"); }
			set { Set("timezone", value); }
 		} 

 		internal int battery_type 
 		{
			get { return GetIntValue("battery_type"); }
			set { Set("battery_type", value); }
 		} 

 		internal string gps_soft_update 
 		{
			get { return GetStringValue("gps_soft_update"); }
			set { Set("gps_soft_update", value); }
 		} 

 		internal int navdata_options 
 		{
			get { return GetIntValue("navdata_options"); }
			set { Set("navdata_options", value); }
 		} 

 }}