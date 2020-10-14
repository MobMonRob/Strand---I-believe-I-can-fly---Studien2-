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
	class DetectConfigSection : AbstractConfigSection
	{

		public DetectConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.DETECT, ref commandSender) { }

		internal int enemy_colors 
 		{
			get { return GetIntValue("enemy_colors"); }
			set { Set("enemy_colors", value); }
 		} 

 		internal int enemy_without_shell 
 		{
			get { return GetIntValue("enemy_without_shell"); }
			set { Set("enemy_without_shell", value); }
 		} 

 		internal int groundstripe_colors 
 		{
			get { return GetIntValue("groundstripe_colors"); }
			set { Set("groundstripe_colors", value); }
 		} 

 		internal int detect_type 
 		{
			get { return GetIntValue("detect_type"); }
			set { Set("detect_type", value); }
 		} 

 		internal int detections_select_h 
 		{
			get { return GetIntValue("detections_select_h"); }
			set { Set("detections_select_h", value); }
 		} 

 		internal int detections_select_v_hsync 
 		{
			get { return GetIntValue("detections_select_v_hsync"); }
			set { Set("detections_select_v_hsync", value); }
 		} 

 		internal int detections_select_v 
 		{
			get { return GetIntValue("detections_select_v"); }
			set { Set("detections_select_v", value); }
 		} 

 }}