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
	class PicConfigSection : AbstractConfigSection
	{

		public PicConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.PIC, ref commandSender) { }

		internal int ultrasound_freq 
 		{
			get { return GetIntValue("ultrasound_freq"); }
			set { Set("ultrasound_freq", value); }
 		} 

 		internal int ultrasound_watchdog 
 		{
			get { return GetIntValue("ultrasound_watchdog"); }
			set { Set("ultrasound_watchdog", value); }
 		} 

 		internal int pic_version 
 		{
			get { return GetIntValue("pic_version"); }
 		} 

 }}