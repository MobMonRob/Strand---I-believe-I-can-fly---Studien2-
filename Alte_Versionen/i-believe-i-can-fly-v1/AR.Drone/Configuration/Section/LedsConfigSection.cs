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
	class LedsConfigSection : AbstractConfigSection
	{

		public LedsConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.LEDS, ref commandSender) { }

		internal float leds_anim 
 		{
			get { return GetFloatValue("leds_anim"); }
			set { Set("leds_anim", value); }
 		} 

 }}