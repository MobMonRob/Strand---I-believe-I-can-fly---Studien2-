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
	class RescueConfigSection : AbstractConfigSection
	{

		public RescueConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.RESCUE, ref commandSender) { }

		internal int rescue 
 		{
			get { return GetIntValue("rescue"); }
			set { Set("rescue", value); }
 		} 

 }}