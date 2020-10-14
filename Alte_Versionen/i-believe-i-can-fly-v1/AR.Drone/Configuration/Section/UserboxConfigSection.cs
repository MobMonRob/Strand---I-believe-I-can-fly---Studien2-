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
	class UserboxConfigSection : AbstractConfigSection
	{

		public UserboxConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.USERBOX, ref commandSender) { }

		internal int userbox_cmd 
 		{
			get { return GetIntValue("userbox_cmd"); }
			set { Set("userbox_cmd", value); }
 		} 

 }}