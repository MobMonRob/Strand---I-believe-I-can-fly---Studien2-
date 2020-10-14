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
	class SyslogConfigSection : AbstractConfigSection
	{

		public SyslogConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.SYSLOG, ref commandSender) { }

		internal int output 
 		{
			get { return GetIntValue("output"); }
			set { Set("output", value); }
 		} 

 		internal int max_size 
 		{
			get { return GetIntValue("max_size"); }
			set { Set("max_size", value); }
 		} 

 		internal int nb_files 
 		{
			get { return GetIntValue("nb_files"); }
			set { Set("nb_files", value); }
 		} 

 }}