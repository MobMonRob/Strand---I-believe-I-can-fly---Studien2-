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
	class CustomConfigSection : AbstractConfigSection
	{

		public CustomConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.CUSTOM, ref commandSender) { }

		internal string application_desc 
 		{
			get { return GetStringValue("application_desc"); }
			set { Set("application_desc", value); }
 		} 

 		internal string profile_desc 
 		{
			get { return GetStringValue("profile_desc"); }
			set { Set("profile_desc", value); }
 		} 

 		internal int application_id 
 		{
			get { return GetIntValue("application_id"); }
			set { Set("application_id", value); }
 		} 

 		internal int profile_id 
 		{
			get { return GetIntValue("profile_id"); }
			set { Set("profile_id", value); }
 		} 

 		internal int session_id 
 		{
			get { return GetIntValue("session_id"); }
			set { Set("session_id", value); }
 		} 

 		internal string session_desc 
 		{
			get { return GetStringValue("session_desc"); }
			set { Set("session_desc", value); }
 		} 

 }}