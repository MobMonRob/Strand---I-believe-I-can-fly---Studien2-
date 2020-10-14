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
	class GpsConfigSection : AbstractConfigSection
	{

		public GpsConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.GPS, ref commandSender) { }

		internal int fw_upload_trigger 
 		{
			get { return GetIntValue("fw_upload_trigger"); }
			set { Set("fw_upload_trigger", value); }
 		} 

 		internal float latitude 
 		{
			get { return GetFloatValue("latitude"); }
			set { Set("latitude", value); }
 		} 

 		internal float longitude 
 		{
			get { return GetFloatValue("longitude"); }
			set { Set("longitude", value); }
 		} 

 		internal float altitude 
 		{
			get { return GetFloatValue("altitude"); }
			set { Set("altitude", value); }
 		} 

 		internal float accuracy 
 		{
			get { return GetFloatValue("accuracy"); }
			set { Set("accuracy", value); }
 		} 

 }}