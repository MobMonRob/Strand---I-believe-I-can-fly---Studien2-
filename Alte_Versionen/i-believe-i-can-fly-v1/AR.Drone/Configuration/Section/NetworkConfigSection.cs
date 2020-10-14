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
	class NetworkConfigSection : AbstractConfigSection
	{

		public NetworkConfigSection(ref CommandSender commandSender)
			:base(ConfigSection.NETWORK, ref commandSender) { }

		internal string ssid_single_player 
 		{
			get { return GetStringValue("ssid_single_player"); }
			set { Set("ssid_single_player", value); }
 		} 

 		internal string ssid_multi_player 
 		{
			get { return GetStringValue("ssid_multi_player"); }
			set { Set("ssid_multi_player", value); }
 		} 

 		internal int wifi_mode 
 		{
			get { return GetIntValue("wifi_mode"); }
			set { Set("wifi_mode", value); }
 		} 

 		internal int wifi_rate 
 		{
			get { return GetIntValue("wifi_rate"); }
			set { Set("wifi_rate", value); }
 		} 

 		internal string owner_mac 
 		{
			get { return GetStringValue("owner_mac"); }
			set { Set("owner_mac", value); }
 		} 

 }}