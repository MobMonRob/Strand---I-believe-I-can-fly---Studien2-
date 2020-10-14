using AR.Drone;
using AR.Drone.WIFI;
using NativeWifi;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace WlanTest
{
    class Program
    {
        static string GetStringForSSID(Wlan.Dot11Ssid ssid)
        {
            return Encoding.ASCII.GetString(ssid.SSID, 0, (int)ssid.SSIDLength);
        }

        static string GetProfileXMLForOpenConnection()
        {
            var assembly = Assembly.GetExecutingAssembly();
            var name = assembly.GetName().Name + ".WifiProfileOpenSecurity.xml";
            StreamReader s = new StreamReader(assembly.GetManifestResourceStream(name));
            var xml = s.ReadToEnd();
            s.Close();
            return xml;
        }

        static void Main(string[] args)
        {
            WlanClient client = new WlanClient();
            foreach (WlanClient.WlanInterface wlanIface in client.Interfaces)
            {
                // Lists all networks with WEP security
                Wlan.WlanAvailableNetwork[] networks = wlanIface.GetAvailableNetworkList(0);
                foreach (Wlan.WlanAvailableNetwork network in networks)
                {
                    //if (network.dot11DefaultCipherAlgorithm == Wlan.Dot11CipherAlgorithm.WEP)
                    //{
                        Console.WriteLine("Found network with SSID {0}.", GetStringForSSID(network.dot11Ssid));
                    //}
                }

                // Retrieves XML configurations of existing profiles.
                // This can assist you in constructing your own XML configuration
                // (that is, it will give you an example to follow).
                foreach (Wlan.WlanProfileInfo profileInfo in wlanIface.GetProfiles())
                {
                    string name = profileInfo.profileName; // this is typically the network's SSID
                    string xml = wlanIface.GetProfileXml(profileInfo.profileName);
                    if (name == DroneConstants.WIFI_NETWORK_NAME)
                    {
                        Console.WriteLine(xml);
                    }
                    
                }
                Console.WriteLine("press any key to connect to test");
                Console.ReadKey();

                // Connects to a known network with WEP security
                /*string profileName = "Cheesecake"; // this is also the SSID
                string mac = "52544131303235572D454137443638";
                string key = "hello";
                string profileXml = string.Format("<?xml version=\"1.0\"?><WLANProfile xmlns=\"http://www.microsoft.com/networking/WLAN/profile/v1\"><name>{0}</name><SSIDConfig><SSID><hex>{1}</hex><name>{0}</name></SSID></SSIDConfig><connectionType>ESS</connectionType><MSM><security><authEncryption><authentication>open</authentication><encryption>WEP</encryption><useOneX>false</useOneX></authEncryption><sharedKey><keyType>networkKey</keyType><protected>false</protected><keyMaterial>{2}</keyMaterial></sharedKey><keyIndex>0</keyIndex></security></MSM></WLANProfile>", profileName, mac, key);
                wlanIface.SetProfile(Wlan.WlanProfileFlags.AllUser, profileXml, true);
                wlanIface.Connect(Wlan.WlanConnectionMode.Profile, Wlan.Dot11BssType.Any, profileName);*/

                //string profileXml = String.Format("<?xml version=\"1.0\"?><WLANProfile xmlns=\"http://www.microsoft.com/networking/WLAN/profile/v1\"><name>{0}</name><SSIDConfig><SSID><name>{0}</name></SSID><nonBroadcast>false</nonBroadcast></SSIDConfig><connectionType>ESS</connectionType><connectionMode>manual</connectionMode><MSM><security><authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption></security></MSM></WLANProfile>", "test");

                string ssid = "test";

                string profileXml = WlanHelper.GetProfileXMLForOpenConnection(ssid);
                wlanIface.SetProfile(Wlan.WlanProfileFlags.AllUser, profileXml, true);
                wlanIface.Connect(Wlan.WlanConnectionMode.Profile, Wlan.Dot11BssType.Any, ssid);
                Console.WriteLine("press any key to continue...");
                Console.ReadKey();
            }
        }
    }
}
