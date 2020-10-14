using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.WIFI
{
    public static class WlanHelper
    {
        public static string GetProfileXMLForOpenConnection(string ssid)
        {
            var assembly = Assembly.GetExecutingAssembly();
            var name = assembly.GetName().Name + ".Profile.WifiProfileOpenSecurity.xml";
            StreamReader s = new StreamReader(assembly.GetManifestResourceStream(name));
            var xml = s.ReadToEnd();
            s.Close();
            return String.Format(xml, ssid);
        }
    }
}
