using AR.Drone.Configuration.Section;
using AR.Drone.Control;
//using AR.Drone.Configuration.Sections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace AR.Drone.Configuration
{
    public class DroneConfiguration
    {
        internal GeneralConfigSection general { get; private set; }
        internal ControlConfigSection control { get; private set; }
        internal CustomConfigSection custom { get; private set; }
        internal DetectConfigSection detect { get; private set; }
        internal FlightplanConfigSection flightplan { get; private set; }
        internal GpsConfigSection gps { get; private set; }
        internal LedsConfigSection leds { get; private set; }
        internal NetworkConfigSection network { get; private set; }
        internal PicConfigSection pic { get; private set; }
        internal RescueConfigSection rescue { get; private set; }
        internal SyslogConfigSection syslog { get; private set; }
        internal UserboxConfigSection userbox { get; private set; }
        internal VideoConfigSection video { get; private set; }

        internal DroneConfiguration(ref CommandSender sender)
        {
            general = new GeneralConfigSection(ref sender);
            control = new ControlConfigSection(ref sender);
            custom = new CustomConfigSection(ref sender);
            detect = new DetectConfigSection(ref sender);
            flightplan = new FlightplanConfigSection(ref sender);
            gps = new GpsConfigSection(ref sender);
            leds = new LedsConfigSection(ref sender);
            network = new NetworkConfigSection(ref sender);
            pic = new PicConfigSection(ref sender);
            rescue = new RescueConfigSection(ref sender);
            syslog = new SyslogConfigSection(ref sender);
            userbox = new UserboxConfigSection(ref sender);
            video = new VideoConfigSection(ref sender);
        }

        /// <summary>
        /// Parses the config file text and adds the value to the dictionary.
        /// After this method call the values of the different  config sections can be addressed
        /// </summary>
        /// <param name="configText"></param>
        public void parseConfig(String configText)
        {
            AbstractConfigSection.Clear();
            MatchCollection result = Regex.Matches(configText, DroneConstants.DRONE_CONFIGURATION_CONFIG_PATTERN);

            foreach (Match match in result)
            {
                string section = match.Groups[1].Value;
                string key = match.Groups[2].Value;
                string value = match.Groups[3].Value;

                AbstractConfigSection.Add(key, ConvertToSuitableType(value), section.ToUpper());
            }
        }

        /// <summary>
        /// Tries to find the best fitting type and converts the value to it
        /// </summary>
        /// <param name="s">The text that should be converted</param>
        /// <returns>The converted object</returns>
        private Object ConvertToSuitableType(string s)
        {
            int intResult;
            if (Int32.TryParse(s, out intResult))
            {
                return intResult;
            }

            float floatResult;
            if (float.TryParse(s, out floatResult))
            {
                return floatResult;
            }

            return s;
        }
    }
}