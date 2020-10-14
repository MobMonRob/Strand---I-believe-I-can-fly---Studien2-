using AR.Drone.Control;
using AR.Drone.Control.Command;
using AR.Drone.Control.Command.Mode;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Configuration.Section
{
    abstract class AbstractConfigSection
    {
        private static readonly Dictionary<string, Object> _settingsMap = new Dictionary<string, Object>();
        private readonly CommandSender _commandSender;
        private readonly ConfigSection _section;

        public AbstractConfigSection(ConfigSection section, ref CommandSender commandSender)
        {
            _section = section;
            _commandSender = commandSender;
        }

        /// <summary>
        /// Returns the value for the specified key
        /// </summary>
        /// <param name="key">The key for which the value is requested</param>
        /// <returns>The value for the given key</returns>
        protected Object GetValueForKey(string key)
        {
            Object value;
            _settingsMap.TryGetValue(_section + ":" + key, out value);
            return value;
        }

        /// <summary>
        /// Fetches the value for the key and converts it to an int32
        /// </summary>
        /// <param name="key">The key</param>
        /// <returns>The resulting int value</returns>
        protected int GetIntValue(string key)
        {
            return (int) GetValueForKey(key);
        }

        /// <summary>
        ///  Fetches the value for the key and converts it to an float
        /// </summary>
        /// <param name="key">The key</param>
        /// <returns>The resulting float value</returns>
        protected float GetFloatValue(string key)
        {
            return (float) GetValueForKey(key);
        }

        /// <summary>
        ///  Fetches the value for the key and converts it to an string
        /// </summary>
        /// <param name="key">The key</param>
        /// <returns>The resulting string value</returns>
        protected string GetStringValue(string key)
        {
            return (string) GetValueForKey(key);
        }

        /// <summary>
        /// Adds the key value pair to the settings
        /// </summary>
        /// <param name="key">The key</param>
        /// <param name="value">The value</param>
        protected void Set(string key, Object value)
        {
            _commandSender.AddCommand(new ConfigCommand(_section.ToString().ToLower() + ":" + key, Convert.ToString(value)));
            _commandSender.AddCommand(new CtrlCommand(CtrlMode.ACK_CONTROL));
        }

        /// <summary>
        /// Adds the key value pair to the settings into the given section string
        /// </summary>
        /// <param name="key">The key</param>
        /// <param name="value">The value</param>
        /// <param name="section">The section</param>
        public static void Add(string key, Object value, string section)
        {
            string mapKey = section + ":" + key;
            if (_settingsMap.ContainsKey(mapKey))
            {
                _settingsMap.Remove(key);
            }
            _settingsMap.Add(mapKey, value);
        }

        /// <summary>
        /// Removes all Key/Value pairs
        /// </summary>
        public static void Clear()
        {
            _settingsMap.Clear();
        }
    }
}