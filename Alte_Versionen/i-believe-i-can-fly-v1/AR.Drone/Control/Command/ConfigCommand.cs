using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    class ConfigCommand : ARCommand
    {
        private string value, key;

        public ConfigCommand(string key, string value)
            : base(CommandName.CONFIG)
        {
            this.key = key;
            this.value = value;
        }

        public override string GetCommandParams()
        {
            return string.Format("\"{0}\",\"{1}\"", key, value);
        }
    }
}