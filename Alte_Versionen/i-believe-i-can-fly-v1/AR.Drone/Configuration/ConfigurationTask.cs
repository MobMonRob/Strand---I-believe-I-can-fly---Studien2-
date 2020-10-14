using AR.Drone.Control.Command;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Configuration
{
    class ConfigurationTask
    {
        internal CtrlCommand command { get; private set; }
        internal Action<String> action { get; private set; }

        internal ConfigurationTask(CtrlCommand command, Action<String> action)
        {
            this.command = command;
            this.action = action;
        }
    }
}
