using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    class WatchdogCommand : ARCommand
    {
        public static WatchdogCommand Instance = new WatchdogCommand();

        public WatchdogCommand()
            : base(CommandName.COMWDG) { }

        public override string GetCommandParams()
        {
            return "";      
        }
    }
}
