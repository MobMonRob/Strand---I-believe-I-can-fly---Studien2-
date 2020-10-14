using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    class MISCCommand : ARCommand
    {
        public MISCCommand()
            : base(CommandName.MISC) { }

        public override string GetCommandParams()
        {
            return "2,20,2000,3000";
        }
    }
}
