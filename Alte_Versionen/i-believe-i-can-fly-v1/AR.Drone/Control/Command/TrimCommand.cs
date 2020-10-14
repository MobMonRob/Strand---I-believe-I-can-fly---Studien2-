using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    class TrimCommand : ARCommand
    {
        public TrimCommand()
            : base(CommandName.FTRIM) {}

        public override string GetCommandParams()
        {
            return "";
        }
    }
}