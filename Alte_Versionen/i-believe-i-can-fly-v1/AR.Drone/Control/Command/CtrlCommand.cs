using AR.Drone.Control.Command.Mode;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    class CtrlCommand : ARCommand
    {
        private CtrlMode _ctrlMode;

        public CtrlCommand(CtrlMode ctrlMode)
            :base(CommandName.CTRL)
        {
            this._ctrlMode = ctrlMode;
        }

        public override string GetCommandParams()
        {
            return string.Format("{0},0", (int) _ctrlMode);
        }
    }
}