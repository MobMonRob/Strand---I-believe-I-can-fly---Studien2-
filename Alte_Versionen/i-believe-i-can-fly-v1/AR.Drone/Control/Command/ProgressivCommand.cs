using AR.Drone.Control.Command.Mode;
using AR.Drone.Util;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    /// <summary>
    /// Represents a progressiv AR.Drone command
    /// </summary>
    class ProgressivCommand : ARCommand
    {
        private ProgressiveCommandMode _commandMode;

        public float leftRightTilt { get; set; }
        public float frontBackTilt { get; set; }
        public float verticalSpeed { get; set; }
        public float angularSpeed { get; set; }

        public ProgressivCommand(ProgressiveCommandMode commandMode)
            :base(CommandName.PCMD)
        {
            this._commandMode = commandMode;
        }

        public override string GetCommandParams()
        {
            return string.Format("{0},{1},{2},{3},{4}", (int) _commandMode, Helper.toInt(leftRightTilt), Helper.toInt(frontBackTilt), Helper.toInt(verticalSpeed), Helper.toInt(angularSpeed));
        }
    }
}