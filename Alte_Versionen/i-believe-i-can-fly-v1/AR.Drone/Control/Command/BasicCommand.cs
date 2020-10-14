using AR.Drone.Control.Command.Mode;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    /// <summary>
    /// Represents a Basic AR.Drone command
    /// </summary>
    class BasicCommand : ARCommand
    {
        public static readonly BasicCommand DEFAULT_COMMAND = new BasicCommand(BasicCommandMode.DEFAULT);
        public static readonly BasicCommand TAKE_OFF_COMMAND = new BasicCommand(BasicCommandMode.TAKE_OFF);
        public static readonly BasicCommand LAND_COMMAND = new BasicCommand(BasicCommandMode.LAND);
        public static readonly BasicCommand EMERGENCY_COMMAND = new BasicCommand(BasicCommandMode.EMERGENCY);

        private BasicCommandMode _commandMode; 
        
        public BasicCommand(BasicCommandMode basicCommandMode)
            : base(CommandName.REF)
        {
            _commandMode = basicCommandMode;
        }
    
        public override string GetCommandParams()
        {
            return string.Format("{0}", (int) _commandMode);
        }
    }
}
