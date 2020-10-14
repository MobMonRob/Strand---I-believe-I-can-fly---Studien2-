using System;

namespace AR.Drone.Control.Command.Mode
{
    [Flags]
    enum ProgressiveCommandMode
    {
        PROGRESSIV = 1 << 0,

        HOVER = 0
    }
}