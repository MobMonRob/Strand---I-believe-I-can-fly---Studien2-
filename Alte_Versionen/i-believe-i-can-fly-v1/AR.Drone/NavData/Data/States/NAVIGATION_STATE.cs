using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.NavData.Data.States
{
    [Flags]
    public enum NAVIGATION_STATE
    {
        Unknown = 0,
        Bootstrap = 1 << 0,
        Landed = 1 << 1,
        Takeoff = 1 << 2,
        Flying = 1 << 3,
        Hovering = 1 << 4,
        Landing = 1 << 5,
        Emergency = 1 << 6,
        Wind = 1 << 7,
        Command = 1 << 8,
        Control = 1 << 9,
        Watchdog = 1 << 10
    }
}
