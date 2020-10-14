using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.NavData.Data.States
{
    public enum CTRL_STATES : ushort
    {
        CTRL_DEFAULT,
        CTRL_INIT,
        CTRL_LANDED,
        CTRL_FLYING,
        CTRL_HOVERING,
        CTRL_TEST,
        CTRL_TRANS_TAKEOFF,
        CTRL_TRANS_GOTOFIX,
        CTRL_TRANS_LANDING,
        CTRL_TRANS_LOOPING
    }
}
