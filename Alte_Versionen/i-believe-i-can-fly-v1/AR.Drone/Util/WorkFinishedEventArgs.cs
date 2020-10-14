using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AR.Drone.Util
{
    internal class WorkFinishedEventArgs
    {      
        internal bool successfull { get; set; }

        internal Exception exception { get; set; }

        public WorkFinishedEventArgs(bool successfull)
        {
            this.successfull = successfull;
        }
    }
}
