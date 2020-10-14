using Fuzzy;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Fuzzy
{
    class Program
    {
        static void Main(string[] args)
        {
            FuzzyController ctrl = new FuzzyController();
            ctrl.InitFIS();
        }
    }
}
