using System.Runtime.InteropServices;

namespace AR.Drone.NavData.Data.Options
{
    [StructLayout(LayoutKind.Sequential, Pack = 1, CharSet = CharSet.Ansi)]
    public struct velocities_t
    {
        public float x;
        public float y;
        public float z;
    }
}
