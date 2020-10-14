using System.Runtime.InteropServices;

namespace AR.Drone.NavData.Data
{
    [StructLayout(LayoutKind.Sequential, Pack = 1, CharSet = CharSet.Ansi)]
    public struct navdata_t
    {
        public uint header;
        public uint ardrone_state;
        public uint sequence;
        public int vision_defined;
    }
}
