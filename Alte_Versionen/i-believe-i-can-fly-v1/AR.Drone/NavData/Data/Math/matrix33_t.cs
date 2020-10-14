using System.Runtime.InteropServices;

namespace AR.Drone.NavData.Data.Math
{
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct matrix33_t
    {
        public float m11;
        public float m12;
        public float m13;
        public float m21;
        public float m22;
        public float m23;
        public float m31;
        public float m32;
        public float m33;
    }
}