using System.Runtime.InteropServices;

namespace AR.Drone.Util
{
    class NativeMethods
    {

        /// <summary>
        /// Kernel Method to set DLL Path
        /// </summary>
        /// <param name="lpPathName">Path to DLL</param>
        /// <returns></returns>
        [DllImport("kernel32", SetLastError = true, CharSet = CharSet.Unicode, ThrowOnUnmappableChar = true)]
        internal static extern bool SetDllDirectory(string lpPathName);
    }
}
