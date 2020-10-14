using System;

namespace AR.Drone.Util
{
    static class Helper
    {
        /// <summary>
        /// Creates an 32bit integer out of the byte representation of the float value.
        /// This is NOT just a cast to integer
        /// </summary>
        /// <param name="value">The float value that shoukd be converted to an int</param>
        /// <returns>The resulting integer value</returns>
        internal static int toInt(float value) {
           byte[] valueBytes = BitConverter.GetBytes(value);
           return BitConverter.ToInt32(valueBytes, 0);
        }
    }
}
