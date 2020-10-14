using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Control.Command
{
    abstract class ARCommand
    {
        private CommandName _commandName;

        public ARCommand(CommandName commandName)
        {
            _commandName = commandName;
        }

        /// <summary>
        /// This method provides the command specifc parameters as a comma seperated string
        /// </summary>
        /// <returns>The command specific parameter as a string</returns>
        public abstract string GetCommandParams();

        /// <summary>
        /// Creates a byte-Array out of the command parameters.
        /// This can be directly sent do the drone
        /// </summary>
        /// <returns>The byte-Array that must be sent</returns>
        public byte[] GetCommandBytes()
        {
            string commandParams = GetCommandParams();
            string command = string.Format("AT*{0}={1}\r", _commandName, commandParams);

            return Encoding.ASCII.GetBytes(command);
        }

        /// <summary>
        /// Creates a byte-Array out of the command parameters.
        /// This can be directly sent do the drone. It includes the sequenz number
        /// </summary>
        /// <param name="sequenzNumber">The sequenz number</param>
        /// <returns>The byte-Array that must be sent</returns>
        public byte[] GetCommandBytes(int sequenzNumber)
        {
            string commandParams = GetCommandParams();
            string command = string.Format("AT*{0}={1},{2}\r", _commandName, sequenzNumber, commandParams);
            return Encoding.ASCII.GetBytes(command);
        }
    }
}
