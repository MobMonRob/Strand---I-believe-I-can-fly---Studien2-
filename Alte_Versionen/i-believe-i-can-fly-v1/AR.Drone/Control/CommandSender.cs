using AR.Drone.Control.Command;
using AR.Drone.Util;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.Control
{
    class CommandSender : Worker
    {
        private bool _disposed = false;

        private readonly BlockingCollection<ARCommand> _commandQueue;
        private readonly IPAddress _droneIP;
        private readonly int _dronePort;
        private static int _sequenzNumber;
        private static Object lockObject = new Object();

        public CommandSender(String ip, int port) 
            :base("CommandSender")
        {
            _droneIP = IPAddress.Parse(ip);
            _dronePort = port;
            _commandQueue = new BlockingCollection<ARCommand>();

            _sequenzNumber = 1;
        }

        /// <summary>
        /// This methodes must be executed by a thread to start the CommandSender
        /// </summary>
        internal override void DoWork()
        {
            try
            {
                CommandSendLoop();
            }
            catch (SocketException e)
            {
                //Should not happen due to UDP.
                Console.WriteLine("Command Sender-thread: UDP-Socket Exception" + e.Message);

                resultState.successfull = false;
                resultState.exception = e;
            }         
        }

        /// <summary>
        /// Opens the UDP Connection to the drone and
        /// takes Commands from the queue and sends them to the udp connection of the drone
        /// </summary>
        private void CommandSendLoop()
        {
            using (var droneConnection = new UdpClient(_dronePort))
            {
                droneConnection.Connect(_droneIP, _dronePort);

                while (!IsStopRequested())
                {
                    ARCommand command;
                    if (_commandQueue.TryTake(out command, DroneConstants.COMMAND_SENDER_CONNECTION_TIMEOUT))
                    {
                        SendCommand(command, droneConnection);
                        continue;
                    }
                    SendCommand(WatchdogCommand.Instance, droneConnection);
                }
                Console.WriteLine("CommandSender-thread: terminating gracefully.");
            }
        }

        /// <summary>
        /// Sends the given command to the drone
        /// </summary>
        /// <param name="command">The command that should be sent to the drone</param>
        private void SendCommand(ARCommand command, UdpClient client)
        {
            byte[] commandBytes = command.GetCommandBytes(NextSequenzNumber());
            Console.WriteLine(string.Format("Sent Command: {0}", Encoding.ASCII.GetString(commandBytes)));

            client.Send(commandBytes, commandBytes.Length);
        }

        /// <summary>
        /// Adds a command to the command queue. This command will be send after all remaining commands have been sent
        /// </summary>
        /// <param name="command">The command that should be added to the queue</param>
        public void AddCommand(ARCommand command)
        {
            try
            {
                _commandQueue.Add(command);
            }
            catch (InvalidOperationException)
            {
                // can be ignored, happens when the thread is shut down
            }

        }

        /// <summary>
        /// Calculates the next sequenz number and returns it
        /// </summary>
        /// <returns>The next sequenz number</returns>
        private static int NextSequenzNumber()
        {
            lock (lockObject)
            {
                return _sequenzNumber++;
            }
        }

        /// <summary>
        /// Resets the sequenz number
        /// </summary>
        private static void resetSequenzNumber()
        {
            _sequenzNumber = 1;
        }

        internal override void OnStopRequest()
        {
            //Do nothing
        }

        protected override void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!_disposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    // terminate thread first
                    RequestStop();
                    this.Join();

                    _commandQueue.Dispose();
                }
                _disposed = true;
            }
            base.Dispose(isSaveToRemoveManagedObjects);
        }
    }
}