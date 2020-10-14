using AR.Drone.Configuration;
using AR.Drone.Control;
using AR.Drone.Data.NavData;
using AR.Drone.NavData.Data;
using AR.Drone.NavData.Data.States;
using AR.Drone.Util;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.NavData
{
    /// <summary>
    /// The Navdata Acquisition Connects to the drone via the NAVDATA_PORT and receives the Navigation Data Continously.
    /// The data will be parsed into a Navigation Packet, to have a better accessibility.
    /// </summary>
    class NavdataAcquisition : Worker
    {
        private bool _disposed = false;

        private DroneController _droneController;
        private NavigationData _navdata;

        /// <summary>
        /// Constructor for naming the thread
        /// </summary>
        public NavdataAcquisition(DroneController droneController)
            : base("NavdataAcquisition") 
        {
            this._droneController = droneController;
        }

        /// <summary>
        /// Within this method the protocoll to initiate navdata connection and receive continous navdata data is implemented
        /// </summary>
        internal override void DoWork()
        {
            try
            {
                NavdataAcquisitionLoop();
            }
            catch (SocketException e)
            {
                //Should not happen due to UDP.
                Console.WriteLine("NavdataAcquisition-thread: UDP-Socket Exception" + e.Message);

                resultState.successfull = false;
                resultState.exception = e;
            }
        }

        /// <summary>
        /// Builds the Navdata UDP-Connection to the drone and receives navdata continously
        /// </summary>
        private void NavdataAcquisitionLoop()
        {
            using (var client = new UdpClient(DroneConstants.NAVDATA_PORT))
            {
                client.Connect(IPAddress.Parse(DroneConstants.IP_ADDRESS), DroneConstants.NAVDATA_PORT);
                var remoteEP = new IPEndPoint(IPAddress.Any, DroneConstants.NAVDATA_PORT);

                //Send first Byte to get the Status of the Drone
                SendKeepAliveSignal(client);

                //now the drone should start to send continous navdata data.
                ReceiveContinousNavdata(client, remoteEP);
            }
        }

        internal override void OnStopRequest()
        {
            //TODO: think about aporpiate implementation
        }

        /// <summary>
        /// This Method sends a keepAliveSignal to the drone, so the drone will not close the connection
        /// </summary>
        private void SendKeepAliveSignal(UdpClient client)
        {
            byte[] data = BitConverter.GetBytes(1);
            client.Send(data, data.Length);
        }

        /// <summary>
        /// this Method runs continously, until it is stopped by calling the RequestStop Method of the super Class or until a connection Timeout happens.
        /// It receives the Navdata bytes, processes the data and sends KeepAlive Signals to the Drone.
        /// </summary>
        private void ReceiveContinousNavdata(UdpClient client, IPEndPoint remoteEP)
        {
            var navdataPacket = new NavdataPacket();

            Stopwatch keepAliveTimer = Stopwatch.StartNew();
            Stopwatch navdataTimer = Stopwatch.StartNew();
            while (!this.IsStopRequested() && navdataTimer.ElapsedMilliseconds < DroneConstants.NAVDATA_TIMEOUT)
            {
                if (client.Available == 0)
                {
                    Thread.Sleep(3);
                }
                else
                {
                    byte[] data = client.Receive(ref remoteEP);
                    navdataTimer.Restart();
                    if (NavigationPacketParser.TryToParseNavdata(ref data, out navdataPacket))
                    {
                        NavdataConverter.ConvertNavdataPacketToNavigationData(ref _navdata, navdataPacket, ref _droneController);
                    }
                    
                }

                if (keepAliveTimer.ElapsedMilliseconds >= DroneConstants.NAVDATA_KEEPALIVETIMEOUT)
                {
                    SendKeepAliveSignal(client);
                    keepAliveTimer.Restart();
                }
            }
        }

        /// <summary>
        /// This method returns the object representatipon of the Processed Navigation Data.
        /// </summary>
        /// <exception cref="TimeoutException">Throws a Timeout Exception if no Connection is available.</exception>
        /// <returns>The navigation data if the Navigation data are not equal null</returns>
        public NavigationData GetNavigationData()
        {
            Stopwatch s = Stopwatch.StartNew();

            while(this._navdata == null && s.ElapsedMilliseconds < DroneConstants.NAVDATA_INIT_TIMEOUT){
                Thread.Sleep(1);
            }
            if (this._navdata == null)
            {
                throw new TimeoutException();
            }
            s.Stop();
            return this._navdata;
        }

        protected override void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!_disposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    //Nothing todo here, yet
                }
                _navdata = null;
                _disposed = true;
            }
            base.Dispose(isSaveToRemoveManagedObjects);
        }

    }
}
 