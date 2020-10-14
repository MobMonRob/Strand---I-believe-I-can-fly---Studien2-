using AR.Drone.Control;
using AR.Drone.Util;
using AR.Drone.WIFI;
using NativeWifi;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.NavData.Wifi
{
    class WifiReceiver : Worker
    {
        private bool _disposed = false;
        private bool _isConnected = false;
        private bool _isAvailable = false;
        

        private readonly object _syncLock = new object();

        private DroneController _droneController;
        private int _wifiStrength;
        public int WifiStrength
        {
            get
            {
                return _wifiStrength;
            }
        }

        private WlanClient _client;

        //private constructor for Siongleton implementation
        internal WifiReceiver(DroneController droneController)
            :base("WifiReceiver") 
        {
            this._droneController = droneController;
            try
            {
                _client = new WlanClient();
               
            } catch (Win32Exception e) {
                // No wifi adapter exists
                // TODO: implement error handling (e contains the Win32 Error Code)
                Console.WriteLine("Could not open WiFi-Client: " + e.Message);
            }
            
        }

        internal override void DoWork()
        {
            if (null == _client)
            {
                _wifiStrength = -1;
                _droneController.errorStateData.wifiAdapterAvailable = false;
                return;
            }

             while (!this.IsStopRequested())
            {
                _isAvailable = ProcessDefinedNetwork(UpdateWifiStrengthAction);
                if (!_isAvailable)
                {
                    _wifiStrength = -1;
                }
                while (!_isConnected)
                {
                    Thread.Sleep(DroneConstants.WIFI_TIME_TO_WAIT);
                }
                Thread.Sleep(DroneConstants.WIFI_TIME_TO_WAIT);
            }

        }

        /// <summary>
        /// Frame method for accessing the Network information of the Drone Wifi Network
        /// </summary>
        /// <param name="method">The method to be executed with the specified Network of the Drone</param>
        private bool ProcessDefinedNetwork(Action<WlanClient.WlanInterface, Wlan.WlanAvailableNetwork> method)
        {
            lock (_syncLock) 
            {
                bool isAvailable = false;
                foreach (WlanClient.WlanInterface wlanInterface in _client.Interfaces)
                {
                    Wlan.WlanAvailableNetwork[] networks = wlanInterface.GetAvailableNetworkList(0);
                    foreach (Wlan.WlanAvailableNetwork network in networks)
                    {
                        Wlan.Dot11Ssid ssid = network.dot11Ssid;
                        string networkName = Encoding.ASCII.GetString(ssid.SSID, 0, (int)ssid.SSIDLength);

                        if (!DroneConstants.WIFI_NETWORK_NAME.Equals(networkName, StringComparison.CurrentCultureIgnoreCase)) continue;

                        //Do specified action
                        method(wlanInterface, network);
                        isAvailable = true;
                        break;
                        //Enddo specified Action
                    }
                }
                return isAvailable;
            }
        }

        /// <summary>
        /// Action Method for updating the wifi Strength of the ARDrone Network
        /// If the currently connected network is not the ardrone network, the WlanStrength is set to -1
        /// </summary>
        /// <param name="wlanInterface"> the representation of the hardware Interface connected to the network</param>
        /// <param name="network"> the current Network</param>
        private void UpdateWifiStrengthAction(WlanClient.WlanInterface wlanInterface, Wlan.WlanAvailableNetwork network)
        {
            try
            {

                if (wlanInterface.CurrentConnection.profileName.Equals(DroneConstants.WIFI_NETWORK_NAME, StringComparison.CurrentCultureIgnoreCase))
                {
                    _wifiStrength = (int)network.wlanSignalQuality;
                    this._isConnected = true;
                }
                else
                {
                    _wifiStrength = -1;
                    this._isConnected = false;
                }
            }
            catch 
            {
                Console.WriteLine("Wifi exception occured");
            }
        }

        /// <summary>
        /// Action method for Connecting to the ARDrone Wifi Network.
        /// </summary>
        /// <param name="wlanInterface"> the representation of the hardware Interface connected to the network</param>
        /// <param name="network"> the current Network</param>
        private void ConnectToNetworkAction(WlanClient.WlanInterface wlanInterface, Wlan.WlanAvailableNetwork network)
        {
            string profileXml = WlanHelper.GetProfileXMLForOpenConnection(DroneConstants.WIFI_NETWORK_NAME);
            wlanInterface.SetProfile(Wlan.WlanProfileFlags.AllUser, profileXml, true);
            wlanInterface.Connect(Wlan.WlanConnectionMode.Profile, Wlan.Dot11BssType.Any, DroneConstants.WIFI_NETWORK_NAME);
        }

        /// <summary>
        /// This Method tries to Connect to the Drone Wifi Network
        /// </summary>
        private void TryConnectToNetwork()
        {
            if (!this._isConnected)
            {
                this._isConnected = ProcessDefinedNetwork(ConnectToNetworkAction);
            }
        }

        private Boolean IsNetworkAvailable()
        {
            return ProcessDefinedNetwork(UpdateWifiStrengthAction);
        }

        internal override void OnStopRequest()
        {
            //not needed at the moment
            //throw new NotImplementedException();
        }

        protected override void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!_disposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    //nothing todo here, yet
                }
                _client = null;
                _disposed = true;
            }
            base.Dispose(isSaveToRemoveManagedObjects);
        }

        public static void ConnectToWifiNetwork()
        {
            WlanClient client = new WlanClient();
            
            foreach (WlanClient.WlanInterface wlanInterface in client.Interfaces)
            {   
                try {
                    if (!wlanInterface.CurrentConnection.profileName.Equals(DroneConstants.WIFI_NETWORK_NAME, StringComparison.CurrentCultureIgnoreCase))
                    {
                        string profileXml = WlanHelper.GetProfileXMLForOpenConnection(DroneConstants.WIFI_NETWORK_NAME);
                        wlanInterface.SetProfile(Wlan.WlanProfileFlags.AllUser, profileXml, true);
                        wlanInterface.Connect(Wlan.WlanConnectionMode.Profile, Wlan.Dot11BssType.Any, DroneConstants.WIFI_NETWORK_NAME);
                    }
                } catch 
                {
                    Console.WriteLine("Wifi error occured");
                }
            }
           
        }
    }
}
