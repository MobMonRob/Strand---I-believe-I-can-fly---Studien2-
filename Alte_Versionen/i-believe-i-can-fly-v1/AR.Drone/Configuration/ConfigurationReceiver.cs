using AR.Drone.Control;
using AR.Drone.Control.Command;
using AR.Drone.Control.Command.Mode;
using AR.Drone.Util;
using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace AR.Drone.Configuration
{
    /// <summary>
    /// This class is responsible for fetching the configuration data
    /// </summary>
    class ConfigurationReceiver : Worker
    {
        private bool _disposed = false;

        private DroneConfiguration _droneConfiguration;
        private CommandSender _sender;

        private BlockingCollection<ConfigurationTask> _requestQueue = new BlockingCollection<ConfigurationTask>();

        private CancellationTokenSource cancellationTokenSource = new CancellationTokenSource();

        public ConfigurationReceiver(CommandSender sender, DroneConfiguration droneConfig)
            :base("ConfigurationReceiver")
        {
            _sender = sender;
            _droneConfiguration = droneConfig;
        }

        /// <summary>
        /// Sends an Acknowledge Command on the control port
        /// </summary>
        private void AcknowledgeControl()
        {
            _sender.AddCommand(new CtrlCommand(CtrlMode.ACK_CONTROL));
            Thread.Sleep(10);
        }

        /// <summary>
        /// Executes a ConfigurationTask.
        /// Therefor it sends the specified command on the command port and waits for the answer on the data port.
        /// If the connection times out 10 retries are made
        /// </summary>
        /// <param name="configTask">The ConfigurationTask that should be executed</param>
        private void ExcecuteConfigurationTask(ConfigurationTask configTask)
        {
            using (var tcpClient = new TcpClient())
            {
                IAsyncResult asyncResult = tcpClient.BeginConnect(DroneConstants.IP_ADDRESS, DroneConstants.CONTROL_PORT, null, null);

                var success = asyncResult.AsyncWaitHandle.WaitOne(TimeSpan.FromSeconds(DroneConstants.CONFIG_RECEIVER_CONNECTION_TIMEOUT));
                if (!success)
                {
                    asyncResult.AsyncWaitHandle.Close();
                    throw new TimeoutException("Connection Timeout! Could not connect to Drone");
                }

                NetworkStream stream = tcpClient.GetStream();
                stream.ReadTimeout = DroneConstants.CONFIG_RECEIVER_STREAM_TIMEOUT;

                _sender.AddCommand(configTask.command);
                Thread.Sleep(100);
                AcknowledgeControl();

                MemoryStream memoryStream = new MemoryStream();
                byte[] buffer = new byte[DroneConstants.CONFIG_RECEIVER_MAX_BUFFER_SIZE];
                int bytesRead = 1;
                int receiveAttempts = 1;

                while (bytesRead > 0 && !IsStopRequested() && receiveAttempts <= DroneConstants.CONFIG_RECEIVER_MAX_CONNECTION_ATTEMPS)
                {
                    try
                    {
                        bytesRead = stream.Read(buffer, 0, buffer.Length);
                        memoryStream.Write(buffer, 0, bytesRead);
                        Console.WriteLine("Bytes read: " + bytesRead);

                        if (bytesRead == 0)
                        {
                            bytesRead = 1;
                            continue;
                        }

                        if (buffer[bytesRead - 1] == DroneConstants.CONFIG_RECEIVER_EOF)
                        {
                            // EOF
                            Console.WriteLine("EOF");
                            AcknowledgeControl();
                            break;
                        }
                    }
                    catch (IOException)
                    {
                        // Configuration data request failed
                        Console.WriteLine("Connection Timeout! Attempt: " + receiveAttempts);
                        receiveAttempts++;

                        // Reset configuration retrieved status
                        AcknowledgeControl();

                        // Request config data again
                        _sender.AddCommand(configTask.command);
                        Thread.Sleep(100);
                    }
                }

                string result = Encoding.UTF8.GetString(memoryStream.ToArray());
                memoryStream.Close();

                configTask.action(result);
            }
        }

        internal override void DoWork()
        {
            while (!IsStopRequested())
            {
                try
                {
                    ConfigurationTask configTask = _requestQueue.Take(cancellationTokenSource.Token);
                    ExcecuteConfigurationTask(configTask);
                }
                catch (OperationCanceledException)
                {
                    // cancellation token received
                    resultState.successfull = true;
                    return;
                }
                catch (Exception e)
                {
                    resultState.successfull = false;
                    resultState.exception = e;
                    return;
                }
            }
        }

        internal override void OnStopRequest()
        {
            cancellationTokenSource.Cancel();
        }

        /// <summary>
        /// Reqeuests the list of custom configuration ids.
        /// When the request is finished the callback Action is called
        /// </summary>
        /// <param name="callback">The Action that should be executed after the data is fetched</param>
        public void GetCustomConfigurationIds(Action<string> callback)
        {
            ConfigurationTask configTask = new ConfigurationTask(new CtrlCommand(CtrlMode.CUSTOM_CFG_GET_CONTROL_MODE), callback);
            _requestQueue.Add(configTask);
        }

        /// <summary>
        /// Requests the configuration of the drone
        /// </summary>
        /// <param name="callback">The Action that should be executed after the data is fetched</param>
        public void GetConfiguration(Action<DroneConfiguration> callback)
        {
            Action<String> action = (configText) =>
            {
                _droneConfiguration.parseConfig(configText); 
                callback(_droneConfiguration );
            };

            ConfigurationTask configTask = new ConfigurationTask(new CtrlCommand(CtrlMode.CFG_GET_CONTROL_MODE), action);
            _requestQueue.Add(configTask);
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

                    _requestQueue.Dispose();
                    cancellationTokenSource.Dispose();
                }
                _disposed = true;
            }
            base.Dispose(isSaveToRemoveManagedObjects);
        }
    }
}