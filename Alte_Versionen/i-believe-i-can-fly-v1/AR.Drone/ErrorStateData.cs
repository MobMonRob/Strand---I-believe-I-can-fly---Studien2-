using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone
{
    /// <summary>
    /// Holds the information about the state of all possible errors
    /// </summary>
    public class ErrorStateData : INotifyPropertyChanged 
    {
        public event PropertyChangedEventHandler PropertyChanged;
        public const string DRONE_CONNECTED = "droneConnected";

        private bool _droneConnected;
        public bool droneConnected {

            get { return _droneConnected; }

            internal set 
            {
                _droneConnected = value;
                OnPropertyChanged(DRONE_CONNECTED);
            }

        }

        private bool _wifiAdapterAvailable;
        public bool wifiAdapterAvailable {
        
            get { return _wifiAdapterAvailable; } 
            
            internal set 
            {
                _wifiAdapterAvailable = value;
                OnPropertyChanged("wifiAdapterAvailable");
            }
        }

        private bool _navigationDataLost;
        public bool navigationDataLost
        {

            get { return _navigationDataLost; }

            internal set
            {
                _navigationDataLost = value;
                OnPropertyChanged("navigationDataLost");
            }
        }

        private bool _ffmpegError;
        public bool ffmpegError
        {
            get { return _ffmpegError; }

            set
            {
                _ffmpegError = value;
                OnPropertyChanged("ffmpegError");
            }
        }

        private string _ffmpegErrorString;
        public string ffmpegErrorString
        {
            get { return _ffmpegErrorString; }

            internal set
            {
                _ffmpegErrorString = value;
                OnPropertyChanged("ffmpegErrorString");
            }
        }
        
        public ErrorStateData() { }

        protected virtual void OnPropertyChanged(PropertyChangedEventArgs e)
        {
            if (null != PropertyChanged)
            {
                PropertyChanged(this, e);
            }
        }

        /// <summary>
        /// Fires the property changed event for the given property name.
        /// (For data binding)
        /// </summary>
        /// <param name="propertyName">The name of the property which has changed</param>
        private void OnPropertyChanged(string propertyName)
        {
            OnPropertyChanged(new PropertyChangedEventArgs(propertyName));
        }

    }
}
