using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.Client.CalibrationBinding
{
    public class CalibrationDataSource : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private String _ArmLengthStatus;
        public String ArmLengthStatus
        {
            get
            {
                return _ArmLengthStatus;
            }

            set
            {
                if (_ArmLengthStatus != value)
                {
                    _ArmLengthStatus = value;
                    OnPropertyChanged("ArmLengthStatus");
                }
            }
        }

        private String _BackTiltStatus;
        public String BackTiltStatus
        {
            get
            {
                return _BackTiltStatus;
            }

            set
            {
                if (_BackTiltStatus != value)
                {
                    _BackTiltStatus = value;
                    OnPropertyChanged("BackTiltStatus");
                }
            }
        }

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

    public enum Status
    {
        Initialized,
        Waiting,
        Checked
    }
}

