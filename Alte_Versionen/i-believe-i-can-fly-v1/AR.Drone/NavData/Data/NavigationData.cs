using AR.Drone.NavData.Data.States;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace AR.Drone.NavData.Data
{
    public class NavigationData : INotifyPropertyChanged 
    {
        public NAVIGATION_STATE State;
        public event PropertyChangedEventHandler PropertyChanged;

        private float _Yaw;
        public float Yaw // radians - Yaw - Z
        {   
            get { return _Yaw; }
            
            set
            {
                if (value != _Yaw)
                {
                    _Yaw = value;
                    OnPropertyChanged("Yaw");
                }                
            }
        }

        private float _Pitch;
        public float Pitch // radians - Pitch - Y
        {
            get { return _Pitch; }

            set
            {
                if (value != _Pitch)
                {
                    _Pitch = value;
                    OnPropertyChanged("Pitch");
                }                
            }
        }

        private float _Roll;
        public float Roll // radians - Roll - X
        {
            get { return _Roll; }

            set
            {
                if (value != _Roll)
                {
                    _Roll = value;
                    OnPropertyChanged("Roll");
                }                
            }
        } 
            
        private float _Altitude;
        public float Altitude // meters
        {
            get { return _Altitude; }

            internal set
            {
                if (value != _Altitude)
                {
                    _Altitude = value;
                    OnPropertyChanged("Altitude");
                }                
            }
        } 

        private Vector3 _Velocity;
        public Vector3 Velocity // meter/second
        {
            get { return _Velocity; }

            internal set
            {
                if (!value.Equals(_Velocity))
                {
                    _Velocity = value;
                    OnPropertyChanged("Velocity");
                }                
            }
        }


       public float BatteryPercentage
        {
            get { return _Battery.Percentage; }

            set
            {
                if (!_Battery.Percentage.Equals(value))
                {
                    if (value < 0)
                    {
                        _Battery.Percentage = 0;
                    }
                    else if (value > 100)
                    {
                        _Battery.Percentage = 0;
                    }
                    else
                    {
                        _Battery.Percentage = value;
                    }

                    OnPropertyChanged("BatteryPercentage");
                }                
            }
        }

       public Boolean BatteryLow
       {
           get { return Battery.Low; }
           set
           {
               if (!_Battery.Low.Equals(value))
               {
                   _Battery.Low = value;
                   OnPropertyChanged("BatteryLow");
               }
           }
       }

        private Battery _Battery;

        public Battery Battery
        {
            get { return _Battery;  }

            set
            {
                if (!_Battery.Equals(value))
                {
                    _Battery = value;
                    OnPropertyChanged("Battery");
                    OnPropertyChanged("BatteryPercentage");
                }
            }
        } 

        private Magneto _Magneto;
        public Magneto Magneto
        {
            get { return _Magneto; }

            internal set
            {
                if (!_Magneto.Equals(value))
                {
                    _Magneto = value;
                    OnPropertyChanged("Magneto");
                }                
            }
        } 

        private float _Time;
        public float Time // seconds
        {
            get { return _Time; }

            internal set
            {
                if (value != _Time)
                {
                    _Time = value;
                    OnPropertyChanged("Time");
                }                
            }
        } 

        private Video _Video;
        public Video Video
        {
            get { return _Video; }

            internal set
            {
                if (!_Video.Equals(value))
                {
                    _Video = value;
                    OnPropertyChanged("Video");
                }                
            }
        }

        public float WifiLinkQuality {

            get { return _Wifi.LinkQuality; }

            set
            {
                if (!_Wifi.LinkQuality.Equals(value))
                {
                    _Wifi.LinkQuality = value;
                    OnPropertyChanged("WifiLinkQuality");
                }
            }
        }

        private Wifi _Wifi;
        public Wifi Wifi
        {
            get { return _Wifi; }

            internal set
            {
                if (!_Wifi.Equals(value))
                {
                    _Wifi = value;
                    OnPropertyChanged("Wifi");
                    OnPropertyChanged("WifiLinkQuality");
                }                
            }
        } 

        /// <summary>
        /// This method is for asking if the state is landed
        /// </summary>
        /// <returns>true if the Drone is landed, false otherwise</returns>
        public bool IsLanded()
        {
            return State.HasFlag(NAVIGATION_STATE.Landed);
        }

        /// <summary>
        /// This method is for asking if the State of the Drone is Flying.
        /// </summary>
        /// <returns>true if the State of the Drone is Flying</returns>
        public bool IsFlying()
        {
            return State.HasFlag(NAVIGATION_STATE.Flying);
        }

        /// <summary>
        /// This method is for Asking if the drone is in emergency Mode
        /// </summary>
        /// <returns>true if the state of the Drone is emergency</returns>
        public bool IsEmergency()
        {
            return State.HasFlag(NAVIGATION_STATE.Emergency);
        }

        /// <summary>
        /// This method is for asking if the state of the Drone is hovering
        /// </summary>
        /// <returns>True if the drone is hovering, false otherwise</returns>
        public bool IsHovering()
        {
            return State.HasFlag(NAVIGATION_STATE.Hovering);            
        }

        /// <summary>
        /// this method is for asking if the drone is in bootstrap Mode.
        /// If the Drone is in Bootstrap mode, it will only send the state vie the NavData Port.
        /// To End Bootstrap Mode and to receive full NavData we have to send a generalControl command.
        /// </summary>
        /// <returns>true if drone is in Bootstrap Mode, false otherwise</returns>
        public bool IsBootstrap()
        {
            return State.HasFlag(NAVIGATION_STATE.Bootstrap);
        }

        /// <summary>
        /// this method is for asking if the drone is in TakeOff Mode.
        /// </summary>
        /// <returns>true if the drone is in TakeOff Mode, false otherwise</returns>
        public bool IsTakeOff()
        {
            return State.HasFlag(NAVIGATION_STATE.Takeoff);
        }

        /// <summary>
        /// this method is for asking if the drone is in Control Mode.
        /// </summary>
        /// <returns>true if the Drone is in Control Mode, false otherwise</returns>
        public bool IsControl()
        {
            return State.HasFlag(NAVIGATION_STATE.Control);
        }

        /// <summary>
        /// this method is for asking if the dron is in watchdog mode
        /// </summary>
        /// <returns>true if the dron is in watchdog mode, otherwise false</returns>
        public bool IsWatchdog()
        {
            return State.HasFlag(NAVIGATION_STATE.Watchdog);
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

    [StructLayout(LayoutKind.Sequential)]
    public struct Battery
    {
        public bool Low;
        public float Percentage;
        public float Voltage; // in volts

        public override string ToString()
        {
            return string.Format("{{Low:{0} Percentage:{1} Voltage:{2}}}", Low, Percentage, Voltage);
        }

        public override bool Equals(object obj)
        {
            Battery battery = (Battery)obj;
            return battery.Low == this.Low && battery.Percentage == this.Percentage && battery.Voltage == this.Voltage;
        }

        public static bool operator == (Battery b1, Battery b2)
        {
            if (object.ReferenceEquals(b1, null))
            {
                return object.ReferenceEquals(b2, null);
            }
            return b1.Equals(b2);
        }

        public static bool operator !=(Battery b1, Battery b2)
        {
            return !(b1 == b2);
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Vector3
    {
        public float X; // meter/second
        public float Y; // meter/second
        public float Z; // meter/second

        public override string ToString()
        {
            return string.Format("{{X:{0} Y:{1} Z:{2}}}", X, Y, Z);
        }

        public override bool Equals(object obj)
        {
            Vector3 vect = (Vector3)obj;
            return vect.X == this.X && vect.Y == this.Y && vect.Z == this.Z;
        }

        public static bool operator ==(Vector3 v1, Vector3 v2)
        {
            if (object.ReferenceEquals(v1, null))
            {
                return object.ReferenceEquals(v2, null);
            }
            return v1.Equals(v2);
        }

        public static bool operator !=(Vector3 v1, Vector3 v2)
        {
            return !(v1 == v2);
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Magneto
    {
        public Vector3 Rectified;
        public Vector3 Offset;

        public override bool Equals(object obj)
        {
            Magneto magneto = (Magneto)obj;
            return magneto.Rectified.Equals(this.Rectified) && magneto.Offset.Equals(this.Offset);
        }

        public static bool operator ==(Magneto m1, Magneto m2) {
            if (object.ReferenceEquals(m1, null)) {
                return object.ReferenceEquals(m2, null);
            }

            return m1.Equals(m2);
        }

        public static bool operator !=(Magneto m1, Magneto m2)
        {
            return !(m1 == m2);
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Wifi
    {
        public float LinkQuality; // 1 is perfect, less than 1 is worse

        public override string ToString()
        {
            return string.Format("{{LinkQuality:{0}}}", LinkQuality);
        }

        public override bool Equals(object obj)
        {
            Wifi wifi = (Wifi)obj;
            return this.LinkQuality == wifi.LinkQuality;
        }

        public static bool operator ==(Wifi w1, Wifi w2) {
            if (object.ReferenceEquals(w1, null))
            {
                object.ReferenceEquals(w2, null);
            }

            return w1.Equals(w2);
        }

        public static bool operator !=(Wifi w1, Wifi w2)
        {
            return !(w1 == w2);
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Video
    {
        public uint FrameNumber;

        public override string ToString()
        {
            return string.Format("{{FrameNumber:{0}}}", FrameNumber);
        }

        public override bool Equals(object obj)
        {
            Video video = (Video)obj;
            return video.FrameNumber == this.FrameNumber;
        }

        public static bool operator ==(Video v1, Video v2) {
            if (object.ReferenceEquals(v1, null))
            {
                return object.ReferenceEquals(v2, null);
            }

            return v1.Equals(v2);
        }

        public static bool operator !=(Video v1, Video v2) {
            return !(v1 == v2);
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
