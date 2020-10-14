using AR.Drone.NavData.Data;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;

namespace AR.Drone.Client.HUD.Controls
{
    public abstract class HUDControl : UserControl
    {
        private bool _handlerSet = false;

        public NavigationData _Navdata;
        public NavigationData Navdata
        {
            get { return _Navdata; }
            set { _Navdata = value;  SetHandler(); }
        }

        /// <summary>
        /// This method is called when the navdata property changed handler is attached
        /// </summary>
        protected abstract void OnHandlerReady();

        /// <summary>
        /// The handler method for the NavdatapPropertyChanged-Event
        /// </summary>
        /// <param name="sender">The object that fired the event</param>
        /// <param name="e">Event arguments</param>
        protected abstract void Navdata_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e);

        private void SetHandler()
        {
            if (!_handlerSet)
            {
                this.DataContext = _Navdata;
                _handlerSet = true;
                Navdata.PropertyChanged += Navdata_PropertyChanged;
                OnHandlerReady();
            }
        }
    }
}
