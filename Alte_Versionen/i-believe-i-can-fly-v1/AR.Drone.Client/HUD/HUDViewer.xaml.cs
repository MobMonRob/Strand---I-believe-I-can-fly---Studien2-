using AR.Drone.NavData.Data;
using Microsoft.Kinect;
using System.Timers;
using System.Windows;
using System.Windows.Controls;

namespace AR.Drone.Client.HUD
{
    /// <summary>
    /// Interaction logic for HUDViewer.xaml
    /// </summary>
    public partial class HUDViewer : UserControl
    {
        public HUDViewer()
        {
            InitializeComponent();
        }

        internal void SetKinectSensor(KinectSensor sensor)
        {
            kinectDepth.Initialize(sensor);
        }

        internal void Destruct() 
        {
            kinectDepth.Destruct();
        }
    }
}
