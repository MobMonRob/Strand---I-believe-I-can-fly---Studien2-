using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using AR.Drone.Client.CalibrationBinding;

namespace AR.Drone.Client.Controls
{
    /// <summary>
    /// Interaktionslogik für DoneControl.xaml
    /// </summary>
    public partial class DoneControl : UserControl
    {
        public DoneControl()
        {
            InitializeComponent();
        }
        public static readonly DependencyProperty StatusProperty =
            DependencyProperty.Register("Status", typeof(String), typeof(DoneControl), new PropertyMetadata(CalibrationBinding.Status.Initialized.ToString()));

        public String Status
        {
            get { return (String)GetValue(StatusProperty); }
            set
            {
                SetValue(StatusProperty, value);
                if (value.Equals(CalibrationBinding.Status.Initialized.ToString()))
                {
                    this.InitializeStatus();
                }
                else if (value.Equals(CalibrationBinding.Status.Waiting.ToString()))
                {
                    this.ToStatusWaiting();
                }
                else if (value.Equals(CalibrationBinding.Status.Checked.ToString()))
                {
                    this.ToStatusChecked();
                }
            }
        }

        private void InitializeStatus()
        {
            refreshCtrl.Fill = (Brush)new BrushConverter().ConvertFrom("#00A0A0A0");
            checkMark.Background = (Brush)new BrushConverter().ConvertFrom("#00007800");
            checkMark.Foreground = (Brush)new BrushConverter().ConvertFrom("#0000C800");
        }

        private void ToStatusWaiting()
        {
            refreshCtrl.IsSpinning = true;
            refreshCtrl.Fill = (Brush)new BrushConverter().ConvertFrom("#FFA0A0A0");
            checkMark.Background = (Brush)new BrushConverter().ConvertFrom("#FF007800");
            checkMark.Foreground = (Brush)new BrushConverter().ConvertFrom("#0000C800");
        }

        private void ToStatusChecked()
        {
            refreshCtrl.Fill = (Brush)new BrushConverter().ConvertFrom("#00A0A0A0");
            checkMark.Background = (Brush)new BrushConverter().ConvertFrom("#FF007800");
            checkMark.Foreground = (Brush)new BrushConverter().ConvertFrom("#FF00C800");
        }
    }
}
