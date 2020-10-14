using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Animation;

namespace AR.Drone.Client.Controls
{
    /// <summary>
    /// Interaktionslogik für Refresh.xaml
    /// </summary>
    public partial class RefreshControl : UserControl
    {
        private Storyboard _returnToRootStory, _spinningAnimation;

        public Brush Fill
        {
            get { return (Brush)GetValue(FillProperty); }
            set { SetValue(FillProperty, value);  }
        }

        public bool IsSpinning
        {
            get { return (bool)GetValue(IsSpinningProperty); }
            set { 
                SetValue(IsSpinningProperty, value);
                if (!value)
                {
                    _returnToRootStory.Children[0].Duration = GetFittingDuration();
                    _returnToRootStory.Begin();
                }
                else 
                {
                    _spinningAnimation.Begin();
                }
            }
        }

        public static readonly DependencyProperty IsSpinningProperty = 
            DependencyProperty.Register("IsSpinning", typeof(bool), typeof(RefreshControl), new PropertyMetadata(false));

        public static readonly DependencyProperty FillProperty =
            DependencyProperty.Register("Fill", typeof(Brush), typeof(RefreshControl), new PropertyMetadata(SystemColors.ControlDarkBrush));

        private Duration GetFittingDuration()
        {
            var currentAngle = canvasRotation.Angle;
            double durationSec = Math.Abs(1d / 180d * (180d - currentAngle));
            return new Duration(TimeSpan.FromSeconds(durationSec));
        }

        public RefreshControl()
        {
            InitializeComponent();
            LayoutRoot.DataContext = this;
            _returnToRootStory = (Storyboard) this.Resources["returnToRoot"];
            _spinningAnimation = (Storyboard)this.Resources["spinningAnimation"];
        }
    }
}
