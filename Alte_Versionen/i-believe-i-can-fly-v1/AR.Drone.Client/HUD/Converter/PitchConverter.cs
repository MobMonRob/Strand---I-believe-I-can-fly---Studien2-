using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;

namespace AR.Drone.Client.HUD.Converter
{
    class PitchConverter : IValueConverter
    {
        private const float MAX_TRANSLATION = 184;
        private const float MAX_DEGREE = 90;
        private const float DEGREE_PER_PIXEL = MAX_TRANSLATION / MAX_DEGREE;

        private const float STANDARD_TRANSLATION = -120;

        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return STANDARD_TRANSLATION + DEGREE_PER_PIXEL * (float)value;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
