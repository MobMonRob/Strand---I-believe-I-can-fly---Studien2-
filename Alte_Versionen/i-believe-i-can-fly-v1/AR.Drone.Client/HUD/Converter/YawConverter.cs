using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;

namespace AR.Drone.Client.HUD.Converter
{
    class YawConverter : IValueConverter
    {
        //private const float TEN_PERCENT_ANGLE = 40;
        private const float ONE_PERCENT_ANGLE = 4;
        private const float MAX_MOVEMENT_POSITIVE = 840;
        private const float MAX_MOVEMENT_NEGATIVE = -840;
        private const float ROUNDJUMP_360 = 360 * ONE_PERCENT_ANGLE;

        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            float fValue = -(float)value % 360 * ONE_PERCENT_ANGLE;
            if (fValue > MAX_MOVEMENT_POSITIVE)
            {
                fValue -= ROUNDJUMP_360;
            }
            else if (fValue < MAX_MOVEMENT_NEGATIVE)
            {
                fValue += ROUNDJUMP_360;
            }
            return fValue;
            //throw new NotImplementedException();
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
