
namespace AR.Drone.Headtracker
{
    public class HeadTrackerErrorState
    {
        public bool hasError{ get; internal set; }
        public string stateText { get; internal set; }

        internal HeadTrackerErrorState(bool hasError, string stateText)
        {
            this.hasError = hasError;
            this.stateText = stateText;
        }
        internal HeadTrackerErrorState() : this(false, "") { }
    }
}
