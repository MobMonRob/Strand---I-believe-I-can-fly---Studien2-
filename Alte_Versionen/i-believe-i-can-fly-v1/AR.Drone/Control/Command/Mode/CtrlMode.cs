namespace AR.Drone.Control.Command.Mode
{
    enum CtrlMode
    {
        /// <summary>
        /// Do nothing
        /// </summary>
        NO_CONTROL_MODE = 0,

        /// <summary>
        /// Requests the control configuration
        /// </summary>
        /// 
        CFG_GET_CONTROL_MODE = 4,

        /// <summary>
        /// Reset command mask in navdata
        /// </summary>
        ACK_CONTROL = 5,

        /// <summary>
        /// Requests the list of custom configuration IDs
        /// </summary>
        CUSTOM_CFG_GET_CONTROL_MODE = 6
    }
}
