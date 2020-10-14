namespace AR.Drone.Control.Command.Mode
{
    /// <summary>
    /// These are the available basic commands
    /// </summary>
    enum BasicCommandMode
    {
        DEFAULT = 1 << 18 | 1 << 20 | 1 << 22 | 1 << 24 | 1 << 28,
        TAKE_OFF = 1 << 9 | DEFAULT,
        LAND = 0 << 9 | DEFAULT,
        EMERGENCY = 1 << 8 | DEFAULT
    };
}