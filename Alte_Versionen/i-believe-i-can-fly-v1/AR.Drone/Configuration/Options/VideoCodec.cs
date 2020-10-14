enum VideoCodec : int
{
    /// <summary>
    /// Live stream with MPEG4.2 soft encoder. No record stream.
    /// </summary>
    MP4_360P_CODEC = 0x80,

    /// <summary>
    /// Live stream with H264 hardware encoder configured in 360p mode. No record stream.
    /// </summary>
    H264_360P_CODEC = 0x81,

    /// <summary>
    /// Live stream with MPEG4.2 soft encoder. Record stream with H264 hardware 
    /// encoder in 720p mode.
    /// </summary>
    MP4_360P_H264_720P_CODEC = 0x82,
    
    /// <summary>
    /// Live stream with H264 hardware encoder configured in 720p mode. No record stream
    /// </summary>
    H264_720P_CODEC = 0x83,

    /// <summary>
    /// Live stream with MPEG4.2 soft encoder. Record stream with H264 hardware encoder in 360p mode.
    /// </summary>
    MP4_360P_H264_360P_CODEC = 0x88
}