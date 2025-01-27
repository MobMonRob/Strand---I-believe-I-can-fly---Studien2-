﻿using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace AR.Drone.Kinect
{
    /// <summary>
    /// Converts a Kinect frame into an HTML5 blob.
    /// </summary>
    public static class FrameSerializer
    {
        /// <summary>
        /// Converts a WriteableBitmap into a byte array.
        /// </summary>
        /// <param name="bitmap">The specified bitmap.</param>
        /// <param name="file">The specified temporary file.</param>
        /// <returns>A binary representation of the bitmap.</returns>
        public static byte[] CreateBlob(WriteableBitmap bitmap, string file)
        {
            // Save bitmap.
            BitmapEncoder encoder = new JpegBitmapEncoder();

            encoder.Frames.Add(BitmapFrame.Create(bitmap as BitmapSource));

            using (var stream = new MemoryStream())
            {
                encoder.Save(stream);
                return stream.ToArray();
            }
        }
    }
}
