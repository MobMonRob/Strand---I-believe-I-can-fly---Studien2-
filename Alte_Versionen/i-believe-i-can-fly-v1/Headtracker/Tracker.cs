// Created: 2013-04-06
// Author: Frederik Schaaf
// Copyright 2012-2013 Inreal Technologies
// eMail: headtracker@inreal-tech.com
// License terms apply (see tracker_sdk_license.txt)

using System;
using System.Runtime.InteropServices;
using System.Text;

namespace AR.Drone.Headtracker
{

    internal struct Vec3
    {
        public Vec3(double px, double py, double pz)
        {
            x = (float)px;
            y = (float)py;
            z = (float)pz;
        }

        public float x, y, z;
        public static Vec3 operator +(Vec3 l, Vec3 r)
        {
            return new Vec3(l.x + r.x, l.y + r.y, l.z + r.z);
        }
        public static Vec3 operator -(Vec3 l, Vec3 r)
        {
            return new Vec3(l.x - r.x, l.y - r.y, l.z - r.z);
        }

        public Vec3 Scale(double r)
        {
            return new Vec3(x * r, y * r, z * r);
        }

        public double Magn()
        {
            return Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2));
        }

        public double Dot(Vec3 r)
        {
            return x * r.x + y * r.y + z * r.z;
        }

        public Vec3 Cross(Vec3 r)
        {
            return new Vec3(y * r.z - z * r.y, z * r.x - x * r.z, x * r.y - y * r.x);
        }
    }

    internal struct Quat
    {
        public Quat(double pw, Vec3 pv)
        {
            w = (float)pw;
            v = pv;
        }

        static public Quat FromVec(Vec3 v)
        {
            return new Quat(0, v);
        }

        public float w;
        public Vec3 v;

        public static Quat operator *(Quat l, Quat r)
        {
            return new Quat(l.w * r.w - l.v.x * r.v.x - l.v.y * r.v.y - l.v.z * r.v.z,
                            new Vec3(l.w * r.v.x + l.v.x * r.w + l.v.y * r.v.z - l.v.z * r.v.y,
                            l.w * r.v.y + l.v.y * r.w + l.v.z * r.v.x - l.v.x * r.v.z,
                            l.w * r.v.z + l.v.z * r.w + l.v.x * r.v.y - l.v.y * r.v.x)
                    );

        }

        public Quat GetInverse()
        {
            return new Quat(-w, v);
        }

        public double Magn()
        {
            return Math.Sqrt(Math.Pow(w, 2) + Math.Pow(v.Magn(), 2));
        }

        public Quat GetNormalized()
        {
            double magn = Magn();
            if (magn == 0) return new Quat(1, new Vec3(0, 0, 0));
            return new Quat(w / magn, v.Scale(1 / magn));
        }

    }

    static class Tracker
    {
        static Tracker() 
        { 
            string trackerDllPath = "lib/" + (Environment.Is64BitProcess ? "x64" : "x86");
            NativeMethods.SetDllDirectory(trackerDllPath);
        }

        public struct Frame
        {
            public Vec3 Acc, Gyr, Mag, MagCalib;
            public Quat Rot;
            public long FrameNumber;
        }

        public struct Euler
        {
            public float Yaw, Pitch, Roll;
        }

        [DllImport("Tracker.dll", EntryPoint = "Tracker_Init", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Init();

        [DllImport("Tracker.dll", EntryPoint = "Tracker_Release", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool Release();

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetState", CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetState(ref int State);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_WaitNextFrame", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool WaitNextFrame(UInt32 waitMilliSeconds = 100);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetFrame", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetFrame(ref Frame frame);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_RotateTrackerToCinemizer", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RotateTrackerToCinemizer(ref Quat cinemizerQuat, Quat trackerQuat);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetMouseSpeed", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetMouseSpeed(ref Byte Speed, UInt32 waitMilliSeconds = 100);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_SetMouseSpeed", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetMouseSpeed(Byte Speed);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetFirmware", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetFirmware(ref UInt32 Firmware, UInt32 waitMilliSeconds = 100);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_SetBootloaderMode", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetBootloaderMode(bool activationMode, bool waitForDeviceToAppear = true, UInt32 waitDeviceMilliSeconds = 10000);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetLastError", CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetLastError();

        [DllImport("Tracker.dll", EntryPoint = "Tracker_QuatGetEuler", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool QuatGetEuler(ref Euler euler, Quat quat);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_VecRotate", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool VecRotate(ref Vec3 vec3, Quat quat);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetErrorString", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetErrorString(StringBuilder trackerErrorString, int TrackerError);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetFirmwareString", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetFirmwareString(StringBuilder trackerFirmwareString, UInt32 Firmware);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_StringTest", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool StringTest(StringBuilder lpBuffer);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetCinemizerRotatedFrame", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetCinemizerRotatedFrame(ref Frame frame);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_CinemizerApplyAutoRollAdjust", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CinemizerApplyAutoRollAdjust();

        [DllImport("Tracker.dll", EntryPoint = "Tracker_CinemizerApplyAutoYawAdjust", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CinemizerApplyAutoYawAdjust();

        [DllImport("Tracker.dll", EntryPoint = "Tracker_SetMagnetometerFusion", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetMagnetometerFusion(bool activationMode);

        [DllImport("Tracker.dll", EntryPoint = "Tracker_GetMagnetometerFusion", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool GetMagnetometerFusion();


    }
}
