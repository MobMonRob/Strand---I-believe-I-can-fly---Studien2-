using AR.Drone.Client.CalibrationBinding;
using AR.Drone.Kinect.Ressources;
using Microsoft.Kinect;
using System;
using System.Threading;

namespace AR.Drone.Kinect
{
    class Calibrater
    {
        private int _calibrateCounterArmLength;
        private int _calibrateCounterBackTilt;
        public CalibrationDataSource calibrationDataSource
        {
            get;
            private set;
        }
        public double armLength { get; private set; }
        public double backTilt { get; private set; }
        public bool armLengthCalibrated { get; private set; }
        public bool backTiltCalibrated { get; private set; }
        public bool hasCalibrationStarted { get; private set; }
        public bool hasCalibrationFinished { get; set; }

        public Calibrater(CalibrationDataSource calibrationDataSource)
        {
            this.hasCalibrationStarted = false;
            this.hasCalibrationFinished = false;
            this.armLength = 1.0;
            this.backTilt = 0.0;
            this._calibrateCounterArmLength = 0;
            this._calibrateCounterBackTilt = 0;
            this.calibrationDataSource = calibrationDataSource;
        }

        public void Start()
        {
            this.hasCalibrationStarted = true;
            this.hasCalibrationFinished = false;
            this.armLengthCalibrated = false;
            this.backTiltCalibrated = false;
            calibrationDataSource.ArmLengthStatus = Status.Waiting.ToString();
            calibrationDataSource.BackTiltStatus  = Status.Waiting.ToString();
        }

        public void DoCalibration(JointCollection joints)
        {
            this.CalibrateArmLength(joints);
            this.CalibrateBackTilt(joints);
        }

        private void CalibrateArmLength(JointCollection joints)
        {
            this._calibrateCounterArmLength++;
            // Calibration task for arm length completed
            if (this._calibrateCounterArmLength >= Constants.CALIBRATE_LIMIT)
            {
                calibrationDataSource.ArmLengthStatus = Status.Checked.ToString();
                this.armLengthCalibrated = true;
                if(this.backTiltCalibrated)
                    FinishCalibration();
            }
            else
            {
                this.MeasureCalibrateValueArmLength(joints);
            }
        }

        private void CalibrateBackTilt(JointCollection joints)
        {
            this._calibrateCounterBackTilt++;
            // Calibration task for back tilt completed
            if (this._calibrateCounterBackTilt >= Constants.CALIBRATE_LIMIT)
            {
                calibrationDataSource.BackTiltStatus = Status.Checked.ToString();
                this.backTiltCalibrated = true;
                if (this.armLengthCalibrated)
                    FinishCalibration();
            }
            else
            {
                this.MeasureCalibrateValueBackTilt(joints);
            }
        }

        private void FinishCalibration()
        {

            this.armLengthCalibrated = false;
            this.backTiltCalibrated = false;
            this.hasCalibrationStarted = false;
            this.hasCalibrationFinished = true;
            this._calibrateCounterArmLength = 0;
            this._calibrateCounterBackTilt = 0;
            
        }

        private double GetNewArmlength(JointCollection joints)
        {
            SkeletonPoint leftShoulderPosition = joints[JointType.ShoulderLeft].Position;
            SkeletonPoint rightShoulderPosition = joints[JointType.ShoulderRight].Position;
            SkeletonPoint leftHandPosition = joints[JointType.HandLeft].Position;
            SkeletonPoint rightHandPosition = joints[JointType.HandRight].Position;
            return (Math.Abs(leftShoulderPosition.X - leftHandPosition.X) +
                Math.Abs(leftShoulderPosition.Y - leftHandPosition.Y) +
                Math.Abs(rightShoulderPosition.X - rightHandPosition.X) +
                Math.Abs(rightShoulderPosition.Y - rightHandPosition.Y));
        }

        private double GetNewBackTiltValue(JointCollection joints)
        {
            return joints[JointType.ShoulderCenter].Position.Z - joints[JointType.HipCenter].Position.Z;
        }

        private void MeasureCalibrateValueArmLength(JointCollection joints)
        {
            double newCalibrateValueArmLength = this.GetNewArmlength(joints);
            // Tracking too fuzzy (continue measuring)
            if (this._calibrateCounterArmLength > 1 && Math.Abs(this.armLength - newCalibrateValueArmLength) > Constants.TOLERANCE_CALIBRATING)
            {
                this._calibrateCounterArmLength = 0;
            }
            else
            {
                this.armLength = newCalibrateValueArmLength;
            }
        }

        private void MeasureCalibrateValueBackTilt(JointCollection joints)
        {
            double newCalibrateValueBackTilt = this.GetNewBackTiltValue(joints);
            // Tracking too fuzzy (continue measuring)
            if (this._calibrateCounterBackTilt > 1 && Math.Abs(this.backTilt - newCalibrateValueBackTilt) > Constants.TOLERANCE_CALIBRATING)
            {
                this._calibrateCounterBackTilt = 0;
            }
            else
            {
                this.backTilt = newCalibrateValueBackTilt;
            }
        }
    }
}
