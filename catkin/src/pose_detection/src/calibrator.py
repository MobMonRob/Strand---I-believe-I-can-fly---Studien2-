#!/usr/bin/env python

import rospy
import time


class Calibrator2D:
    """
    This class is used to calibrate a 2D Fuzzy Controller.
    """

    def __init__(self, publisher):
        self.publisher = publisher
        self.calibrated = False
        self.calibrating = False
        self.calibration_started_at = None
        self.final_callback = None
        self.avg_arm_length = None
        self.shoulder_distance = None

    def start_calibration(self, skeleton, final_callback):
        """
        This method call starts the calibration process and resets the current calibration, in case the method is not
        called for the first time.
        :param skeleton: first skeleton to start calibration
        :param final_callback: will be called as soon as the calibration is finished, 3 params: success (boolean),
            calibrated average arm length (float) and calibrated shoulder distance (float)
        """
        if not self.calibrated and not self.calibrating:
            rospy.logdebug('Calibration started!')
            self.final_callback = final_callback
            self.calibrating = True
            self.skeleton_changed(skeleton)
        elif self.calibrated:
            # Recalibrate in case there was already a calibration and this method is called again
            self.reset_calibration()
            self.start_calibration(skeleton, final_callback)

    def reset_calibration(self):
        """
        Stops any calibration process and resets every internal saved value, including any save calibration.
        """
        self.calibrated = False
        self.calibrating = False
        self.calibration_started_at = None
        self.final_callback = None
        self.avg_arm_length = None
        self.shoulder_distance = None

    def reset_calibration_started_at(self):
        """
        Resets the time point when the first skeleton was successfully measured.
        """
        rospy.logwarn('Calibration timer reset!')
        self.publish_status('2D_RESET_TIMER')
        self.calibration_started_at = None

    def skeleton_changed(self, skeleton):
        """
        Needs to be called for every available skeleton (frame) during the calibration process. Calibration requires 3
        seconds of consecutive successfully parsed skeletons (frames) to be successful.
        :param skeleton: skeleton to be parsed
        """
        # Check if calibrator is actually calibrating, otherwise stop further operations
        if not self.calibrating or self.calibrated:
            rospy.logdebug('Calibration not running!')
            return

        # Check skeleton for important points and transform coordinates in case skeleton is fine.
        if not skeleton.check_for_important_keypoints():
            rospy.logdebug('Missing important skeleton points!')
            self.publish_status('2D_MISSING_POINTS')
            return self.reset_calibration_started_at()
        skeleton.transform_points()

        # Check given skeleton for important metrics which do need to be calibrated
        self.avg_arm_length = skeleton.get_avg_arm_length()
        if self.avg_arm_length is None:
            rospy.logdebug('Average arm length could not be calculated!')
            return self.reset_calibration_started_at()
        self.shoulder_distance = skeleton.get_shoulder_distance()
        if self.shoulder_distance is None:
            rospy.logdebug('Shoulder distance could not be calculated!')
            return self.reset_calibration_started_at()

        # Check if user is in calibration pose
        #   => arms stretched to left and right in a 90 degree angle (looking like a T)
        hand_to_shoulder_distance = skeleton.get_hand_to_shoulder_distance()
        if hand_to_shoulder_distance is None or not (-100 < hand_to_shoulder_distance < 100):
            rospy.logdebug('Arms are not on the same level as the shoulders are!')
            self.publish_status('2D_ARMS_NOT_ON_SHOULDER_HEIGHT')
            return self.reset_calibration_started_at()
        hand_distance = skeleton.get_hand_distance()
        if hand_distance is None or hand_distance < (skeleton.get_total_arm_length() * 0.9):
            rospy.logdebug('Arms are not stretched out!')
            self.publish_status('2D_ARMS_NOT_STRETCHED')
            return self.reset_calibration_started_at()

        # Set calibration start time point if it is not already set to mark first successful calibration frame
        if self.calibration_started_at is None:
            rospy.logdebug('Starting calibration timer.')
            self.publish_status('2D_STARTED')
            self.calibration_started_at = int(time.time())
        # Stop calibration if calibration was successful for last 3 seconds
        elif int(time.time()) - self.calibration_started_at > 3:
            self.calibrating = False
            self.calibrated = True
            # Notify original caller about successful calibration
            if self.final_callback is not None:
                self.final_callback(True, self.avg_arm_length, self.shoulder_distance)
                self.publish_status('2D_FINISHED')

    def is_calibrated(self):
        """
        Returns whether calibrator is in a calibrated state.
        :return: True if calibrator is calibrated, otherwise False.
        """
        return self.calibrated

    def is_calibrating(self):
        """
        Returns whether calibrator is currently calibrating.
        :return: True if calibrator is currently calibrating, otherwise False.
        """
        return self.calibrating

    def get_calibrated_avg_arm_length(self):
        """
        Returns calibrated average arm length.
        :return: calibrated arm length in case calibration was successful, otherwise None
        """
        if self.is_calibrated():
            return self.avg_arm_length
        return None

    def get_calibrated_shoulder_distance(self):
        """
        Returns calibrated shoulder distance.
        :return: calibrated shoulder distance in case calibration was successful, otherwise None
        """
        if self.is_calibrated():
            return self.shoulder_distance
        return None

    def publish_status(self, status):
        self.publisher.publish(status = status)
