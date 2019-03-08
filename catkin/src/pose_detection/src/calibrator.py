#!/usr/bin/env python

import rospy
import time


class Calibrator2D:
    def __init__(self):
        self.calibrated = False
        self.calibrating = False
        self.calibration_started_at = None
        self.final_callback = None
        self.avg_arm_length = None
        self.shoulder_distance = None

    def start_calibration(self, skeleton, final_callback):
        if not self.calibrated and not self.calibrating:
            self.final_callback = final_callback
            self.calibrating = True
            rospy.logdebug('Calibration started!')
            self.skeleton_changed(skeleton)
        elif self.calibrated:
            # Recalibrate in case there was already a calibration and this method is called again
            self.reset_calibration()
            self.start_calibration(skeleton, final_callback)

    def reset_calibration(self):
        self.calibrated = False
        self.calibrating = False
        self.calibration_started_at = None
        self.final_callback = None
        self.avg_arm_length = None
        self.shoulder_distance = None

    def reset_calibration_started_at(self):
        rospy.loginfo('Calibration timer reset!')
        self.calibration_started_at = None

    def skeleton_changed(self, skeleton):
        # Check if calibrator is actually calibrating, otherwise stop further operations
        if not self.calibrating or self.calibrated:
            rospy.logdebug('Calibration not running!')
            return

        # Check skeleton for important points and transform coordinates in case skeleton is fine.
        if not skeleton.check_for_important_keypoints():
            rospy.logdebug('Missing important skeleton points!')
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
            return self.reset_calibration_started_at()
        hand_distance = skeleton.get_hand_distance()
        if hand_distance is None or hand_distance < (skeleton.get_total_arm_length() * 0.9):
            rospy.logdebug('Arms are not stretched out!')
            return self.reset_calibration_started_at()

        # Set calibration start time point if it is not already set to mark first successful calibration frame
        if self.calibration_started_at is None:
            rospy.logdebug('Starting calibration timer.')
            self.calibration_started_at = int(time.time())
        # Stop calibration if calibration was successful for last 3 seconds
        elif int(time.time()) - self.calibration_started_at > 3:
            self.calibrating = False
            self.calibrated = True
            # Notify original caller about successful calibration
            if self.final_callback is not None:
                self.final_callback(True, self.avg_arm_length, self.shoulder_distance)

    def is_calibrated(self):
        return self.calibrated

    def is_calibrating(self):
        return self.calibrating

    def get_calibrated_avg_arm_length(self):
        if self.is_calibrated():
            return self.avg_arm_length
        return None

    def get_calibrated_shoulder_distance(self):
        if self.is_calibrated():
            return self.shoulder_distance
        return None
