#!/usr/bin/env python

import rospy


class Skeleton:
    def __init__(self, frame, keypoints, required_accuracy):
        # Input
        self.frame = frame
        self.keypoints = keypoints
        self.required_accuracy = required_accuracy
        # Internal Metrics
        self.shoulder_distance = None
        self.avg_arm_length = None
        # External Metrics
        self.hand_gradient = None
        self.hand_distance = None
        self.hand_to_shoulder_distance = None

    def check_for_important_keypoints(self):
        if 1 not in self.keypoints:
            rospy.logwarn('Neck not detected at frame %i!', self.frame)
            return False
        elif 1 in self.keypoints and self.keypoints[1].acc < self.required_accuracy:
            rospy.logwarn('Detected neck with low accuracy of ~%i%% at frame %i!', int(self.keypoints[1].acc * 100),
                          self.frame)
            return False
        elif 2 not in self.keypoints:
            rospy.logwarn('Right shoulder not detected at frame %i!', self.frame)
            return False
        elif 2 in self.keypoints and self.keypoints[2].acc < self.required_accuracy:
            rospy.logwarn('Detected right shoulder with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[2].acc * 100), self.frame)
            return False
        elif 3 not in self.keypoints:
            rospy.logwarn('Right elbow not detected at frame %i!', self.frame)
            return False
        elif 3 in self.keypoints and self.keypoints[3].acc < self.required_accuracy:
            rospy.logwarn('Detected right elbow with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[3].acc * 100), self.frame)
            return False
        elif 4 not in self.keypoints:
            rospy.logwarn('Right hand not detected at frame %i!', self.frame)
            return False
        elif 4 in self.keypoints and self.keypoints[4].acc < self.required_accuracy:
            rospy.logwarn('Detected right hand with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[4].acc * 100), self.frame)
            return False
        elif 5 not in self.keypoints:
            rospy.logwarn('Left shoulder not detected at frame %i!', self.frame)
            return False
        elif 5 in self.keypoints and self.keypoints[5].acc < self.required_accuracy:
            rospy.logwarn('Detected left shoulder with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[5].acc * 100), self.frame)
            return False
        elif 6 not in self.keypoints:
            rospy.logwarn('Left elbow not detected at frame %i!', self.frame)
            return False
        elif 6 in self.keypoints and self.keypoints[6].acc < self.required_accuracy:
            rospy.logwarn('Detected left elbow with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[6].acc * 100), self.frame)
            return False
        elif 7 not in self.keypoints:
            rospy.logwarn('Left hand not detected at frame %i!', self.frame)
            return False
        elif 7 in self.keypoints and self.keypoints[7].acc < self.required_accuracy:
            rospy.logwarn('Detected left hand with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[7].acc * 100), self.frame)
            return False
        elif 8 not in self.keypoints:
            rospy.logwarn('Hip not detected at frame %i!', self.frame)
            return False
        elif 8 in self.keypoints and self.keypoints[8].acc < self.required_accuracy:
            rospy.logwarn('Detected hip with low accuracy of ~%i%% at frame %i!', int(self.keypoints[8].acc * 100),
                          self.frame)
            return False
        return True

    def transform_points(self):
        for key in self.keypoints:
            if self.keypoints[key].index is not 8:
                self.keypoints[key].x = self.keypoints[key].x - self.keypoints[8].x
                self.keypoints[key].y = self.keypoints[8].y - self.keypoints[key].y
        self.keypoints[8].x = 0
        self.keypoints[8].y = 0

    def calc_shoulder_distance(self):
        # Include neck in calculation and do not use direct distance between shoulders
        if self.keypoints[1] is not None and self.keypoints[5] is not None and self.keypoints[2] is not None:
            self.shoulder_distance = self.keypoints[1].distance_to(self.keypoints[2]) + self.keypoints[1].distance_to(
                self.keypoints[5])
        else:
            self.shoulder_distance = None

    def get_shoulder_distance(self):
        if self.shoulder_distance is None:
            self.calc_shoulder_distance()
        return self.shoulder_distance

    def calc_avg_arm_length(self):
        # check existence of both arms [hand, elbow, shoulder] and neck (keypoints 1 - 7)
        for i in range(1, 8):
            if self.keypoints[i] is None:
                self.avg_arm_length = None
                return
        # Average arm length calculation can not be done via loop as the keypoints 4 and 5 are no direct neighbours in
        # the skeleton.
        right_arm_length = self.keypoints[2].distance_to(self.keypoints[3]) + self.keypoints[3].distance_to(
            self.keypoints[4])
        left_arm_length = self.keypoints[5].distance_to(self.keypoints[6]) + self.keypoints[6].distance_to(
            self.keypoints[7])
        self.avg_arm_length = int((right_arm_length + left_arm_length) / 2)

    def get_avg_arm_length(self):
        if self.avg_arm_length is None:
            self.calc_avg_arm_length()
        return self.avg_arm_length

    def get_total_arm_length(self):
        avg_arm_length = self.get_avg_arm_length()
        shoulder_distance = self.get_shoulder_distance()
        if avg_arm_length is not None and shoulder_distance is not None:
            return 2 * avg_arm_length + shoulder_distance
        return None

    def calc_hand_gradient(self):
        if self.keypoints[4] is not None and self.keypoints[7] is not None:
            self.hand_gradient = self.keypoints[7].gradient_to(self.keypoints[4])
        else:
            self.hand_gradient = None

    def get_hand_gradient(self):
        if self.hand_gradient is None:
            self.calc_hand_gradient()
        return self.hand_gradient

    def calc_hand_distance(self):
        if self.keypoints[4] is not None and self.keypoints[7] is not None:
            self.hand_distance = self.keypoints[7].distance_to(self.keypoints[4])
        else:
            self.hand_distance = None

    def get_hand_distance(self):
        if self.hand_distance is None:
            self.calc_hand_distance()
        return self.hand_distance

    def calc_hand_to_shoulder_distance(self):
        if self.keypoints[5] is not None and self.keypoints[2] is not None and \
                self.keypoints[7] is not None and self.keypoints[4] is not None:
            right_distance = self.keypoints[2].y - self.keypoints[4].y
            left_distance = self.keypoints[5].y - self.keypoints[7].y
            self.hand_to_shoulder_distance = (right_distance + left_distance) / 2
        else:
            self.hand_to_shoulder_distance = None

    def get_hand_to_shoulder_distance(self):
        if self.hand_to_shoulder_distance is None:
            self.calc_hand_to_shoulder_distance()
        return self.hand_to_shoulder_distance
