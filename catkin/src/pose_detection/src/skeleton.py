#!/usr/bin/env python

import rospy


class Skeleton:
    def __init__(self, frame, keypoints, required_accuracy):
        self.frame = frame
        self.keypoints = keypoints
        self.required_accuracy = required_accuracy

    def check_for_important_keypoints(self):
        if 8 not in self.keypoints:
            rospy.logwarn('Hip not detected at frame %i!', self.frame)
            return False
        elif 8 in self.keypoints and self.keypoints[8].acc < self.required_accuracy:
            rospy.logwarn('Detected hip with low accuracy of ~%i%% at frame %i!', int(self.keypoints[8].acc * 100),
                          self.frame)
            return False
        elif 2 not in self.keypoints:
            rospy.logwarn('Right shoulder not detected at frame %i!', self.frame)
            return False
        elif 2 in self.keypoints and self.keypoints[2].acc < self.required_accuracy:
            rospy.logwarn('Detected right shoulder with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[2].acc * 100), self.frame)
            return False
        elif 5 not in self.keypoints:
            rospy.logwarn('Left shoulder not detected at frame %i!', self.frame)
            return False
        elif 5 in self.keypoints and self.keypoints[5].acc < self.required_accuracy:
            rospy.logwarn('Detected left shoulder with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[5].acc * 100), self.frame)
            return False
        elif 4 not in self.keypoints:
            rospy.logwarn('Right hand not detected at frame %i!', self.frame)
            return False
        elif 4 in self.keypoints and self.keypoints[4].acc < self.required_accuracy:
            rospy.logwarn('Detected right hand with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[4].acc * 100), self.frame)
            return False
        elif 7 not in self.keypoints:
            rospy.logwarn('Left hand not detected at frame %i!', self.frame)
            return False
        elif 7 in self.keypoints and self.keypoints[7].acc < self.required_accuracy:
            rospy.logwarn('Detected left hand with low accuracy of ~%i%% at frame %i!',
                          int(self.keypoints[7].acc * 100), self.frame)
            return False
        return True

    def transform_points(self):
        for key in self.keypoints:
            if self.keypoints[key].index is not 8:
                self.keypoints[key].x = self.keypoints[key].x - self.keypoints[8].x
                self.keypoints[key].y = self.keypoints[8].y - self.keypoints[key].y
        self.keypoints[8].x = 0
        self.keypoints[8].y = 0
