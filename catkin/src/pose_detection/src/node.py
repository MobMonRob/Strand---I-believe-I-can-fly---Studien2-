#!/usr/bin/env python

import rospy
from point import Point
from skeleton import Skeleton
from person_detection.msg import Skeleton as SkeletonMsg
from pose_detector import PoseDetector

frame = 0


def return_number(number):
    if number is None:
        return "None"
    else:
        return str(round(number, 2))


def detect_pose(skeleton):
    global frame
    frame += 1
    keypoints = {}
    for keypoint in skeleton.keypoints:
        keypoints[keypoint.index] = Point(keypoint.x, keypoint.y, keypoint.accuracy, keypoint.index, keypoint.name)
    skeleton = Skeleton(frame, keypoints, 0.3)
    if skeleton.check_for_important_keypoints():
        skeleton.transform_points()
        detector = PoseDetector(skeleton)
        pose = detector.detect_pose()
        print(pose + " (frame " + str(skeleton.frame) + " with HD " +
              return_number(detector.get_hand_distance()) + "/HG " + return_number(
            detector.get_hand_gradient()) + "/SD " +
              return_number(detector.get_shoulder_distance()) + "/HSD " + return_number(
            detector.get_hand_to_shoulder_distance()) + ")")
    else:
        print('Skeleton at frame ' + str(frame) + ' is missing some points!')


if __name__ == '__main__':
    rospy.init_node('pose_parser')
    rospy.Subscriber("person_detection", SkeletonMsg, detect_pose)
    rospy.spin()
