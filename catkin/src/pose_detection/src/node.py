#!/usr/bin/env python

import rospy
from point import Point
from skeleton import Skeleton
from pose import Pose
from pose_detector import PoseDetector
from person_detection.msg import Skeleton as SkeletonMsg
from pose_detection.msg import Command as CommandMsg

frame = 0
publisher = None
last_pose = None


def return_number(number):
    if number is None:
        return 'None'
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
        publish_pose(pose)
        rospy.loginfo('%s (frame %i with HD %s/HG %s/SD %s/HSD %s)', pose, skeleton.frame,
                      return_number(detector.get_hand_distance()), return_number(
                detector.get_hand_gradient()), return_number(detector.get_shoulder_distance()), return_number(
                detector.get_hand_to_shoulder_distance()))

    else:
        publish_pose(Pose.HOLD)
        rospy.logwarn('Skeleton at frame %i is missing some points!', frame)


def publish_pose(pose):
    global last_pose, publisher
    if pose is not last_pose:
        rospy.loginfo('Pose changed to %s', pose)
        last_pose = pose
        publisher.publish(CommandMsg(command = pose))


if __name__ == '__main__':
    rospy.init_node('pose_parser')
    rospy.Subscriber('person_detection', SkeletonMsg, detect_pose)
    publisher = rospy.Publisher('pose_detection', CommandMsg, queue_size = 10)
    rospy.spin()
