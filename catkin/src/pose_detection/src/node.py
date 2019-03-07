#!/usr/bin/env python

import rospy
import sys
from calibrator import Calibrator2D
from fuzzy_controller_2D import FuzzyController2D
from point import Point
from skeleton import Skeleton
from pose import Pose
from person_detection.msg import Skeleton as SkeletonMsg
from pose_detection.msg import Command as CommandMsg

calibrator_2D = Calibrator2D()
fuzzy_controller_2D = FuzzyController2D()
frame = 0
publisher = None
last_pose = None


def return_number(number):
    if number is None:
        return 'None'
    else:
        return str(round(number, 2))


def calibration_2D_finished(success, avg_arm_length, shoulder_distance):
    if success:
        rospy.loginfo('Calibration finished successfully! Average arm length: %i, shoulder distance: %i',
                      avg_arm_length, shoulder_distance)
    else:
        rospy.logwarn('Calibration could not be finished!')


def detect_pose_2D(skeleton_msg):
    global frame, calibrator_2D

    frame += 1
    keypoints = {}
    for keypoint in skeleton_msg.keypoints:
        keypoints[keypoint.index] = Point(keypoint.x, keypoint.y, keypoint.accuracy, keypoint.index, keypoint.name)
    skeleton = Skeleton(frame, keypoints, 0.3)

    # check calibration status
    if not calibrator_2D.is_calibrated() and not calibrator_2D.is_calibrating():
        calibrator_2D.start_calibration(skeleton, calibration_2D_finished)
    elif calibrator_2D.is_calibrating():
        calibrator_2D.skeleton_changed(skeleton)
    elif calibrator_2D.is_calibrated():
        if skeleton.check_for_important_keypoints():
            skeleton.transform_points()
            pose, intensity = fuzzy_controller_2D.detect_pose(skeleton)
            rospy.loginfo('%s, intensity %i (frame %i with HD %s/HG %s/SD %s/HSD %s)', pose, intensity, skeleton.frame,
                          return_number(skeleton.get_hand_distance()), return_number(
                    skeleton.get_hand_gradient()), return_number(skeleton.get_shoulder_distance()), return_number(
                    skeleton.get_hand_to_shoulder_distance()))
        else:
            publish_pose(Pose.HOLD)
            rospy.logwarn('Skeleton at frame %i is missing some important points!', frame)


def publish_pose(pose, intensity = 0):
    global last_pose, publisher

    if pose is not last_pose:
        rospy.loginfo('Pose changed to %s with intensity %i', pose, intensity)
        last_pose = pose
        publisher.publish(CommandMsg(command = pose, intensity = intensity))


if __name__ == '__main__':
    rospy.init_node('pose_detection',
                    log_level = (rospy.DEBUG if rospy.get_param('/pose_detection/debug') else rospy.ERROR))
    if rospy.get_param('/pose_detection/mode') == '2D':
        rospy.Subscriber('person_detection', SkeletonMsg, detect_pose_2D)
    else:
        rospy.logerr('Invalid mode detected! Allowed values are: \'2D\'')
        sys.exit()
    publisher = rospy.Publisher('pose_detection', CommandMsg, queue_size = 10)
    rospy.spin()
