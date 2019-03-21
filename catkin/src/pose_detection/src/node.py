#!/usr/bin/env python

import rospy
import sys
from calibrator import Calibrator2D
from fuzzy_controller_2D import FuzzyController2D
from point import Point
from skeleton import Skeleton
from pose import Pose
from person_detection.msg import Skeleton as SkeletonMsg
from pose_detection.msg import Calibration as CalibrationMsg
from pose_detection.msg import Instruction as InstructionMsg
from pose_detection.msg import Instructions as InstructionsMsg

publisher_calibration = None
publisher_instructions = None
calibrator_2D = None
fuzzy_controller_2D = None
frame = 0


def return_number(number):
    """
    Returns number as string. If number is None, string 'None' is returned instead.
    :param number: number to be converted as string
    :return: number converted to string and round to 2 positions after decimal point or 'None' in case of given None
    """
    if number is None:
        return 'None'
    else:
        return str(round(number, 2))


def calibration_2D_finished(success, avg_arm_length, shoulder_distance):
    """
    Callback for the finished calibration process. Forwards the calibrated values to the actual Fuzzy Controller.
    :param success: True if calibration was successful, otherwise False
    :param avg_arm_length: calibrated average arm length
    :param shoulder_distance: calibrated shoulder distance
    """
    global fuzzy_controller_2D

    if success and fuzzy_controller_2D.calibrate(avg_arm_length, shoulder_distance):
        rospy.loginfo('Calibration finished successfully! Average arm length: %i, shoulder distance: %i',
                      avg_arm_length, shoulder_distance)
    else:
        rospy.logwarn('Calibration could not be finished!')


def detect_pose_2D(skeleton_msg):
    """
    Controls the actual program flow from the calibration to the actual flight instruction publication.
    :param skeleton_msg: Received skeleton message from the ROS node 'person_detection'.
    """
    global frame, calibrator_2D

    frame += 1
    keypoints = dict()
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
            instructions = fuzzy_controller_2D.detect_pose(skeleton)
            publish_instructions(instructions)
            rospy.logdebug('%s, (frame %i with HD %s/HG %s/SD %s/HSD %s)', str(instructions), skeleton.frame,
                           return_number(skeleton.get_hand_distance()), return_number(
                    skeleton.get_hand_gradient()), return_number(skeleton.get_shoulder_distance()), return_number(
                    skeleton.get_hand_to_shoulder_distance()))
        else:
            publish_instructions({Pose.HOLD: 100.0})
            rospy.logwarn('Skeleton at frame %i is missing some important points!', frame)


def publish_instructions(instructions):
    """
    Publishes given flight instructions.
    :param instructions: flight instructions to publish
    """
    global publisher_instructions

    if instructions is None:
        return

    converted_instructions = list()
    for key in instructions:
        converted_instructions.append(InstructionMsg(instruction = key, intensity = instructions[key]))
    publisher_instructions.publish(InstructionsMsg(instructions = converted_instructions))


if __name__ == '__main__':
    rospy.init_node('pose_detection',
                    log_level = (rospy.DEBUG if rospy.get_param('/pose_detection/debug') else rospy.ERROR))
    publisher_instructions = rospy.Publisher('flight_instructions', InstructionsMsg, queue_size = 10)
    publisher_calibration = rospy.Publisher('calibration_status', CalibrationMsg, queue_size = 10)
    calibrator_2D = Calibrator2D(publisher_calibration)
    fuzzy_controller_2D = FuzzyController2D()

    if rospy.get_param('/pose_detection/mode') == '2D':
        rospy.Subscriber('person_detection', SkeletonMsg, detect_pose_2D)
    else:
        rospy.logerr('Invalid mode detected! Allowed values are: \'2D\'')
        sys.exit()
    rospy.spin()
