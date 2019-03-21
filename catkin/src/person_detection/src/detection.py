#!/usr/bin/env python

import cv2
import rospy
import os
from videostream import VideoStream
from openpose_runner import OpenPoseRunner
from util import is_empty_or_none
from person_detection.msg import Keypoint as KeypointMsg
from person_detection.msg import Skeleton as SkeletonMsg


class Detection:
    """
    Used to manage person detection actions like converting keypoints to different formats and outputting the detected
    keypoints to other ROS nodes or local files.
    """

    def __init__(self, video_source, config, publisher):
        self.config = config
        # debugging only
        if self.config['debugging']:
            if not os.path.exists(self.config['debugging_output'] + '/images'):
                os.makedirs(self.config['debugging_output'] + '/images')
            self.debugging_index = 0
            self.file = open(self.config['debugging_output'] + '/log', 'w')
            self.file.write('[\n')

        # ROS
        self.publisher = publisher

        # OpenPose
        self.stream = VideoStream(video_source)
        self.openpose_runner = OpenPoseRunner(self.config, self.stream, self.image_analyzed_callback)
        self.openpose_runner.start()

    def convert_keypoint_to_message(self, index, keypoint):
        """
        Converts a single keypoint to a ROS message.
        :param index: index of the keypoint
        :param keypoint: array containing the x/y coordinates and the measured accuracy of this keypoint
        :return: ROS message (type Keypoint) containing the keypoint details
        """
        return KeypointMsg(name = self.config['body_mapping'][index], index = index, x = keypoint[0], y = keypoint[1],
                           accuracy = keypoint[2])

    def convert_keypoints_to_message(self, keypoints):
        """
        Converts multiple keypoints to a ROS message.
        :param keypoints: keypoints to be converted
        :return: ROS message (type Skeleton) containing all keypoints
        """
        converted_keypoints = []
        # TODO: Check behaviour with multiple persons on the image. Which index is assigned to which person? Does the
        #  same person always have the same index?
        if not is_empty_or_none(keypoints) and not is_empty_or_none(keypoints[0]):
            for index, keypoint in enumerate(keypoints[0]):
                if keypoint[0] != 0 or keypoint[1] != 0 or keypoint[2] != 0:
                    converted_keypoints.append(self.convert_keypoint_to_message(index, keypoint))
        return SkeletonMsg(keypoints = converted_keypoints)

    def convert_keypoints_to_json(self, keypoints):
        """
        Only used for debugging purposes.
        Converts multiple keypoints to JSON format.
        :param keypoints: keypoints to be converted to JSON
        :return: JSON containing all keypoints
        """
        points = ''
        if not is_empty_or_none(keypoints) and not is_empty_or_none(keypoints[0]):
            for index, keypoint in enumerate(keypoints[0]):
                if keypoint[0] != 0 or keypoint[1] != 0 or keypoint[2] != 0:
                    if points is not '':
                        points += ','
                    rospy.logdebug(keypoint)
                    points += '{"part":{%d},"description":"{}","x":{%f},"y":{%f},"accuracy":{%f}}' \
                        .format(index, self.config['body_mapping'][index], keypoint[0], keypoint[1], keypoint[2])
        return '{"index":{},"points":[{}]}'.format(
            (str(self.debugging_index) if self.config['debugging'] else '"unknown"'), points)

    def show_skeleton(self, image):
        """
        Only used for debugging or demo purposes.
        Shows an image to the user. In case users presses the 'q' key, program will stop further person detection and
        shuts down ROS node.
        :param image: image to be shown
        """
        cv2.namedWindow('Output', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Output', 1800, 1000)
        cv2.putText(image, 'Press q to quit the program!',
                    (0, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1)
        cv2.imshow('Output', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.openpose_runner.stop()

    def publish(self, skeleton):
        """
        Publishes skeleton on the configured publisher.
        :param skeleton: skeleton to be published (type Skeleton)
        """
        self.publisher.publish(skeleton)

    def image_analyzed_callback(self, keypoints, image):
        """
        Used as callback when neutral net finishes processing of a single frame. Transforms the detected keypoints,
        publishes them to other ROS nodes and shows them to the user in case the node is started in debugging mode.
        :param keypoints: detected keypoints
        :param image: image containing the analyzed (original) image and the detected skeleton as an overlay
        """
        # debugging only
        if self.config['debugging']:
            if not is_empty_or_none(keypoints) and not is_empty_or_none(keypoints[0]):
                cv2.imwrite(self.config['debugging_output'] + '/images/' + str(self.debugging_index) + '.jpg', image)
                keypoints_json = self.convert_keypoints_to_json(keypoints)
                self.file.write(keypoints_json if self.debugging_index is 0 else ',\n' + keypoints_json)
                self.debugging_index += 1

        self.publish(self.convert_keypoints_to_message(keypoints))
        if self.config['debugging'] or self.config['show_skeleton']:
            self.show_skeleton(image)

    def shutdown(self):
        """
        Shuts down ROS node and stops further person detection.
        """
        # debugging only
        if self.config['debugging']:
            self.file.write(']')
            self.file.close()

        self.openpose_runner.stop()
        rospy.signal_shutdown('Shutdown initialized by user')
