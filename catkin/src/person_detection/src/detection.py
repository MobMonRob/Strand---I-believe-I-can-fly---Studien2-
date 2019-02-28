#!/usr/bin/env python

import cv2
import rospy
import os
from videostream import VideoStream
from openpose_runner import OpenPoseRunner
from util import is_empty_or_none
from person_detection.msg import Keypoint, Skeleton


class Detection:
    def __init__(self, video_source, config):
        self.config = config
        # debugging only
        if self.config['debugging']:
            if not os.path.exists(self.config['debugging_output'] + '/images'):
                os.makedirs(self.config['debugging_output'] + '/images')
            self.debugging_index = 0
            self.file = open(self.config['debugging_output'] + '/log', 'w')
            self.file.write('[\n')

        # ROS
        rospy.on_shutdown(self.shutdown)
        self.publisher = rospy.Publisher('person_detection', Skeleton, queue_size = 10)

        # OpenPose
        self.stream = VideoStream(video_source)
        self.openpose_runner = OpenPoseRunner(self.config, self.stream, self.image_analyzed_callback)
        self.openpose_runner.start()

    def convert_keypoint_to_message(self, index, keypoint):
        return Keypoint(name = self.config['body_mapping'][index], index = index, x = keypoint[0], y = keypoint[1],
                        accuracy = keypoint[2])

    def convert_keypoints_to_message(self, keypoints):
        converted_keypoints = []
        if not is_empty_or_none(keypoints) and not is_empty_or_none(keypoints[0]):
            for index, keypoint in enumerate(keypoints[0]):
                if keypoint[0] != 0 or keypoint[1] != 0 or keypoint[2] != 0:
                    converted_keypoints.append(self.convert_keypoint_to_message(index, keypoint))
        return Skeleton(keypoints = converted_keypoints)

    def convert_keypoints_to_json(self, keypoints):
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
        self.publisher.publish(skeleton)

    def image_analyzed_callback(self, keypoints, image):
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
        # debugging only
        if self.config['debugging']:
            self.file.write(']')
            self.file.close()
        self.openpose_runner.stop()
        rospy.signal_shutdown('Shutdown initialized by user')
