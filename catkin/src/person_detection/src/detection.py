#!/usr/bin/env python

import cv2
import rospy
import os
from videostream import VideoStream
from openpose_runner import OpenPoseRunner
from util import is_empty_or_none
from person_detection.msg import Keypoint, Skeleton

VIDEO_SOURCE = 0  # integer for webcam id, string for video file path
DEBUGGING = False  # true = save images and keypoints to defined output folder and show keypoints in console, otherwise false
DEBUGGING_OUTPUT_FOLDER = '/home/informatik/openpose/output'  # save images and keypoints at this folder

# openpose configuration
OPENPOSE_PARAMS = dict()
OPENPOSE_PARAMS['installation_path'] = '/media/informatik/Linux-Daten/openpose'  # path to root directory of openpose
OPENPOSE_PARAMS['print_keypoints'] = DEBUGGING
OPENPOSE_PARAMS['show_skeleton'] = True
OPENPOSE_PARAMS['logging_level'] = 3
OPENPOSE_PARAMS['output_resolution'] = '-1x-1'
OPENPOSE_PARAMS['net_resolution'] = '-1x368'
OPENPOSE_PARAMS['model_pose'] = 'BODY_25'  # BODY_25 (fastest), COCO or MPI
OPENPOSE_PARAMS['body_mapping'] = ['Nose', 'Neck', 'RightShoulder', 'RightElbow', 'RightWrist', 'LeftShoulder',
                                   'LeftElbow', 'LeftWrist', 'MidHip', 'RightHip', 'RightKnee', 'RightAnkle',
                                   'LeftHip', 'LeftKnee', 'LeftAnkle', 'RightEye', 'LeftEye', 'RightEar',
                                   'LeftEar', 'LeftBigToe', 'LeftSmallToe', 'LeftHeel', 'RightBigToe',
                                   'RightSmallToe', 'RightHeel', 'Background']  # for BODY_25
OPENPOSE_PARAMS['alpha_pose'] = 0.6
OPENPOSE_PARAMS['scale_gap'] = 0.3
OPENPOSE_PARAMS['scale_number'] = 1
OPENPOSE_PARAMS['render_threshold'] = 0.05
OPENPOSE_PARAMS['num_gpu_start'] = 0
OPENPOSE_PARAMS['disable_blending'] = False
OPENPOSE_PARAMS['default_model_folder'] = OPENPOSE_PARAMS['installation_path'] + '/models/'


class Detection:
    def __init__(self, video_source, openpose_params):
        # debugging only
        if DEBUGGING:
            if not os.path.exists(DEBUGGING_OUTPUT_FOLDER + '/images'):
                os.makedirs(DEBUGGING_OUTPUT_FOLDER + '/images')
            self.debugging_index = 0
            self.file = open(DEBUGGING_OUTPUT_FOLDER + '/log', 'w')
            self.file.write('[\n')

        # ROS
        rospy.init_node('person_detection')
        rospy.on_shutdown(self.shutdown)
        self.publisher = rospy.Publisher('person_detection', Skeleton, queue_size = 10)
        self.config = openpose_params

        # OpenPose
        self.stream = VideoStream(video_source)
        self.openpose_runner = OpenPoseRunner(openpose_params, self.stream, self.image_analyzed_callback)
        self.openpose_runner.start()

    def convert_keypoint_to_message(self, index, keypoint):
        return Keypoint(name = self.config['body_mapping'][index], x = keypoint[0], y = keypoint[1],
                        accuracy = keypoint[2])

    def convert_keypoints_to_message(self, keypoints):
        converted_keypoints = []
        if not is_empty_or_none(keypoints) and not is_empty_or_none(keypoints[0]):
            for index, keypoint in enumerate(keypoints[0]):
                if keypoint[0] != 0 or keypoint[1] != 0 or keypoint[2] != 0:
                    converted_keypoints.append(self.convert_keypoint_to_message(index, keypoint))
                    # print('%s = X: %f, Y: %f with %i%% confidence' % (self.config['body_mapping'][index], keypoint[0], keypoint[1], keypoint[2] * 100))
        return Skeleton(keypoints = converted_keypoints)

    def convert_keypoints_to_json(self, keypoints):
        points = ''
        if not is_empty_or_none(keypoints) and not is_empty_or_none(keypoints[0]):
            for index, keypoint in enumerate(keypoints[0]):
                if keypoint[0] != 0 or keypoint[1] != 0 or keypoint[2] != 0:
                    if points is not '':
                        points += ',\n'
                    points += '      { "part": ' + str(index) + ', "description": "' + OPENPOSE_PARAMS['body_mapping'][
                        index] + '", "x": ' + str(keypoint[0]) + ', "y": ' + str(
                        keypoint[1]) + ', "accuracy": ' + str(keypoint[2]) + '}'
        return '  { "index": ' + (
            str(self.debugging_index) if DEBUGGING else '"unknown"') + ' ,\n    "points": [\n' + points + '\n    ]\n  }'

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
        if DEBUGGING:
            if not is_empty_or_none(keypoints) and not is_empty_or_none(keypoints[0]):
                cv2.imwrite(DEBUGGING_OUTPUT_FOLDER + '/images/' + str(self.debugging_index) + '.jpg', image)
                keypoints_json = self.convert_keypoints_to_json(keypoints)
                self.file.write(keypoints_json if self.debugging_index is 0 else ',\n' + keypoints_json)
                self.debugging_index += 1

        self.publish(self.convert_keypoints_to_message(keypoints))
        if DEBUGGING or self.config['show_skeleton']:
            self.show_skeleton(image)

    def shutdown(self):
        # debugging only
        if DEBUGGING:
            self.file.write('\n]')
            self.file.close()
        self.openpose_runner.stop()


if __name__ == '__main__':
    person_detection = Detection(VIDEO_SOURCE, OPENPOSE_PARAMS)
