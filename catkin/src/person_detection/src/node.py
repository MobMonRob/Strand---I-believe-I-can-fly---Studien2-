#!/usr/bin/env python

from detection import Detection

VIDEO_SOURCE = 0  # integer for webcam id, string for video file path

# openpose configuration
OPENPOSE_PARAMS = dict()
OPENPOSE_PARAMS['debugging'] = False  # true = save images and keypoints to defined output folder and show keypoints in console, otherwise false
OPENPOSE_PARAMS['debugging_output'] = '/home/informatik/openpose/output'  # save images and keypoints at this folder
OPENPOSE_PARAMS['installation_path'] = '/media/informatik/Linux-Daten/openpose'  # path to root directory of openpose
OPENPOSE_PARAMS['print_keypoints'] = OPENPOSE_PARAMS['debugging']
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

if __name__ == '__main__':
    person_detection = Detection(VIDEO_SOURCE, OPENPOSE_PARAMS)
