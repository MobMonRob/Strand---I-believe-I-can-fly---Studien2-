# ---------------------- Imports ----------------------
import sys
import cv2

sys.path.append('/usr/local/python/openpose')
from openpose import *
from threading import Thread

# ---------------------- Configuration ----------------------
# path to root directory of openpose
OPENPOSE_PATH = '/media/informatik/Linux-Daten/openpose'

# integer for webcam id, string for video file path
VIDEO_SOURCE = 0

# set this to True to enable debugging output in console
DEBUGGING = True

# openpose configuration
OPENPOSE_PARAMS = dict()
OPENPOSE_PARAMS['logging_level'] = 3
OPENPOSE_PARAMS['output_resolution'] = '-1x-1'
OPENPOSE_PARAMS['net_resolution'] = '-1x368'
OPENPOSE_PARAMS['model_pose'] = 'BODY_25'  # BODY_25 (fastest), COCO or MPI
OPENPOSE_PARAMS['body_mapping'] = ['Nose', 'Neck', 'Right Shoulder', 'Right Elbow', 'Right Wrist', 'Left Shoulder',
                                   'Left Elbow', 'Left Wrist', 'Mid Hip', 'Right Hip', 'Right Knee', 'Right Ankle',
                                   'Left Hip', 'Left Knee', 'Left Ankle', 'Right Eye', 'Left Eye', 'Right Ear',
                                   'Left Ear', 'Left Big Toe', 'Left Small Toe', 'Left Heel', 'Right Big Toe',
                                   'Right Small Toe', 'Right Heel', 'Background']
OPENPOSE_PARAMS['alpha_pose'] = 0.6
OPENPOSE_PARAMS['scale_gap'] = 0.3
OPENPOSE_PARAMS['scale_number'] = 1
OPENPOSE_PARAMS['render_threshold'] = 0.05
OPENPOSE_PARAMS['num_gpu_start'] = 0
OPENPOSE_PARAMS['disable_blending'] = False
OPENPOSE_PARAMS['default_model_folder'] = OPENPOSE_PATH + '/models/'


# ---------------------- Classes ----------------------
class VideoStream:
    """
    Provides multi-threaded video capturing to improve performance as video capturing of a camera is a blocking process.
    """

    def __init__(self, src = 0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target = self.update, args = ()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

    def is_stopped(self):
        return self.stopped


# ---------------------- Methods ----------------------
def is_empty_or_none(array):
    """
    Checks if array is empty.
    :param array: array to check
    :return: True if array is None or empty, otherwise False.
    """
    return object is None or len(array) == 0


def print_detected_parts(keypoints, show_missing = False):
    """
    Prints given keypoints to console using body part mapping provided in OPENPOSE_PARAMS['body_mapping'].
    :param keypoints: keypoints to print
    :param show_missing: True if not recognized points should also be printed, False if these should be hidden (default)
    """
    print('######################################################')
    if is_empty_or_none(keypoints) or is_empty_or_none(keypoints[0]):
        print('No person detected!')
    else:
        for index, keypoint in enumerate(keypoints[0]):
            if keypoint[0] != 0 or keypoint[1] != 0 or keypoint[2] != 0 or (
                    keypoint[0] == 0 and keypoint[1] == 0 and keypoint[2] == 0 and show_missing):
                print('%s = X: %f, Y: %f with %i%% confidence' % (
                    OPENPOSE_PARAMS['body_mapping'][index], keypoint[0], keypoint[1], keypoint[2] * 100))


# ---------------------- Program ----------------------
openpose = OpenPose(OPENPOSE_PARAMS)
stream = VideoStream(VIDEO_SOURCE).start()
while not stream.is_stopped():
    frame = stream.read()
    keypoints, output_image = openpose.forward(frame, True)
    if DEBUGGING:
        print_detected_parts(keypoints)
    cv2.imshow('Output', output_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
stream.stop()
cv2.destroyWindow('Output')
