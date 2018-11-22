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

# openpose configuration
params = dict()
params["logging_level"] = 3
params["output_resolution"] = "-1x-1"
params["net_resolution"] = "-1x368"
params["model_pose"] = "BODY_25"  # BODY_25 (fastest), COCO or MPI
params["alpha_pose"] = 0.6
params["scale_gap"] = 0.3
params["scale_number"] = 1
params["render_threshold"] = 0.05
params["num_gpu_start"] = 0
params["disable_blending"] = False
params["default_model_folder"] = OPENPOSE_PATH + "/models/"


# ---------------------- Classes ----------------------
class VideoStream:
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


# ---------------------- Program ----------------------
openpose = OpenPose(params)
stream = VideoStream(VIDEO_SOURCE).start()
while not stream.is_stopped():
    frame = stream.read()
    keypoints, output_image = openpose.forward(frame, True)
    cv2.imshow("Output", output_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
stream.stop()
cv2.destroyWindow("Output")
