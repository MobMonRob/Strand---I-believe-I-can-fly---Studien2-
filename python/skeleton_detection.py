import cv2
import numpy as np

# TODO: Speed up calculation, maybe use GPU?
# TODO: Don't buffer webcam images, instead use current image every time calculation finishes

# Configuration
threshold = 0.1

# COCO:
protoFile = "/media/informatik/Linux-Daten/learnopencv-master/OpenPose/pose/coco/pose_deploy_linevec.prototxt"
weightsFile = "/media/informatik/Linux-Daten/learnopencv-master/OpenPose/pose/coco/pose_iter_440000.caffemodel"
nPoints = 18
POSE_PAIRS = [[1, 0], [1, 2], [1, 5], [2, 3], [3, 4], [5, 6], [6, 7], [1, 8], [8, 9], [9, 10], [1, 11], [11, 12],
              [12, 13], [0, 14], [0, 15], [14, 16], [15, 17]]


# MPI:
# protoFile = "/media/informatik/Linux-Daten/learnopencv-master/OpenPose/pose/mpi/pose_deploy_linevec.prototxt"
# protoFile = "/media/informatik/Linux-Daten/learnopencv-master/OpenPose/pose/mpi/pose_deploy_linevec_faster_4_stages.prototxt"
# weightsFile = "/media/informatik/Linux-Daten/learnopencv-master/OpenPose/pose/mpi/pose_iter_160000.caffemodel"
# nPoints = 15
# POSE_PAIRS = [[0, 1], [1, 2], [2, 3], [3, 4], [1, 5], [5, 6], [6, 7], [1, 14], [14, 8], [8, 9], [9, 10], [14, 11], [11, 12], [12, 13]]


def processFrameInNet(frame, net):
    blob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (368, 368), (0, 0, 0), swapRB=False, crop=False)
    net.setInput(blob)
    return net.forward()


# Methods
def processNetOutput(output, frame, nPoints, threshold):
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]
    outHeight = output.shape[2]
    outWidth = output.shape[3]
    points = []
    for i in range(nPoints):
        probMap = output[0, i, :, :]
        minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
        x = (frameWidth * point[0]) / outWidth
        y = (frameHeight * point[1]) / outHeight

        if prob > threshold:
            cv2.circle(frame, (int(x), int(y)), 8, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
            # cv2.putText(frame, "{}".format(i), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, lineType=cv2.LINE_AA)
            points.append((int(x), int(y)))
        else:
            points.append(None)
    return points


def drawSkeletonToFrame(frame, points, posePairs):
    for pair in posePairs:
        partA = pair[0]
        partB = pair[1]
        if points[partA] and points[partB]:
            cv2.line(frame, points[partA], points[partB], (0, 255, 255), 2)
            # cv2.circle(frame, points[partA], 8, (0, 0, 255), thickness=-1, lineType=cv2.FILLED)
    return frame


def trackWebcam():
    cap = cv2.VideoCapture(0)
    net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
    while True:
        print("Reading new image...")
        ret, frame = cap.read()
        output = processFrameInNet(frame, net)
        points = processNetOutput(output, frame, nPoints, threshold)
        frame = drawSkeletonToFrame(frame, points, POSE_PAIRS)
        print("Processing finished...")
        cv2.imshow('Output-Skeleton', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Stopping program...")
            break
    cap.release()
    cv2.destroyAllWindows()


# Program
trackWebcam()
