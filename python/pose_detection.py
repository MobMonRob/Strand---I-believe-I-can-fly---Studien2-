import json
import math

class Point:
    def __init__(self, x, y, acc, index, desc):
        self.x = x
        self.y = y
        self.acc = acc
        self.index = index
        self.desc = desc

    def __str__(self):
        return "[x: " + str(self.x) + ", y: " + str(self.y) + ", acc: " + str(int(self.acc * 100)) + "%" + ", index: " + str(self.index) + ", desc: " + str(self.desc) + "]"

    def __repr__(self):
        return self.__str__()

class Skeleton:
    def __init__(self, frame, keypoints, required_accuracy):
        self.frame = frame
        self.keypoints = keypoints
        self.required_accuracy = required_accuracy
        self.handGradient = None
        self.handDistance = None

    def checkForImportantKeypoints(self):
        if 8 not in self.keypoints:
            print("Hip not detected at frame " + str(self.frame) + "!")
            return False
        elif 8 in self.keypoints and self.keypoints[8].acc < self.required_accuracy:
            print("Detected hip with low accuracy of ~" + str(int(self.keypoints[8].acc * 100)) + "% at frame " + str(self.frame) + "!")
            return False
        elif 2 not in self.keypoints:
            print("Right shoulder not detected at frame " + str(self.frame) + "!")
            return False
        elif 2 in self.keypoints and self.keypoints[2].acc < self.required_accuracy:
            print("Detected right shoulder with low accuracy of ~" + str(int(self.keypoints[2].acc * 100)) + "% at frame " + str(self.frame) + "!")
            return False
        elif 5 not in self.keypoints:
            print("Left shoulder not detected at frame " + str(self.frame) + "!")
            return False
        elif 5 in self.keypoints and self.keypoints[5].acc < self.required_accuracy:
            print("Detected left shoulder with low accuracy of ~" + str(int(self.keypoints[5].acc * 100)) + "% at frame " + str(self.frame) + "!")
            return False
        elif 4 not in self.keypoints:
            print("Right hand not detected at frame " + str(self.frame) + "!")
            return False
        elif 4 in self.keypoints and self.keypoints[4].acc < self.required_accuracy:
            print("Detected right hand with low accuracy of ~" + str(int(self.keypoints[4].acc * 100)) + "% at frame " + str(self.frame) + "!")
            return False
        elif 7 not in self.keypoints:
            print("Left hand not detected at frame " + str(self.frame) + "!")
            return False
        elif 7 in self.keypoints and self.keypoints[7].acc < self.required_accuracy:
            print("Detected left hand with low accuracy of ~" + str(int(self.keypoints[7].acc * 100)) + "% at frame " + str(self.frame) + "!")
            return False
        return True

    def transformPoints(self):
        for key in self.keypoints:
            if self.keypoints[key].index is not 8:
                self.keypoints[key].x = self.keypoints[key].x - self.keypoints[8].x
                self.keypoints[key].y = self.keypoints[8].y - self.keypoints[key].y
        self.keypoints[8].x = 0
        self.keypoints[8].y = 0

    def calcHandGradient(self):
        if self.keypoints[4] is not None and self.keypoints[7] is not None:
            self.handGradient = (self.keypoints[7].y - self.keypoints[4].y) / (self.keypoints[7].x - self.keypoints[4].x)
        else:
            self.handGradient = None

    def getHandGradient(self):
        if self.handGradient is None:
            self.calcHandGradient()
        return self.handGradient
    
    def calcHandDistance(self):
        if self.keypoints[4] is not None and self.keypoints[7] is not None:
            self.handDistance = math.sqrt(((self.keypoints[7].x - self.keypoints[4].x) ** 2) + ((self.keypoints[7].y - self.keypoints[4].y) ** 2))
        else:
            self.handDistance = None

    def getHandDistance(self):
        if self.handDistance is None:
            self.calcHandDistance()
        return self.handDistance
        

file = open("../data/OpenPose Demo#1/log_minified", "r")
content = file.read()
parsed_data = json.loads(content)
total_scanned = 0
missing_keypoints = 0
for frame_set in parsed_data:
    points = {}
    for point in frame_set["points"]:
        points[point["part"]] = Point(point["x"], point["y"], point["accuracy"], point["part"], point["description"])
    skeleton = Skeleton(frame_set["index"], points, 0.4)
    total_scanned += 1
    if skeleton.checkForImportantKeypoints():
        skeleton.transformPoints()
        if skeleton.frame % 50 == 0:
            pass
    else:
        missing_keypoints += 1
        
print("Detection rate: " + str(100 - round(100 * (missing_keypoints * 1.0 / total_scanned), 2)) + "% (" + str(total_scanned - missing_keypoints) + "/" + str(total_scanned) + ")")
