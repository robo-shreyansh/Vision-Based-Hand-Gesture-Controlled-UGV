import numpy as np
import csv
import math
import random

points_of_interest = [1,5,9,13,17, 4,8,12,16,20]
bases = [1,5,9,13,17]
tips = [4,8,12,16,20]

class vectors:
    def __init__(self, points):
        self.x = points[0]
        self.y = points[1]
        try :
            self.angle = math.atan(self.y/self.x)
        except ZeroDivisionError:
            if self.y>0:
                self.angle = np.pi/2
            else:
                self.angle = -np.pi/2
    
    def __sub__(self, vec2):
        return vectors([self.x - vec2.x , self.y - vec2.y])
    
    def distance(self, vec2 ):
        diff = self.__sub__(vec2)
        return (diff.x**2 + diff.y**2)**0.5
    def __str__(self):
        return "x: {} , y : {}".format(self.x, self.y)

class Fingers:
    def __init__(self, points):
        self.points = points
        self.fingers_points = []
        self.finger_tips = []
        self.finger_base = []
        for i in range(10):
            self.fingers_points.append(vectors(points[2*i:2*i+2]))
            if i<5:
                self.finger_base.append(vectors(points[2*i:2*i+2]))
            else:
                self.finger_tips.append(vectors(points[2*i:2*i+2]))
    
    def get_finger_points(self):
        return self.fingers_points

    def tip_wrist_angle(self):
        tip_wrist_angles = []
        for ind in range(4):
            th1 = self.finger_tips[ind+1].angle
            th2 = self.finger_tips[ind].angle
            tip_wrist_angles.append( (th2 - th1)/(1+th1*th2) )
        return tip_wrist_angles
    
    def tip_wrist_distance(self):
        tip_wrist_distances = []
        for ind in range(5):
            tip_wrist_distances.append(self.finger_tips[ind].distance(vectors([0,0])))
        return tip_wrist_distances
    
    def tip_base_distance(self):
        tip_base_distances = []
        for ind in range(5):
            tip_base_distances.append(self.finger_tips[ind].distance(self.finger_base[ind]))
        return tip_base_distances



def FeatureExtractor(points, label=None):
    fings = Fingers(points)
    if label!=None:
        feat_row = fings.tip_wrist_angle() + fings.tip_wrist_distance() + fings.tip_base_distance() + [label]
    else:
        feat_row = fings.tip_wrist_angle() + fings.tip_wrist_distance() + fings.tip_base_distance()
    return feat_row


if __name__=="__main__":

    feature_matrix = []
    with open("vbh_dataset.csv", newline = '') as file:
        data = csv.reader(file, delimiter=',')
        for row in data:
            label = int(row[-1])
            points = [float(x) for x in row[:-1]]
            feat_row = FeatureExtractor(points, label) 
            feature_matrix.append(feat_row)
    random.shuffle(feature_matrix)
    with open("data_features.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        for row in feature_matrix:
            writer.writerow(row)
