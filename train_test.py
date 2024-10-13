# Load the data, create a split, and return the particular sample. 
# TO DO: 
# Build / Load model
# Train and test
# Save in whichever format#

import csv
import numpy as np
import random
import cv2
from statistics import mode

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


# Declaring the frame width and height
FRAME_WIDTH=640
FRAME_HEIGHT=480


# Medipipe hand landmark detector options
base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
options = vision.HandLandmarkerOptions(base_options=base_options,
                                       num_hands=1,
                                       min_hand_detection_confidence=0.4)
detector = vision.HandLandmarker.create_from_options(options)

from feature_extraction import FeatureExtractor

random.seed(2)

class Dataset:
    def __init__(self, filepath=None):
        self.filepath = filepath

    def load_data(self):
        with open(self.filepath, mode='r', newline='') as csvfile:
            data = csv.reader(csvfile, delimiter=',')
            data_matrix = []
            for row in data:
                #convert to features here! 
                data_vec = [float(x) for x in row[:-1]]
                label = int(row[-1])
                train_feats = FeatureExtractor(data_vec,label)
                data_matrix.append(train_feats)
        data_matrix = np.array(data_matrix)
        random.shuffle(data_matrix)
        random.shuffle(data_matrix)

        self.labels = data_matrix[:, -1].reshape(-1)
        self.data_matrix = np.array(data_matrix[:,:-1])
    
    def train_test_split(self,train_len = 0.8):
        self.train_data = self.data_matrix[:int(train_len*len(self.labels)),:]
        self.train_labels = self.labels[:int(train_len*len(self.labels))]
        self.test_data = self.data_matrix[int(train_len*len(self.labels)):, :]
        self.test_labels = self.labels[int(train_len*len(self.labels)):]

    def get_train_data(self):
        return self.train_data, self.train_labels
    
    def get_test_data(self):
        return self.test_data, self.test_labels
    

class KNN:
    def __init__(self, k=0):
        self.k = k
        self.neighbours=None

    def load_data(self,data,label):
        self.data = data
        self.label = label

    def predict(self,x):
        self.feature_vec = x
        self.find_nearest_neighbours()

    def find_nearest_neighbours(self):
        dist_list = []
        for point in self.data:
            dist = 0
            for i in range(len(point)):
                dist+= (point[i] - self.feature_vec[i])**2
            dist_list.append( dist**0.5 )
        
        distance = list(zip(dist_list, self.label))
        distance.sort(key= lambda x : x[0])
        self.neighbours = [distance[k][1] for k in range(self.k)]
    
    def get_label(self):
        ## Get the label of the most occuring point.
        pass



if __name__=="__main__":
    FRAME_WIDTH=640
    FRAME_HEIGHT=480
    path = "vbh_dataset.csv"
    dataset = Dataset(path)
    dataset.load_data()
    dataset.train_test_split(0.95)
    train_data, train_labels = dataset.get_train_data()
    test_data, test_labels = dataset.get_test_data()
    knn = KNN(k = 3)
    knn.load_data(train_data, train_labels)
    webcam = cv2.VideoCapture(0)
    webcam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    num_samp = 0
    points_of_interest = [1,5,9,13,17, 4,8,12,16,20]
    # create / open vbh_dataset.csv

    while(True):
        ret, frame = webcam.read()
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        hand_landmarker_result = detector.detect(mp_image)
        hand_landmarks = hand_landmarker_result.hand_landmarks
        points = []
        lab = ""
        if (len(hand_landmarks)!=0):
            points = []
            for point in points_of_interest:
                x = hand_landmarks[0][point].x - hand_landmarks[0][0].x
                y = hand_landmarks[0][point].y - hand_landmarks[0][0].y
                points.append(x)
                points.append(y)
            feats = FeatureExtractor(points)
            knn.predict(x = feats)
            print(mode(knn.neighbours))
        cv2.imshow('WebCam', frame)
        cv2.waitKey(50)
        if cv2.waitKey(1) & 0xFF==ord('q'):
            break

    webcam.release()
    cv2.destroyAllWindows()

    knn.load_data(train_data, train_labels)
    knn.predict(test_data[5])
    print(knn.neighbours)