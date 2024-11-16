#! /usr/bin/env python3

import cv2
import os

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from ugv_vision.vis_models.models import Dataset, KNN
from ugv_vision.vis_models.feature_extraction import FeatureExtractor

from ament_index_python.packages import get_package_share_directory


package_share_directory = get_package_share_directory('ugv_vision')
hand_lanmarker = os.path.join(package_share_directory, 'hand_landmarker.task')
dataset_path = os.path.join(package_share_directory, 'vbh_dataset.csv')

dataset = Dataset(filepath=dataset_path)
# dataset.load_data()
data, labs = dataset.get_data()
knn = KNN(k = 3)
knn.load_data(data, labs)
base_options = python.BaseOptions(hand_lanmarker)
options = vision.HandLandmarkerOptions(base_options=base_options,
                                       num_hands=1,
                                       min_hand_detection_confidence=0.4)
detector = vision.HandLandmarker.create_from_options(options)
points_of_interest = [1,5,9,13,17, 4,8,12,16,20]




CLASSES = ["","",""] # now here define the classes as used 'victory' 'fist', 'palm' in whichever order. 

class PC_Viewer(Node):
    def __init__(self):
        super().__init__("pc_viewer")
        self.cv_bridge = CvBridge()
        self.img_subscription = self.create_subscription(Image, "/img_data", self.img_sub_callback, 10)

    def img_sub_callback(self, data : Image):
        img_frame = self.cv_bridge.imgmsg_to_cv2(data)
        cv2.imshow("Img Feed", img_frame)
        cv2.waitKey(2)
        self.classify(img_frame)

    def classify(self, frame):
        # Use mediapipe to get the points using mediapipe, transform and feed to KNN
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
            knn.predict(feats)
            m = knn.get_label()
            print(m)

def main(args=None):
    rclpy.init(args= args)
    node = PC_Viewer()
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__=="__main__":
    main()