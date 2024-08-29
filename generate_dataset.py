import cv2
import random
import sys
import csv

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


# Data augmentation probabilites
flip_probability = 0.8
rotation_probability = 0.5
blur_probability = 0.3

# Data augmentation aux
theta = random.uniform(-20,20) 
rot_mat = cv2.getRotationMatrix2D( (FRAME_WIDTH/2, FRAME_HEIGHT/2) , theta, 1.0)
blur_kernel = (5,5)

# Points corresponding to writst, finger base, and finger tips
points_of_interest = [1,5,9,13,17, 4,8,12,16,20]


def generate_dataset(label, dataset_size, create_new=0):
    webcam = cv2.VideoCapture(0)
    webcam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    num_samp = 0

    # create / open vbh_dataset.csv
    if create_new==0:
            mode='a'
    else:
        mode='w'

    with open('vbh_dataset.csv', mode=mode, newline='') as file:
        writer = csv.writer(file)

        while(num_samp<dataset_size):
            ret, frame = webcam.read()
            
            if random.random()>=flip_probability:
                frame = cv2.flip(frame, random.choice([-1, 0, 1]))

            if random.random()>=rotation_probability:
                frame = cv2.warpAffine(frame, rot_mat, (FRAME_WIDTH, FRAME_HEIGHT))

            if random.random()>=blur_probability:
                frame = cv2.GaussianBlur(frame, blur_kernel, 0)

            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
            hand_landmarker_result = detector.detect(mp_image)
            hand_landmarks = hand_landmarker_result.hand_landmarks
            
            points = []

            if (len(hand_landmarks)!=0):

                for point in points_of_interest:

                    x = hand_landmarks[0][point].x - hand_landmarks[0][0].x
                    y = hand_landmarks[0][point].y - hand_landmarks[0][0].y
                    points.append(x)
                    points.append(y)

                points.append(label)
                writer.writerow(points)
                num_samp+=1

            cv2.imshow('WebCam', frame)
            cv2.waitKey(50)
            if cv2.waitKey(1) & 0xFF==ord('q'):
                break

    webcam.release()
    cv2.destroyAllWindows()



def main():
    label = sys.argv[1]
    dataset_size = int(sys.argv[2])
    try:
        create_new = int(sys.argv[3])
    except:
        create_new = 0

    classes = {
    "Palm": 0,
    "Fingers": 1,
    "Fist": 2,
    }

    generate_dataset(label = classes[label], dataset_size=dataset_size, create_new = create_new)

if __name__=="__main__":
    main()

    
