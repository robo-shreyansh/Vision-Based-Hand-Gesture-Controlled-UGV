from models import Dataset, KNN
import numpy as np
import matplotlib.pyplot as plt


if __name__=="__main__":

    path = "vbh_dataset.csv"
    dataset = Dataset(path)
    dataset.load_data()
    num_folds = 5
    fold_len = 0.75
    folds, folds_labels, test, test_labels = dataset.get_cross_validation_folds(num_folds=num_folds, fold_len=fold_len)

    epoch_accuracy = []
    epoch_mean_accuracy=[]
    k_vals = []

    for k in range(2,16):

        knn = KNN(k = k)
        knn.set_distance("euclidean")
        accuracy = []
        
        for i in range(num_folds):

            val = folds[i]
            val_labels = folds_labels[i]

            train_data = np.vstack([folds[j] for j in range(num_folds) if j != i])
            train_labels = np.hstack( [folds_labels[j] for j in range(num_folds) if j != i] )
            knn.load_data(train_data, train_labels)
            output=[]

            for val_point in val:
                knn.predict(val_point)
                output.append( knn.get_label() )
            output = np.array(output)
            accuracy.append( np.sum((output==val_labels))/len(output) )
        
        print("Checking for k = ",k)

        mean_accuracy = sum(accuracy)/len(accuracy)
        k_vals.append(k)
        epoch_accuracy.append(accuracy)
        epoch_mean_accuracy.append(mean_accuracy)
        
    plt.figure(figsize=(8, 6))
    plt.plot(k_vals, epoch_mean_accuracy, marker='o', linestyle='-', color='b', label='Euclidean Distance')

    plt.xlabel('Number of Neighbors (k)')
    plt.ylabel('Mean Accuracy')
    plt.title('KNN Performance vs. Number of Neighbors (k)')
    plt.legend()

    plt.grid(True)
    plt.show()



#import mediapipe as mp
#from mediapipe.tasks import python
#from mediapipe.tasks.python import vision
# 
# Medipipe hand landmark detector options
# base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
# options = vision.HandLandmarkerOptions(base_options=base_options,
#                                        num_hands=1,
#                                        min_hand_detection_confidence=0.4)
# detector = vision.HandLandmarker.create_from_options(options)
    # # Declaring the frame width and height
# FRAME_WIDTH=640
# FRAME_HEIGHT=480
    # train_data, train_labels = dataset.get_train_data()
    # test_data, test_labels = dataset.get_test_data()
    # knn = KNN(k = 3)
    # knn.load_data(train_data, train_labels)
    # FRAME_WIDTH=640
    # FRAME_HEIGHT=480
    # webcam = cv2.VideoCapture(0)
    # webcam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    # webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    # num_samp = 0
    # points_of_interest = [1,5,9,13,17, 4,8,12,16,20]
    # # create / open vbh_dataset.csv
    # while(True):
    #     ret, frame = webcam.read(_of_interest:
    #             x = hand_landmarks[0][point].x - hand_landmarks[0][0].x
    #             y = hand_landmarks[0][point].y - hand_landmarks[0][0].y
    #             points.append(x)
    #             points.append(y)
    #         feats = FeatureExtractor(points)
    #         knn.predict(x = feats)
    #         print(mode(knn.neighbours))
    #     cv2.imshow('WebCam', frame)
    #     cv2.waitKey(50)
    #     if cv2.waitKey(1) & 0xFF==ord('q'):
    #         break
    # webcam.release()
    # cv2.destroyAllWindows()
    # knn.load_data(train_data, train_labels)
    # knn.predict(test_data[5])
    # print(knn.neighbours))
    #     mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
    #     hand_landmarker_result = detector.detect(mp_image)
    #     hand_landmarks = hand_landmarker_result.hand_landmarks
    #     points = []
    #     lab = ""
    #     if (len(hand_landmarks)!=0):
    #         points = []
    #         for point in points_