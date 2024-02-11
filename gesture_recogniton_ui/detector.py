import cv2
import numpy

FRAME_WIDTH=640
FRAME_HEIGHT=480
windowshapes = [[64,64], [128,128], [224,224]] # Sliding windows, sizes: heights and widths


# Initialize the webcam
webcam = cv2.VideoCapture(0)
webcam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


while(True):
    ret, frame = webcam.read()

    # Sliding windows
    for windowshape in windowshapes:
        for i in range(0,FRAME_HEIGHT-windowshape[0]//2, windowshape[0]//2):
            for j in range(0,FRAME_WIDTH-windowshape[1]//2, windowshape[1]//2):
                window = frame[i:i+windowshape[0], j:j+windowshape[1]]

                # Write recognition code here

                cv2.imshow("windows", window)
                cv2.waitKey(10)

    cv2.imshow('WebCam', frame)
    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF==ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()