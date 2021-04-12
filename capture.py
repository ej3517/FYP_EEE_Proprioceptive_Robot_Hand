import numpy as np
import cv2

cap = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fps = 20.0
capSize = (1920,1080)  # this is the size of my source video
#fourcc = cv2.cv.CV_FOURCC('m', 'p', '4', 'v')  # note the lower case
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, fps, capSize)

while cap.isOpened():
    ret, frame = cap.read()
    if ret:

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()