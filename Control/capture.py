import numpy as np
import cv2
import time
import json

cap = cv2.VideoCapture(0)

# VIDEO PARAMETERS
fps = 20.0
capSize = (1920,1080)  # this is the size of my source video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, fps, capSize)

# CLOCK SET UP
def get_time(start_time):
    current_time = time.time() - start_time
    return current_time

# FONT PARAMETERS

font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 100)
fontScale = 3
fontColor = (255, 0, 0)  # red
lineType = 2

# START THE CLOCK
start_manip = time.time()

# DICTIONARY KEY = TIME_INSTANT : VALUE FRAME
dic_frames = {}

# SCREENSHOT COUNTER
pict_nb = 1



while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        # update the dictionary
        #dic_frames[get_time(start_manip)] = frame.tolist()

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('a'):
            picture_name = "screenshot" + str(pict_nb) + ".png"
            cv2.putText(frame, str(get_time(start_manip))[:5], bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
            cv2.imwrite(picture_name, frame)
            pict_nb += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            picture_name = "screenshot" + str(pict_nb) + ".png"
            cv2.imwrite(picture_name, frame)
            pict_nb += 1
            break
    else:
        break


# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()