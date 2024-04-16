import cv2 as cv
import time
from cv2 import aruco
import numpy as np

# dictionary to specify type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# detect the marker
param_markers = aruco.DetectorParameters()

# utilizes default camera/webcam driver
cap = cv.VideoCapture(0)
frame_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
FOV = 120
print(frame_width)
x_center = int(frame_width) / 2
past_displacement = -10000
degree_tolerance = 2

# iterate through multiple frames, in a live video feed
while True:
    ret, frame = cap.read()
    if not ret:
        break
    # turning the frame to grayscale-only (for efficiency)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    # getting conrners of markers
    
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv.polylines(
                gray_frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            cv.putText(
                gray_frame,
                f"id: {ids[0]}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )
            # print(ids, "  ", corners)

            marker_x_center = (top_left[0] +top_right[0] + bottom_left[0] + bottom_right[0]) / 4
            displacement = x_center - marker_x_center
            angular_displacement = displacement * (FOV/frame_width)
            # if abs(angular_displacement - past_displacement) > degree_tolerance:
            #     print(angular_displacement)
            #     past_displacement = angular_displacement
            print(angular_displacement)
            time.sleep(0.5)

    cv.imshow("frame", gray_frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()