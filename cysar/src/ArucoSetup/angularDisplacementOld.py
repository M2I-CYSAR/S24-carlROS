import cv2
import numpy as np

# Load the camera calibration matrix, focal lengths and principal points gained from calibration
focal_length_x = 0.42371
focal_length_y =0.42252
principal_point_x = 0.32383
principal_point_y = 0.23551
camera_matrix = np.array([[focal_length_x, 0, principal_point_x],
                          [0, focal_length_y, principal_point_y],
                          [0, 0, 1]])

dist_coeffs = np.zeros((4,1))  # adjust for distortion from camera calibration

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Define parameters for marker detection
parameters = cv2.aruco.DetectorParameters()

# Initialize video capture
cap = cv2.VideoCapture(1)  # Camera index

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        for i, corner in enumerate(corners):
            # Estimate pose of the detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)

            # Draw axis and distance on the frame
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
            distance = np.linalg.norm(tvecs[i])
            cv2.putText(frame, f"Distance: {distance:.2f} meters", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Calculate the position of a point at a set distance from the marker
            set_distance = 0.01  # Set distance in meters
            direction_vector = tvecs[i] / distance  # Normalize the vector
            point_position = tvecs[i] + set_distance * direction_vector  # Calculate the point's position

            # Draw the point on the frame (if you want)
            point_pixel_position, _ = cv2.projectPoints(point_position, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            point_pixel_position = tuple(point_pixel_position[0].astype(int))

            if point_pixel_position is not None:
                point_pixel_position = tuple(map(int, point_pixel_position[0]))  # Convert to int and extract first element
                cv2.circle(frame, point_pixel_position, 5, (0, 0, 255), -1)


    # Display the frame
    cv2.imshow('Frame', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

