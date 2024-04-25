import rclpy
from rclpy.node import Node
from cysar.msg import ArucoData
import cv2 
import time
from cv2 import aruco
import numpy as np

"""
ArucoAngularDisplacement.py

Desc: Publishes Aruco angular displacement data to ROS
Author: Ethan Cabelin
Date: 4/17/24
"""

class ArucoTransposedDistance(Node):
 
    def __init__(self) -> None:
        super().__init__('ArucoTransposedDistance')
        self.aruco = ArucoData()
        self.aruco_dist_publisher = self.create_publisher(ArucoData, 'arucodata', 10)
        self.timer = self.create_timer(0.05, self.talker)

        # Load the camera calibration matrix, focal lengths and principal points gained from calibration
        self.focal_length_x = 0.42371
        self.focal_length_y =0.42252
        self.principal_point_x = 0.32383
        self.principal_point_y = 0.23551
        self.camera_matrix = np.array([[self.focal_length_x, 0, self.principal_point_x],
                                [0, self.focal_length_y, self.principal_point_y],
                                [0, 0, 1]])

        self.dist_coeffs = np.zeros((4,1))  # adjust for distortion from camera calibration

        # Define the ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Define parameters for marker detection
        self.parameters = cv2.aruco.DetectorParameters()

        # Initialize video capture
        self.cap = cv2.VideoCapture(1)  # Camera index

    def talker(self) -> None:
        """
        Loop for retrieving transposed distance and publishing to ROS
        """
        while True: # Test that this does not generate a double loop
            ret, frame = self.cap.read()
            if not ret:
                break

            # Detect ArUco markers
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                for i, corner in enumerate(corners):
                    # Estimate pose of the detected marker
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, self.camera_matrix, self.dist_coeffs)

                    # Draw axis and distance on the frame
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                    distance = np.linalg.norm(tvecs[i])
                    cv2.putText(frame, f"Distance: {distance:.2f} meters", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    # Calculate the position of a point at a set distance from the marker
                    set_distance = 0.01  # Set distance in meters
                    direction_vector = tvecs[i] / distance  # Normalize the vector
                    point_position = tvecs[i] + set_distance * direction_vector  # Calculate the point's position

                    # Draw the point on the frame (if you want)
                    point_pixel_position, _ = cv2.projectPoints(point_position, rvecs[i], tvecs[i], self.camera_matrix, self.dist_coeffs)
                    point_pixel_position = tuple(point_pixel_position[0].astype(int))

                    if point_pixel_position is not None:
                        point_pixel_position = tuple(map(int, point_pixel_position[0]))  # Convert to int and extract first element
                        cv2.circle(frame, point_pixel_position, 5, (0, 0, 255), -1)

                    # Distance Publisher
                    self.aruco.point_pos = point_pixel_position
                    self.aruco_dist_publisher.publish(self.aruco)

            '''
            Edit/Remove Later. Likely not needed
            '''
            # Display the frame
            cv2.imshow('Frame', frame)
            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the camera and close all OpenCV windows
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    ArucoTD = ArucoTransposedDistance()

    # Run the node
    rclpy.spin(ArucoTD)

    # Destroy it when done
    ArucoTD.disconnect()
    ArucoTD.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
