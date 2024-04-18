import rclpy
from rclpy.node import Node
from cysar.msg import ArucoData
import cv2 as cv
import time
from cv2 import aruco
import numpy as np

"""
ArucoAngularDisplacement.py

Desc: Publishes Aruco angular displacement data to ROS
Author: Ethan Cabelin
Date: 4/17/24
"""

class ArucoAngularDisplacement(Node):
 
    def __init__(self) -> None:
        super().__init__('Operator_Interface')
        self.aruco = ArucoData()
        self.aruco_angle_publisher = self.create_publisher(ArucoData, 'arucodata', 10)
        self.timer = self.create_timer(0.05, self.talker)

        # dictionary to specify type of the marker
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # detect the marker
        self.param_markers = aruco.DetectorParameters()
        # utilizes default camera/webcam driver
        '''
        Change later for different sources
        '''
        self.cap = cv.VideoCapture(0)
        self.frame_width = self.cap.get(cv.CAP_PROP_FRAME_WIDTH)
        self.FOV = 120
        self.x_center = int(self.frame_width) / 2
        self.past_displacement = -10000
        self.degree_tolerance = 2

    def talker(self) -> None:
        """
        Loop for retrieving angular displacement and publishing to ROS
        """
        # iterate through multiple frames, in a live video feed
        while True: # Test that this does not generate a double loop
            ret, frame = self.cap.read()
            if not ret:
                break
            # turning the frame to grayscale-only (for efficiency)
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = aruco.detectMarkers(
                gray_frame, self.marker_dict, parameters=self.param_markers
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
                    displacement = self.x_center - marker_x_center
                    angular_displacement = displacement * (self.FOV/self.frame_width)
                    # if abs(angular_displacement - past_displacement) > degree_tolerance:
                    #     print(angular_displacement)
                    #     past_displacement = angular_displacement
                    self.aruco.disp_angle = angular_displacement
                    self.aruco_angle_publisher.publish(self.aruco)

                    #print(angular_displacement) # Integrate into ROS
                    time.sleep(0.5)

            '''
            Edit this section for later debugging. Likely not needed.
            '''
            cv.imshow("frame", gray_frame)
            key = cv.waitKey(1)
            if key == ord("q"):
                break

        self.cap.release()
        cv.destroyAllWindows()
        #!/usr/bin/python3  


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    ArucoAD = ArucoAngularDisplacement()

    # Run the node
    rclpy.spin(ArucoAD)

    # Destroy it when done
    ArucoAD.disconnect()
    ArucoAD.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()