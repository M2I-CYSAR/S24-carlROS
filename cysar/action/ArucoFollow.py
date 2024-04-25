import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from cysar.msg import ArucoData
import JoystickGhost
import time

class ArucoFollow (Node):

    def __init__(self): # Constructor
        super().__init__('ArucoFollow')
        '''
        1. ROS 2 node to add the server to.
        2. Type of action
        3. Action name
        4. A callback function for executing accepted goals: self.execute_callback. This callback must return a result message for the action type.
        '''
        self._action_server = ActionServer(
            self,
            ArucoFollow,
            'arucofollow',
            self.execute_callback)
        
        self.arucodata = ArucoData()
        self.joystick = JoystickGhost()

    ''' 
    The Method called to execute an action. This method handles 3 message types.
    1. A Goal Service - A Request sent from the client, to the server. The Server can send back a response.
    2. A Feedback Topic - A middle line that the client can access to receive notes on task progress from the server.
    3. A Result Service - A final request from the client to the server. A response is sent back from the server.
    '''
    def execute_callback(self, goal_handle): # Method called - Goal_Handle Param input = time
        self.get_logger().info('Executing goal...')
        # Run an action execution and output new joystick values from terminal.
        #self.get_logger().info(str(goal_handle.request.time))
        duration = goal_handle.request.time
        time_length = float(time.time()) + float(duration)
        Aruco_AngleD = self.arucodata.disp_angle
        Aruco_PointPos = self.arucodata.point_pos
        neutral_angle = 0 # Adjust for param
        neutral_pos = 0 # Adjust for param
        # Test
        while Aruco_AngleD != neutral_angle:
            if Aruco_AngleD < neutral_angle:
                self.joystick.right(0, 0)
                
            elif Aruco_AngleD > neutral_angle:
                self.joystick.left(0, 0)
                
        while Aruco_PointPos != neutral_pos:
            if Aruco_PointPos < neutral_pos:
                self.joystick.forward(0, 0)
                
            elif Aruco_PointPos > neutral_pos:
                self.joystick.backward(0, 0)
                
        goal_handle.succeed() # A method that indicates the goal was successful4
        result = ArucoFollow.Result()
        return result # Result should return a boolean for completion

# DON'T FORGET TO "source install/setup.bash" BEFORE RUNNING ANYTHING WITH PYTHON
def main(args=None):
    rclpy.init(args=args)

    Arucofollow_action_server = ArucoFollow()

    rclpy.spin(Arucofollow_action_server)


if __name__ == '__main__':
    main()
