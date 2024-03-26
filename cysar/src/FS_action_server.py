import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from src.action import FourSquare 


class FourSquareActionServer(Node):

    def __init__(self): # Constructor
        super().__init__('FS_action_server')
        '''
        1. ROS 2 node to add the server to.
        2. Type of action
        3. Action name
        4. A callback function for executing accepted goals: self.execute_callback. This callback must return a result message for the action type.
        '''
        self._action_server = ActionServer(
            self,
            FourSquare,
            'FourSquare',
            self.execute_callback)

    ''' 
    The Method called to execute an action. This method handles 3 message types.
    1. A Goal Service - A Request sent from the client, to the server. The Server can send back a response.
    2. A Feedback Topic - A middle line that the client can access to receive notes on task progress from the server.
    3. A Result Service - A final request from the client to the server. A response is sent back from the server.
    '''
    def execute_callback(self, goal_handle): # Method called - Goal_Handle Param input = time
        self.get_logger().info('Executing goal...')
        result = FourSquare.Result()
        '''
        Action Server will just need to publish data to Joystick.msg.
        This will take testing to get the proper Joystick Values required for changing velocity, especially
        in cases where we want to turn the robot in different ways.
        - Factors to keep in mind.
        (Time) - How long should a specific joystick input be kept in a position to... [turn left, right, etc.]
        (Terrain) - What factors of the terrain could effect how well a robot maneuvers while automated [could be important
        since we don't want to start an autonoumous program, and the terrain causes the robot to move not as expected] 
        '''

        #TODO: Test Case
        # Run an action execution and output new joystick values from terminal.

        goal_handle.succeed() # A method that indicates the goal was successful
        return result # Result should return a boolean for completion


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FourSquareActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()