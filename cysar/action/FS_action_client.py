import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action import FourSquare


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        '''
        Params:
        1. self - ROS 2 node to add the aciton client to.
        2. Type of action
        3. Action name
        '''
        self._action_client = ActionClient(self, FourSquare, 'FourSquare')
    

    def send_goal(self, order):
        goal_msg = FourSquare.Goal()
        goal_msg.order = order


        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()