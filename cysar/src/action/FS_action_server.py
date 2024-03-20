import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from src.action import FourSquare


class FourSquareActionServer(Node):

    def __init__(self):
        super().__init__('FS_action_server')
        self._action_server = ActionServer(
            self,
            FourSquare,
            'FourSquare',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = FourSquare.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FourSquareActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()