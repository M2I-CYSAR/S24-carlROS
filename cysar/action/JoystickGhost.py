import rclpy
from cysar.msg import Joystick
import time

class JoystickGhost:
    def __init__(self) -> None:
        JoyStickList = []
        self.joystick = Joystick()
        self.joystick_publisher = self.create_publisher(Joystick, 'joystick', 10)
        self.joystick

    ''' 
    Initial combo theory. Combo could feature an array of inputs, with sub params specifying
    input intensity and time. 
    '''
    def combo(self, combo):
        for move in combo:
            action = move[0]
            if action[0] == "F":
                self.forward(action[1], action[2])
            elif action[0] == "B":
                self.backward(action[1], action[2])
            elif action[0] == "L":
                self.left(action[1], action[2])
            elif action[0] == "R":
                self.right(action[1], action[2])
        

    def forward(self, input, time):  
        while time.time() < time:
            self.joystick.stick_left_y = input # Float 
            self.joystick_publisher.publish(self.joystick)
        self.reset()

    def backward(self, input, time):
        while time.time() < time:
            self.joystick.stick_left_y = (input * -1) # Float 
            self.joystick_publisher.publish(self.joystick)
        self.reset()

    def left(self, input, time): # TEST NEG vs. POS
        while time.time() < time:
            self.joystick.stick_left_x = (input * -1) # Float 
            self.joystick_publisher.publish(self.joystick)
        self.reset()

    def right(self, input, time): # TEST NEG vs. POS
        while time.time() < time:
            self.joystick.stick_left_x = input # Float 
            self.joystick_publisher.publish(self.joystick)
        self.reset()

    def diagonal(self, input, time):
        pass

    def reset(self):
        self.joystick.stick_left_x = 0.0
        self.joystick.stick_left_y = 0.0
        self.joystick.stick_right_x = 0.0
        self.joystick.stick_right_y = 0.0
        self.joystick.trigger_left = 0.0
        self.joystick.trigger_right = 0.0
        self.joystick.button_a = False
        self.joystick.button_b = False
        self.joystick.button_x = False
        self.joystick.button_y = False
        self.joystick.bumper_left = False
        self.joystick.bumper_right = False
        self.joystick.button_back = False
        self.joystick.button_xbox = False
        self.joystick.button_start = False
        self.joystick.button_right_stick = False
        self.joystick.button_right_stick = False
        self.joystick.d_pad_up = False
        self.joystick.d_pad_down = False
        self.joystick.d_pad_left = False
        self.joystick.d_pad_right = False
        self.joystick_publisher.publish(self.joystick)

    def output(self, input, time):
        pass 
