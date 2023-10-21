#!/usr/bin/python3

"""
drive_control.py

Desc: Uses the CANbus interface to set the velocity of the motors.
Author: Isaac Denning
Date: 10/21/23
"""

from cysar.msg import DriveTrain
import SparkCANLib

# CAN IDs for Drive Controllers
FLD = 11
FRD = 12
BLD = 13
BRD = 14

INVERTED = -1

class DriveControl():
    """
    Uses the CANbus interface to set the velocity of the motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """
    def __init__(self, bus : SparkCANLib.SparkCAN.SparkBus): #TODO
        self.bus = bus
        self.FLMotor = self.bus.init_controller(FLD)
        self.FRMotor = self.bus.init_controller(FRD)
        self.BLMotor = self.bus.init_controller(BLD)
        self.BRMotor = self.bus.init_controller(BRD)

    def set_velocity(self, msg : DriveTrain):
        """
        Sets the velocity of the motors based on the ROS values.

        Args:
            msg (DriveTrain): The values from ROS indicating the velocity of each motor.
        """
        self.FLMotor.percent_output(msg.front_left)
        self.FRMotor.percent_output(INVERTED * msg.front_right)
        self.BLMotor.percent_output(msg.back_left)
        self.BRMotor.percent_output(INVERTED * msg.back_right)
