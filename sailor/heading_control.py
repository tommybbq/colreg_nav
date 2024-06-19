from math import radians, degrees, cos, sin

from boat import boat

import numpy as np

class HeadingControl():

    def __init__(self, boat: boat.Boat) -> None:
        self.boat = boat

    def line_following_control(self, desired_heading_angle):
        
        heading_difference = radians(self.boat.yaw_angle - desired_heading_angle)

        if cos(heading_difference) >= 0:
            input_rudder_angle = self.boat.max_rudder_angle * sin(heading_difference)
        else:
            input_rudder_angle = self.boat.max_rudder_angle * np.sign(sin(heading_difference))
        self.boat.send_rudder_angle(input_rudder_angle)
        print(f"{input_rudder_angle = }")

    def rc_control(self, desired_heading_angle):
        pass
    
    def backup_heading_control(self, input_rudder_angle):
        """
        Heading control to default a rudder angle
        """
        self.boat.send_rudder_angle(input_rudder_angle)
        print("Angle sent")
    