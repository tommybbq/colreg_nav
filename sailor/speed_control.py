from math import pi, cos, radians

from boat import boat

from web_backend import websocketserver

import numpy as np

cnt = 0

class SpeedControl():
    def __init__(self, boat: boat.Boat) -> None:
        self.boat = boat


    def extremum_seeking(self):
        """
        Extremum Seeking Control:
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9307155
        - Reaches optimal speed without requiring system
        - Requires convergence time and may have steady state oscillation
        """

        # TIME_MAX = 0
        # TIME_MIN = 0
        # desired_wingsail_angle = 30

        global cnt
        eps = 0.1 # min speed
        speed_flag = True
        wingsail_angle_minus_2: float = 0
        wingsail_angle_minus_1: float = 0
        boat_speed_minus_1: float = 0

        angle_step_size = 5

        #Initial conditions
        if cnt < 2:
            if self.boat.app_wind_angle >= 180:
                desired_wingsail_angle_of_attack = -30

            elif self.boat.app_wind_angle < 180:
                desired_wingsail_angle_of_attack = 30

            self.boat.control_flap_angle(desired_wingsail_angle_of_attack)

            if cnt < 1:
                wingsail_angle_minus_2 = self.boat.wingsail.absolute_angle
            else:
                wingsail_angle_minus_1 = self.boat.wingsail.absolute_angle

            boat_speed_minus_1 = self.boat.speed

        else:
            #Extremum seeking
            if self.boat.speed < eps:
                speed_flag = False
            wingsail_angle_change = wingsail_angle_minus_1 - wingsail_angle_minus_2
            boat_speed_change = self.boat.speed - boat_speed_minus_1

            desired_wingsail_angle = self.boat.wingsail.absolute_angle + angle_step_size * np.sign(wingsail_angle_change) * np.sign(boat_speed_change)
            desired_wingsail_angle_of_attack = self.boat.wingsail.calculate_desired_angle_of_attack(desired_wingsail_angle, self.boat.app_wind_angle)
            print(f"{desired_wingsail_angle_of_attack = }")
            print(f"{self.boat.app_wind_angle = }")
            self.boat.control_flap_angle(desired_wingsail_angle_of_attack)

            wingsail_angle_minus_2 = wingsail_angle_minus_1
            wingsail_angle_minus_1 = self.boat.wingsail.absolute_angle

            boat_speed_minus1 = self.boat.speed

        cnt += 1

        #TODO: figure out how to dynamically change T and alpha
        #Utilize information about how much speed is changing to force updates
        #If speed is only oscillating it doesnt need to do anything
        return speed_flag
            
    
    def linear_sail_angle(self):
        """
        Linear Sail Angle
        - Method used to control speed of boat
        """
        #TODO: NOT SURE IF IN RADIANS OR DEGREES
        max_sail_angle = pi / 2 * ((cos(radians(self.boat.true_wind_angle - self.boat.yaw_angle)) + 1) / 2)
        sail_angle = (max_sail_angle -self.boat.min_sail_angle) * self.boat.app_wind_angle / pi + self.boat.min_sail_angle

    
    def rc_control(self):
        ws = websocketserver.WebSocketServer(addr="10.0.0.3")
        if not ws.start():
            print("Something went wrong")

        self.boat.serial.flap_timeout = 0.5

        while True:
            if ws.read_avaliable():
                part, data = ws.read_in_order()
                if part == "rudder":
                    print("Rudder:", data)
                    self.boat.send_rudder_angle(data)
                elif part == "sail":
                    print("Flap:", data)
                    self.boat.send_flap_angle(data)
                else:
                    print("Bad Data?")
    

    def simple_speed_control(self):
        if self.boat.app_wind_angle >= 180:
            desired_wingsail_angle_of_attack = -30

        elif self.boat.app_wind_angle < 180:
            desired_wingsail_angle_of_attack = 30

        self.boat.control_flap_angle(desired_wingsail_angle_of_attack)

    # def backup_speed_control(self):
    #     if self.boat.app_wind_angle >= 180:
    #         desired_wingsail_angle_of_attack = -50

    #     elif self.boat.app_wind_angle < 180:
    #         desired_wingsail_angle_of_attack = 50

    #     self.boat.control_flap_angle(desired_wingsail_angle_of_attack)
