from math import pi, cos, sin, radians

#from boat import boat
from sailor import heading_control
from sailor import speed_control

import numpy as np

from web_backend import websocketserver
import threading


import time 
from datetime import datetime


"""
Lasciate ogne spearanza, voi ch'intrate 
(Abandon all hope, ye who enter here)

Low level controls
"""

cnt = 0

class Sailor:
    #def __init__(self, heading_control_method: str, speed_control_method: str) -> None:
    #def __init__(self, boat: boat.Boat, speed_control_method: str, heading_control_method: str) -> None:
    def __init__(self, boat: boat.Boat, speed_control_method, heading_control_method) -> None:

        """Creates a Sailor"""
        self.boat = boat
        self.speed_control = speed_control_method
        self.heading_control = heading_control_method

    # Function to read data from the serial port and log it
    def read_and_log_data(self):
          with open("logs/Test13.txt", 'a') as data:
            while True:
                self.boat.update()
                data.write(f'{datetime.now()}: {self.boat.gps.time=}, {self.boat.__str__()}\n')

                # Adjust the sleep time to achieve the desired frequency (100Hz)
                time.sleep(0.08)

    def run(self, desired_heading_angle):

        #Number of times sail is adjusted
        num_iterations = 5
        self.heading_control(desired_heading_angle)
        time.sleep(0.5)

        for i in range(num_iterations):
            time.sleep(0.2)
            self.speed_control()

        print(f"{self.speed_control = }, {self.heading_control = }, ")

    def test(self):
        for i in range(100):
            time.sleep(0.2)
            print(f"{self.boat.yaw_angle = }, {self.boat.pitch_angle = }, {self.boat.roll_angle = }")

if __name__ == "__main__":

    boat_width = 6
    boat_length = 6
    boat_mass = 62
    cargo_mass = 0

    #Wingsail size properties in meters
    wingsail_chord = 0.381
    wingsail_span = 2.02563984

    #Flap size properties in meters
    flap_chord = 0.17778984
    flap_span = 0.5586984

    #Rudder size properties in meters
    rudder_plan_area = 0.1
    rudder_span = 1 / 3

    #Moments of inertia in kg * m^2
    inertia_xx = 623.68
    inertia_yy = 623.92
    inertia_zz = 64.58

    test_boat = boat.Boat(boat_width, 
                        boat_length, 
                        boat_mass,
                        cargo_mass,
                        wingsail_chord,
                        wingsail_span,
                        flap_chord,
                        flap_span,
                        rudder_plan_area,
                        rudder_plan_area,
                        inertia_xx,
                        inertia_yy,
                        inertia_zz)


    #test_sailor = Sailor(test_boat, "extremum_seeking", "backup_heading_control")
    heading_control = heading_control.HeadingControl(test_boat)
    speed_control = speed_control.SpeedControl(test_boat)

    test_sailor = Sailor(test_boat, speed_control.extremum_seeking, heading_control.line_following_control)

    # Create threads for reading/logging data and sending periodic commands
    read_thread = threading.Thread(target = test_sailor.boat.sql_log)
    send_thread = threading.Thread(target = test_sailor.run)

    # Start the threads
    read_thread.start()
    send_thread.start()

    # Wait for the threads to finish (you may choose to handle this differently)
    read_thread.join()
    send_thread.join()


