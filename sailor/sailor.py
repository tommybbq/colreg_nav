from sailor import heading_control
from sailor import speed_control
from boat import boat

from math import pi, cos, sin, radians
import numpy as np


from web_backend import websocketserver
import threading

import time 
from datetime import datetime


"""
Lasciate ogne spearanza, voi ch'intrate 
(Abandon all hope, ye who enter here)
"""

cnt = 0

class Sailor:
    def __init__(self, boat: boat.Boat, speed_control_method, heading_control_method, 
                 default_speed_control_method=None, default_heading_control_method= None) -> None:
        """
        Creates a Sailor for low level controls
        """
        self.boat = boat
        self.speed_control = speed_control_method
        self.heading_control = heading_control_method
        self.default_speed_control = default_speed_control_method
        self.default_heading_control = default_heading_control_method


    def read_and_log_data(self):
        """
        Function to read data from serial port and log it
        - This is antiquated with introduction of SQL
        """
        with open("logs/Test_home1.txt", 'a') as data:
            while True:
                self.boat.update()
                data.write(f'{datetime.now()}: {self.boat.gps.time=}, {self.boat.__str__()}\n')

                # Adjust the sleep time to achieve the desired frequency (100Hz)
                time.sleep(0.08)


    def run(self, desired_heading_angle):
        """
        Runs heading and speed control
        """
        #TODO: Clean up these timings

        #Number of times sail is adjusted
        #num_iterations = 5

        self.heading_control(desired_heading_angle)

        # for i in range(num_iterations):
        #     time.sleep(2)
        self.speed_control()

    def rc_run(self):
        self.heading_control(0)
        self.speed_control()
    

    def speed_run(self):
        self.heading_control(0)

        num_iterations = 500

        for i in range(num_iterations):
            time.sleep(2)
            self.speed_control()

            print(f"{self.boat.gps.speed = }")
    
    
    def heading_run(self, desired_heading_angle):

        self.speed_control()

        num_iterations = 500

        for i in range(num_iterations):
            time.sleep(2)
            self.heading_control(desired_heading_angle)

        
    def default_control(self):
        self.speed_control = self.default_speed_control
        self.heading_control = self.default_heading_control


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
    read_thread = threading.Thread(target = test_sailor.read_and_log_data)
    send_thread = threading.Thread(target = test_sailor.run(300))

    # Start the threads
    read_thread.start()
    send_thread.start()

    # Wait for the threads to finish (you may choose to handle this differently)
    read_thread.join()
    send_thread.join()


