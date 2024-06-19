from math import sqrt, cos, sin, atan, atan2, pi, radians, degrees
from lib import config_loader, sailor_serial, sqllogger

import numpy as np

from helper_functions import *
from wing import wing
from rudder import rudder
from boat_gps import boat_gps

import time
import struct

cnt = 0

class Boat:
    """Holds all information about the boat and calculates its speed, forces and torques.
    Represents the boat from the master controller downward."""

    def __init__(self, 
                 boat_width: float, 
                 boat_length: float, 
                 boat_mass: float, 
                 cargo_mass: float,
                 wingsail_chord: float,
                 wingsail_span: float,
                 flap_chord: float,
                 flap_span: float,
                 rudder_plan_area: float,
                 rudder_span: float,
                 inertia_xx: float,
                 inertia_yy: float,
                 inertia_zz: float,
                )-> None:
        """
        Holds information for Sea++ Trimaran
        - Boat yaw angle is defined counterclockwise from the North, i.e. an angle of 0 is pointing true North
        - Boat speed is in m/s

        """
        #Serial Communication
        self.serial_cfg = config_loader.ConfigLoader().load_file("config/serial.toml")
        self.boat_cfg = config_loader.ConfigLoader().load_file("config/boat.toml")
        self.serial = sailor_serial.SailorSerial(self.serial_cfg, self.boat_cfg)

        time.sleep(1)

        # #Initializing SQL Logger
        # self.sqllogger

        #Boat Physical Properties
        self.boat_width = boat_width                                #Width of the boat in meters
        self.boat_length = boat_length                              #Length of the boat in meters
        self.boat_mass = boat_mass                                  #Weight of the boat in kg
        self.total_mass = boat_mass + cargo_mass                    #Total weight of the boat in kg
        self.inertia_xx = inertia_xx                                #Moment of inertia of the sailboat in xx
        self.inertia_yy = inertia_yy                                #Moment of inertia of the sailboat in yy
        self.inertia_zz = inertia_zz                                #Moment of inertia of the sailboat in zz

        #Boat Physical Constraints
        self.boat_constraints = self.boat_cfg.data.physical_constraints
        self.min_upwind_angle = self.boat_constraints.min_upwind_angle
        self.min_downwind_angle = self.boat_constraints.min_downwind_angle
        self.min_rudder_angle = self.boat_constraints.min_rudder_angle
        self.max_rudder_angle = self.boat_constraints.max_rudder_angle
        self.min_flap_angle = self.boat_constraints.min_flap_angle
        self.max_flap_angle = self.boat_constraints.max_flap_angle
        self.max_sail_angle = self.boat_constraints.max_sail_angle
        self.min_sail_angle = self.boat_constraints.min_sail_angle

        #Inertial Measurement Unit (IMU) Information
        self.speed_x: float = 0                                #Boat surge speed in m/s (North)
        self.speed_y: float = 0                                #Boat sway speed in m/s (East)
        self.speed_z: float = 0                                #Boat heave speed in m/s (Down)

        self.roll_angle: float = 0                             #Boat rotation about x-axis counterclockwise in degrees
        self.pitch_angle: float = 0                            #Boat rotation about y-axis counterclockwise in degrees
        self.yaw_angle: float = 0                              #Boat rotation about z-axis counterclockwise in degrees

        #Sensor Information
        self.app_wind_magnitude: float = 0                          #Apparent wind magnitude in m/s
        self.app_wind_angle: float = 0                              #Apparent wind angle in degree. 0 apparent wind is defined as head on to boat

        #Actuator Information
        self.wingsail_angle: float = 0                              #Relative angle of wingsail to boat
        self.flap_angle: float = 0                                  #Relative angle of flap to wingsail
        self.rudder_angle: float = 0                                #Angle of rudders

        #Initialize gps, wingsail, flap, and rudder classes
        self.gps = boat_gps.BoatGPS()
        self.wingsail = wing.Wingsail(wingsail_chord, wingsail_span)
        self.flap = wing.Flap(flap_chord,flap_span)
        self.port = rudder.Rudder(rudder_plan_area, rudder_span)
        self.starboard = rudder.Rudder(rudder_plan_area, rudder_span)

        #Derived Parameters
        self.speed: float = 0                                       #Speed of boat
        self.true_wind_magnitude: float = 0                         #True wind magnitude in m/s
        self.true_wind_angle: float = 0                             #True wind angle in degrees

        # Characterisation of sailing abilities
        self.min_upwind_angle: float = 0
        self.min_downwind_angle: float = 0

        #NOT SURE IF DERIVED OR GIVE
        self.roll_angular_velocity: float = 0
        self.pitch_angular_velocity: float = 0
        self.yaw_angular_velocity: float = 0

        #Boat position for convenience
        self.coords = [self.gps.lat, self.gps.lon]
        
        # self.magnetometer: [0,0,0]

    def update(self):

        serial_status = False
        gps_status = False
        time.sleep(0.01)

        try:
            serial_status, received_data_dict = self.serial.receive_all()
            #print(received_data_dict)
        except UnicodeDecodeError:
            print("Message incomplete or ill-formed")
        except struct.error as err:
            print("Incomplete packet")

        if serial_status:
            
            temp_euler = received_data_dict['euler']
            temp_magnetometer = received_data_dict['magnetometer']
            temp_acceleration = received_data_dict['acceleration']
            temp_wind_speed = received_data_dict['wind_speed']
            temp_wind_angle = received_data_dict['wind_angle']

            if temp_euler != None:
                self.roll_angle, self.pitch_angle, self.yaw_angle = temp_euler

            # if temp_magnetometer != None:
            #     self.magnetometer = temp_magnetometer
            # if temp_acceleration != None:
            #     self.acceleration = temp_acceleration
            if temp_wind_speed != None:
                self.app_wind_magnitude = temp_wind_speed
            if temp_wind_angle != None:
                self.app_wind_angle = temp_wind_angle

            self.gps.update()
            self.wingsail.update(received_data_dict)
            self.flap.update(received_data_dict)
            self.update_true_wind()


    def read_and_log_data(self):
        print("starting")
        sqll = sqllogger.SQLLogger("/home/antonio/code/guide-nav/databases/NavigationData.db")
        while True:
            self.update()
            print("e")
            self.sql_log(sqll)

            time.sleep(0.08)
    
    
    def update_boat_speed(self):
        self.speed_x = self.gps.speed * cos(radians(self.yaw_angle))
        self.speed_y = self.gps.speed * sin(radians(self.yaw_angle))
        speed_magnitude = sqrt(self.speed_x ** 2 + self.speed_y ** 2)
        return speed_magnitude


    def send_rudder_angle(self, input_rudder_angle):
        self.serial.send_rudder(input_rudder_angle, "multi")
    

    def send_flap_angle(self, input_flap_angle):
        self.serial.send_flap(input_flap_angle)
    

    def update_true_wind(self):
        self.update_boat_speed()

        #Apparent wind velocity in x-direction
        apparent_wind_x = self.app_wind_magnitude * cos(radians(self.app_wind_angle))
        #Apparent wind velocity in y-direction
        apparent_wind_y = self.app_wind_magnitude * sin(radians(self.app_wind_angle))

        #True wind velocity in x-direction
        true_wind_x = apparent_wind_x - self.speed_x
        #True wind velocity in y-direction
        true_wind_y = apparent_wind_y - self.speed_y

        self.true_wind_angle = degrees(atan2(true_wind_y, true_wind_x))

        if self.true_wind_angle < 0:
            self.true_wind_angle += 360
        
        self.true_wind_magnitude = sqrt(true_wind_x ** 2 + true_wind_y ** 2)


    def control_flap_angle(self, desired_wingsail_angle_of_attack):
        """
        P-Control to adjust to maintain desired wingsail angle of attack
        """
        #https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9239150&tag=1
        input_flap_angle = desired_wingsail_angle_of_attack / 3.21
        self.send_flap_angle(input_flap_angle)
        print(f"{input_flap_angle = }")


    def __str__(self):
        return f"{self.flap.relative_angle = }, {self.wingsail.absolute_angle = }, {self.app_wind_magnitude = }, "\
               f"{self.app_wind_angle = }, {self.gps.lat = }, {self.gps.lon = }, {self.gps.speed = }, "\
               f"{self.pitch_angle = }, {self.roll_angle = }, {self.yaw_angle = }, "\
               f"{self.true_wind_magnitude = }, {self.true_wind_angle = }"
    

    def sql_log(self, sqllogger):
        # wind
        sqllogger.log_wind_data(self.app_wind_magnitude, self.app_wind_angle)
        # pitch, yaw, roll
        sqllogger.log_imu_data(self.pitch_angle, self.roll_angle, self.yaw_angle)
        # GPS
        sqllogger.log_gps_data(self.gps.lat, self.gps.lon, self.gps.speed)
        # Flap
        sqllogger.log_motor_data("Flap", self.flap.relative_angle, self.wingsail.absolute_angle)
    
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

    test_boat = Boat(boat_width, 
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


    for i in range(10000):
        time.sleep(0.01)
        test_boat.update()
        print(test_boat)
    