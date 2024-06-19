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

    def __init__(self)-> None:
        """
        Holds information for Sea++ Trimaran
        - Boat yaw angle is defined counterclockwise from the North, i.e. an angle of 0 is pointing true North
        - Boat speed is in m/s

        """
        #Serial Communication
        self.serial_cfg = config_loader.ConfigLoader().load_file("config/serial.toml")
        self.boat_cfg = config_loader.ConfigLoader().load_file("config/boat.toml")
        self.serial = sailor_serial.SailorSerial(self.serial_cfg, self.boat_cfg)
        # time.sleep(1)
        
        #Boat Physical Constraints
        self.boat_constraints = self.boat_cfg.data.physical_constraints
        self.boat_properties = self.boat_cfg.data.physical_properties
        self.min_upwind_angle = self.boat_constraints.min_upwind_angle
        self.min_downwind_angle = self.boat_constraints.min_downwind_angle
        self.min_rudder_angle = self.boat_constraints.min_rudder_angle
        self.max_rudder_angle = self.boat_constraints.max_rudder_angle
        self.min_flap_angle = self.boat_constraints.min_flap_angle
        self.max_flap_angle = self.boat_constraints.max_flap_angle
        self.max_sail_angle = self.boat_constraints.max_sail_angle
        self.min_sail_angle = self.boat_constraints.min_sail_angle

        #Boat Physical Properties
        self.wingsail_chord = self.boat_properties.wingsail_chord
        self.wingsail_span = self.boat_properties.wingsail_span
        self.flap_chord = self.boat_properties.flap_chord
        self.flap_span = self.boat_properties.flap_span
        self.rudder_plan_area = self.boat_properties.rudder_plan_area
        self.rudder_span = self.boat_properties.rudder_span

        #Inertial Measurement Unit (IMU) Information
        self.calibration_order = ["System", "Gyro", "Accel", "Mag"]
        self.speed_x: float = 0                                #Boat surge speed in m/s (North)
        self.speed_y: float = 0                                #Boat sway speed in m/s (East)
        self.speed_z: float = 0                                #Boat heave speed in m/s (Down)
        while not self.check_imu_calibration():
            time.sleep(0.1)

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
        self.wingsail = wing.Wingsail(self.wingsail_chord, self.wingsail_span)
        self.flap = wing.Flap(self.flap_chord, self.flap_span)
        self.port = rudder.Rudder(self.rudder_plan_area, self.rudder_span)
        self.starboard = rudder.Rudder(self.rudder_plan_area, self.rudder_span)

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
        # self.coords = [self.gps.lat, self.gps.lon]
        
        # self.magnetometer: [0,0,0]

    def check_imu_calibration(self) -> bool:
        try:
            serial_status, received_data_dict = self.serial.receive_all()
        except UnicodeDecodeError:
            print("Message incomplete or ill-formed")
            return False
        except struct.error:
            print("Incomplete packet")
            return False

        if serial_status:
            calibration_values = received_data_dict["initialization"]
            if calibration_values:  # did calibration data actually get sent
                calibration_statuses = [calibration_value >= 3 for calibration_value in calibration_values]  # True if calibrated
                if sum(calibration_statuses) < 4:
                    print("IMU still calibrating...")
                    for i, calibration_status in calibration_statuses:
                        if not calibration_status:
                            print(f"\t{self.calibration_order[i]}: {calibration_values[i]}")
                    return False
                else:
                    return True
            else:
                print("No IMU calibration data received")
                return False
        else:
            print("No serial data received")
            return False

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
    

    def send_port_angle(self, input_rudder_angle):
        self.serial.send_rudder(input_rudder_angle, "port")

    
    def send_starboard_angle(self, input_rudder_angle):
        self.serial.send_rudder(input_rudder_angle, "starboard")

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

    test_boat = Boat()

    for i in range(10000):
        time.sleep(0.01)
        test_boat.update()
        print(test_boat)
    
