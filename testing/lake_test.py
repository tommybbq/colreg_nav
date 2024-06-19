from math import pi, cos, sin, radians
from helper_functions import magnitude, haversine, read_csv_to_list

from boat import boat
from sailor import heading_control
from sailor import speed_control
from sailor import sailor
from local_planning import local_pathfinder
from lib.sqllogger import SQLLogger
from object_avoidance import camera

import csv
import numpy as np

from web_backend import websocketserver
import threading

import time 
from datetime import datetime

def rc_check(boat: boat.Boat, heading_control, speed_control):
    "Test 0: Verify manual control of rudders, sail actuator, and produce lift"
    rc_sailor = sailor.Sailor(boat, speed_control.rc_control, heading_control.rc_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)
    
    rc_sailor.rc_run()


def find_no_go(boat: boat.Boat, heading_control, speed_control):
    "Test 1: No-go zone characterization"

    rc_sailor = sailor.Sailor(boat, speed_control.rc_control, heading_control.rc_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)
    
    rc_sailor.rc_run()


def polar_diagram(boat: boat.Boat, heading_control, speed_control):
    "Test 2: No-go zone characterization"

    rc_sailor = sailor.Sailor(boat, speed_control.rc_control, heading_control.rc_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)
    
    rc_sailor.rc_run()


def simple_speed_test(boat: boat.Boat, heading_control, speed_control):
    """
    Test 3: Confirm boat speed increases using extremum seeking
    """

    speed_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.backup_heading_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)

    speed_sailor.speed_run()

    speed_sailor.default_control()
    speed_sailor.rc_run()


def simple_heading_test(boat: boat.Boat, heading_control, speed_control, desired_heading_angle):
    """
    Test 4: Confirm boat can maintain heading using line following control
    """

    heading_sailor = sailor.Sailor(boat, speed_control.backup_speed_control, heading_control.line_following_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)

    heading_sailor.heading_run(desired_heading_angle)

    heading_sailor.default_control()
    heading_sailor.rc_run()

    sailor.boat.send_rudder_angle(50)
    time.sleep(2)
    sailor.heading_control(300)


def simple_lpp_test(boat: boat.Boat, heading_control, speed_control):
    """
    Test 5: Confirm simple LPP. Can go two waypoints
    """

    simple_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.line_following_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)

    delta_l = 0.00036 #adds about 40 m north direction
    #print(haversine([current_lat,current_lon],[current_lat,current_lon+delta_l]))
    current_lat = simple_sailor.boat.gps.lat
    current_lon = simple_sailor.boat.gps.lon
   
    test_waypoints = [[current_lat,current_lon],[current_lat,current_lon+delta_l]]

    simple_lpp = local_pathfinder.LocalPathfinder(simple_sailor, test_waypoints)
    simple_lpp.run()

    simple_sailor.rc_run()

def station_keeping_lpp_test(boat: boat.Boat, heading_control, speed_control):
    """
    Test 6: Fake global path planning
    """
    full_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.line_following_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)

    delta_l = 0.0007 #adds about 77 m both directions, 109 m hypotenuse
    current_lat = full_sailor.boat.gps.lat
    current_lon = full_sailor.boat.gps.lon
    waypoint_start = [current_lat,current_lon]

    ##TODO: option (1) Global waypoints assumes NSEW wind, pick as appropriate
    # waypoint_second = [waypoint_start[0]-delta_l,waypoint_start[0]+delta_l] #-x,+y
    # waypoint_third = [waypoint_second[0]+delta_l,waypoint_second[0]+delta_l] # +x,+y
    # waypoiny_fourth = [waypoint_third[0]+delta_l,waypoint_third[0]-delta_l] #+x,-y
    # global_waypoints = [waypoint_start,waypoint_second,waypoint_third,waypoiny_fourth,waypoint_start]

    ##TODO: option (2) Global waypoints assumes diagonal wind, change as appropriate
    # waypoint_second = [waypoint_start[0],waypoint_start[0]+delta_l] #+y
    # waypoint_third = [waypoint_second[0]+delta_l,waypoint_second[0]] # +x
    # waypoiny_fourth = [waypoint_third[0],waypoint_third[0]-delta_l] #-y
    # global_waypoints = [waypoint_start,waypoint_second,waypoint_third,waypoiny_fourth,waypoint_start]


    station_keeping_lpp = local_pathfinder.LocalPathfinder(full_sailor, global_waypoints)

    station_keeping_lpp.run()

    sailor.rc_run()

def full_lpp_test(boat: boat.Boat, heading_control, speed_control,global_waypoints):
    """
    Test 7: global path planning with real waypoints from optimal sailor
    """
    full_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.line_following_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)

    current_lat = full_sailor.boat.gps.lat
    current_lon = full_sailor.boat.gps.lon
    waypoint_start = [current_lat,current_lon]

    #add starting location as first wayipoint if not already feature
    #global_waypoints.insert(0,waypoint_start)

    full_lpp = local_pathfinder.LocalPathfinder(full_sailor, global_waypoints)

    full_lpp.run()

    sailor.rc_run()


def data_collection(boat: boat.Boat):
    """
    Test 7: Collect data for Gaussian MPC
    https://www.mdpi.com/2077-1312/9/12/1420
    """
    num_iterations = 500

    for i in range(num_iterations):
        random_angle = np.random.uniform(low = boat.min_rudder_angle, high = boat.max_rudder_angle)
        boat.send_rudder_angle(random_angle)
        print(f"{random_angle = }")
        time.sleep(2)


if __name__ == "__main__":

    #Following code if for pond test Feb 10

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

    test_boat = boat.Boat()

    #TODO Change if necessary: 
    desired_heading_angle = 330

    # local_waypoints = read_csv_to_list("coords.csv")
    # local_waypoints = [[29.555989, -95.068363],[29.55858,-95.06726]]
    # global_waypoints = local_waypoints


    heading_control = heading_control.HeadingControl(test_boat)
    speed_control = speed_control.SpeedControl(test_boat)

    def read_and_log_data():
        sqllogger = SQLLogger("/home/antonio/guide-nav/databases/LakeTestDatabase.db")
        while True:
            test_boat.update()
            test_boat.sql_log(sqllogger)
            time.sleep(0.08)                            

    def log_pictures():
        cam = camera.Camera()
        while True:
            cam.capture("/home/antonio/Pictures")
            time.sleep(15)

    # Create threads for reading/logging data and sending periodic commands
    read_thread = threading.Thread(target = read_and_log_data, daemon = True)
    send_thread = threading.Thread(target = rc_check, args=(test_boat, heading_control, speed_control), daemon = True)
    #send_thread = threading.Thread(target = find_no_go(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target = polar_diagram(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target = simple_speed_test(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target = simple_heading_test(test_boat, heading_control, speed_control, desired_heading_angle))
    #send_thread = threading.Thread(target=simple_lpp_test(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target=station_keeping_lpp_test(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target=full_lpp_test(test_boat, heading_control, speed_control,global_waypoints))
    #send_thread = threading.Thread(target = data_collection(test_boat))
    #camera_thread = threading.Thread(target=log_pictures)

    # Start the threads
    read_thread.start()
    send_thread.start()
    #camera_thread.start()

    # Wait for the threads to finish (you may choose to handle this differently)
    read_thread.join()
    send_thread.join()
    #camera_thread.join()
