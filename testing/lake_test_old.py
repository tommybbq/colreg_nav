from math import pi, cos, sin, radians

from boat import boat
from sailor import heading_control
from sailor import speed_control
from sailor import sailor
from local_planning import local_pathfinder

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


def simple_lpp_test(boat: boat.Boat, heading_control, speed_control, test_waypoints):
    """
    Test 5: Confirm simple LPP. Can go two waypoints
    """

    simple_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.line_following_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)

    simple_lpp = local_pathfinder.LocalPathfinder(sailor, test_waypoints)

    simple_lpp.run()

    simple_sailor.rc_run()

def full_lpp_test(boat: boat.Boat, heading_control, speed_control, global_waypoints):
    """
    Test 6: Fake global path planning
    """
    full_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.line_following_control,
                              default_speed_control_method=speed_control.rc_control, 
                              default_heading_control_method=heading_control.rc_control)

    full_lpp = local_pathfinder.LocalPathfinder(sailor, global_waypoints)

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

    #TODO Change if necessary: 
    desired_heading_angle = 330

    heading_control = heading_control.HeadingControl(test_boat)
    speed_control = speed_control.SpeedControl(test_boat)

    # Create threads for reading/logging data and sending periodic commands
    read_thread = threading.Thread(target = test_boat.read_and_log_data)
    #send_thread = threading.Thread(target = rc_check(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target = find_no_go(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target = polar_diagram(test_boat, heading_control, speed_control))
    send_thread = threading.Thread(target = simple_speed_test(test_boat, heading_control, speed_control))
    #send_thread = threading.Thread(target = simple_heading_test(test_boat, heading_control, speed_control, desired_heading_angle))
    
    send_thread = threading.Thread(target = data_collection(test_boat))

    # Start the threads
    read_thread.start()
    send_thread.start()

    # Wait for the threads to finish (you may choose to handle this differently)
    read_thread.join()
    send_thread.join()


