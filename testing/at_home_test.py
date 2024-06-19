from math import atan2, cos, sin, radians, atan, degrees, pi

import time
import numpy as np

from helper_functions import haversine, magnitude, find_bearing

time_last_call = time.time()
parameter_estimate: float = 0.0
waypoint_index = 0



def home_los(current_location, waypoints):
        """
        Adaptive Line of Sight Algorithm:
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10087026

        """
        #Coefficients of significance
        waypoint_radius = 10                                        #Waypoint radius in meters
        lookahead_dist = 10                                         #Lookahead distance in meters
        adaptation_gain = 0.05                                      #Adaptation gain
        #parameter_estimate: float = 0.0                             #Parameter estimate

        time_current_call: float = time.time()
        global time_last_call
        time_step = time_current_call - time_last_call              #Difference in time between last function call and current function call

        #TODO: Figure out if north-east coordinate is used
        #TODO: Determine desired lookahead distance and waypoint radius
        #TODO: Need to figure out crab angle
        #TODO: Need to figure out units
        #TODO: add flag for obstacle avoidance

        global waypoint_index
        #Current waypoint from waypoint list
        current_waypoint = waypoints[waypoint_index]
        current_location = current_location

        #Distance between current location and next waypoint
        distance_to_waypoint = haversine(current_location, current_waypoint)

        next_waypoint_index = waypoint_index + 1
        
        if next_waypoint_index <= waypoint_index + 1:
            next_waypoint = waypoints[next_waypoint_index]
            waypoint_distance = haversine(current_waypoint, next_waypoint)
            azimuth_angle = find_bearing(current_waypoint, next_waypoint)
            azimuth_angle = radians(azimuth_angle)
            
            r_matrix = np.array([[cos(azimuth_angle), -sin(azimuth_angle)], 
                                [sin(azimuth_angle), cos(azimuth_angle)]])
            
            # Difference between current point and waypoint in latitude and longitude
            current_lat_difference = haversine([current_waypoint[0], 0], [current_location[0], 0])
            current_lon_difference = haversine([0, current_waypoint[1]], [0, current_location[1]])

            position_difference_vector = np.array([[current_lat_difference], [current_lon_difference]])
            position_error_vector = np.dot(np.transpose(r_matrix), position_difference_vector)

            along_track_error = position_error_vector[0][0]
            cross_track_error = position_error_vector[1][0]

            parameter_estimate_derivative = adaptation_gain * (lookahead_dist / magnitude([lookahead_dist, cross_track_error])) * cross_track_error

            global parameter_estimate
            if abs(parameter_estimate_derivative) >= 2.0:
                parameter_estimate += parameter_estimate_derivative * time_step

            desired_heading_angle = azimuth_angle - parameter_estimate - atan(cross_track_error / lookahead_dist)
            desired_heading_angle = (360 + degrees(desired_heading_angle)) % 360


            bearing_angle = find_bearing(waypoints[waypoint_index], waypoints[waypoint_index + 1])
            bearing_angle = (360 + bearing_angle) % 360
            print(f"{bearing_angle = }")
            print(f"{desired_heading_angle = }")
            time_last_call = time.time()

            return desired_heading_angle

if __name__ == "__main__":
    test_waypoints = [[29.720532, -95.402756], [29.720215, -95.403554]]
    test_test_waypoints = [[40.7128, -74.0060], [34.0522,-118.2437]]
    #flag_test(test_sailor,test_waypoints)
    within_goal_waypoint = [29.720181833333335, -95.403487500000000] #within goal radius point
    midpoint_waypoint = [29.720340999999998, -95.403155000000000] #midpoint
    outbounds_waypoint = [29.720273333000000, -95.402558333000000]          #out of bounds to right
    # inv_waypoint = [29.720273333, -95.402558333000000] # cross track point

    crabbing_waypoint = [30,-96]

    goal_result = home_los(test_waypoints[1], test_waypoints)

# from math import pi, cos, sin, radians
# from helper_functions import magnitude, haversine

# from boat import boat
# from sailor import heading_control
# from sailor import speed_control
# from sailor import sailor
# from local_planning import local_pathfinder

# #jordan change
# from local_planning import local_pathfinder_jmcopy

# import numpy as np

# from web_backend import websocketserver
# import threading

# import time 
# from datetime import datetime

# def rc_check(boat: boat.Boat, heading_control, speed_control):
#     "Test 0: Verify manual control of rudders, sail actuator, and produce lift"
#     rc_sailor = sailor.Sailor(boat, speed_control.rc_control, heading_control.rc_control,
#                               default_speed_control_method=speed_control.rc_control, 
#                               default_heading_control_method=heading_control.rc_control)
    
#     rc_sailor.rc_run()


# def find_no_go(boat: boat.Boat, heading_control, speed_control):
#     "Test 1: No-go zone characterization"

#     rc_sailor = sailor.Sailor(boat, speed_control.rc_control, heading_control.rc_control,
#                               default_speed_control_method=speed_control.rc_control, 
#                               default_heading_control_method=heading_control.rc_control)
    
#     rc_sailor.rc_run()


# def polar_diagram(boat: boat.Boat, heading_control, speed_control):
#     "Test 2: No-go zone characterization"

#     rc_sailor = sailor.Sailor(boat, speed_control.rc_control, heading_control.rc_control,
#                               default_speed_control_method=speed_control.rc_control, 
#                               default_heading_control_method=heading_control.rc_control)
    
#     rc_sailor.rc_run()


# def simple_speed_test(boat: boat.Boat, heading_control, speed_control):
#     """
#     Test 3: Confirm boat speed increases using extremum seeking
#     """

#     speed_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.backup_heading_control,
#                               default_speed_control_method=speed_control.rc_control, 
#                               default_heading_control_method=heading_control.rc_control)

#     speed_sailor.speed_run()

#     speed_sailor.default_control()
#     speed_sailor.rc_run()


# def simple_heading_test(boat: boat.Boat, heading_control, speed_control, desired_heading_angle):
#     """
#     Test 4: Confirm boat can maintain heading using line following control
#     """

#     heading_sailor = sailor.Sailor(boat, speed_control.backup_speed_control, heading_control.line_following_control,
#                               default_speed_control_method=speed_control.rc_control, 
#                               default_heading_control_method=heading_control.rc_control)

#     heading_sailor.heading_run(desired_heading_angle)

#     heading_sailor.default_control()
#     heading_sailor.rc_run()

#     sailor.boat.send_rudder_angle(50)
#     time.sleep(2)
#     sailor.heading_control(300)


# def simple_lpp_test(boat: boat.Boat, heading_control, speed_control):
#     """
#     Test 5: Confirm simple LPP. Can go two waypoints
#     """

#     simple_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.line_following_control,
#                               default_speed_control_method=speed_control.extremum_seeking, 
#                               default_heading_control_method=heading_control.line_following_control)

#     delta_l = 0.00036 #adds about 40 m north direction
#     #print(haversine([current_lat,current_lon],[current_lat,current_lon+delta_l]))
#     current_lat = simple_sailor.boat.gps.lat
#     current_lon = simple_sailor.boat.gps.lon

#     #test 
    
#     #jordan change
#     test_waypoints = [[29.154156, -94.96709654545455],[29.154156, -94.96709654545455+delta_l]]

#     #jordan change remove 1 test
#     simple_lpp = local_pathfinder.LocalPathfinder(simple_sailor, test_waypoints,test_waypoints)
#     simple_lpp.run()

#     simple_sailor.rc_run()

# def full_lpp_test(boat: boat.Boat, heading_control, speed_control, test_waypoints,fake_locations):
#     #jordan get rid of fake
#     """
#     Test 6: Fake global path planning
#     """
#     full_sailor = sailor.Sailor(boat, speed_control.extremum_seeking, heading_control.line_following_control,
#                               default_speed_control_method=speed_control.extremum_seeking, 
#                               default_heading_control_method=heading_control.line_following_control)

#     #speed_control.extremum_seeking, heading_control.line_following_control

#     #jordan change
#     full_lpp = local_pathfinder_jmcopy.LocalPathfinder(full_sailor, fake_locations,test_waypoints)

#     full_lpp.run()

#     sailor.rc_run()


# def data_collection(boat: boat.Boat):
#     """
#     Test 7: Collect data for Gaussian MPC
#     https://www.mdpi.com/2077-1312/9/12/1420
#     """
#     num_iterations = 500

#     for i in range(num_iterations):
#         random_angle = np.random.uniform(low = boat.min_rudder_angle, high = boat.max_rudder_angle)
#         boat.send_rudder_angle(random_angle)
#         print(f"{random_angle = }")
#         time.sleep(2)


# if __name__ == "__main__":

#     #Following code if for pond test Feb 10

#     boat_width = 6
#     boat_length = 6
#     boat_mass = 62
#     cargo_mass = 0

#     #Wingsail size properties in meters
#     wingsail_chord = 0.381
#     wingsail_span = 2.02563984

#     #Flap size properties in meters
#     flap_chord = 0.17778984
#     flap_span = 0.5586984

#     #Rudder size properties in meters
#     rudder_plan_area = 0.1
#     rudder_span = 1 / 3

#     #Moments of inertia in kg * m^2
#     inertia_xx = 623.68
#     inertia_yy = 623.92
#     inertia_zz = 64.58

#     test_boat = boat.Boat(boat_width, 
#                         boat_length, 
#                         boat_mass,
#                         cargo_mass,
#                         wingsail_chord,
#                         wingsail_span,
#                         flap_chord,
#                         flap_span,
#                         rudder_plan_area,
#                         rudder_plan_area,
#                         inertia_xx,
#                         inertia_yy,
#                         inertia_zz)

#     #TODO Change if necessary: 
#     desired_heading_angle = 330

#     #TODO: option (1) Global waypoints assumes NSEW wind, pick as appropriate

#     delta_l = 0.0007 #adds about 77 m both directions, 109 m hypotenuse
#     #current_lat = simple_sailor.boat.gps.lat
#     #current_lon = simple_sailor.boat.gps.lon
#     #jordan and ^ move this into function
#     waypoint_start = [29.154156, -94.96709654545455]
#     waypoint_second = [waypoint_start[0]-delta_l,waypoint_start[0]+delta_l] #-x,+y
#     waypoint_third = [waypoint_second[0]+delta_l,waypoint_second[0]+delta_l] # +x,+y
#     waypoiny_fourth = [waypoint_third[0]+delta_l,waypoint_third[0]-delta_l] #+x,-y
#     global_waypoints = [waypoint_start,waypoint_second,waypoint_third,waypoiny_fourth,waypoint_start]

#     #jordan
#     def generate_fake_locations(waypoints, num_per_waypoint=2, max_offset=0.0000005):
#         fake_waypoints = []
#         for lat, lon in waypoints:
#             for _ in range(num_per_waypoint):
#                 lat_offset = np.random.uniform(-max_offset, max_offset)
#                 lon_offset = np.random.uniform(-max_offset, max_offset)
#                 fake_lat = lat + lat_offset
#                 fake_lon = lon + lon_offset
#                 fake_waypoints.append([fake_lat, fake_lon])
#         return fake_waypoints
#     fake_locations = generate_fake_locations(global_waypoints)
#     #print(fake_locations)

#     #TODO: option (2) Global waypoints assumes diagonal wind, change as appropriate
#     # delta_l = 0.0007 #adds about 77 m both directions, 109 m hypotenuse
#     # current_lat = simple_sailor.boat.gps.lat
#     # current_lon = simple_sailor.boat.gps.lon
#     # waypoint_start = [current_lat,current_lon]
#     # waypoint_second = [waypoint_start[0],waypoint_start[0]+delta_l] #+y
#     # waypoint_third = [waypoint_second[0]+delta_l,waypoint_second[0]] # +x
#     # waypoiny_fourth = [waypoint_third[0],waypoint_third[0]-delta_l] #-y
#     # global_waypoints = [waypoint_start,waypoint_second,waypoint_third,waypoiny_fourth,waypoint_start]

#     heading_control = heading_control.HeadingControl(test_boat)
#     speed_control = speed_control.SpeedControl(test_boat)

#     # Create threads for reading/logging data and sending periodic commands
#     read_thread = threading.Thread(target = test_boat.read_and_log_data)
#     #send_thread = threading.Thread(target = rc_check(test_boat, heading_control, speed_control))
#     #send_thread = threading.Thread(target = find_no_go(test_boat, heading_control, speed_control))
#     #send_thread = threading.Thread(target = polar_diagram(test_boat, heading_control, speed_control))
#     #send_thread = threading.Thread(target = simple_speed_test(test_boat, heading_control, speed_control))
#     #send_thread = threading.Thread(target = simple_heading_test(test_boat, heading_control, speed_control, desired_heading_angle))
#     #send_thread = threading.Thread(target=simple_lpp_test(test_boat, heading_control, speed_control))
#     send_thread = threading.Thread(target=full_lpp_test(test_boat, heading_control, speed_control, global_waypoints,fake_locations))
#     #send_thread = threading.Thread(target = data_collection(test_boat))

#     # Start the threads
#     read_thread.start()
#     send_thread.start()

#     # Wait for the threads to finish (you may choose to handle this differently)
#     read_thread.join()
#     send_thread.join()


