from math import atan, atan2, cos, sin, sqrt, degrees, radians
from helper_functions import magnitude, haversine
import random


from sailor import sailor
from sailor import heading_control
from sailor import speed_control
from boat import boat


import numpy as np
import time
import threading

class LocalPathfinder:
    def __init__(self, sailor: sailor.Sailor, waypoints: list) -> None:
       
        self.sailor = sailor

        #Way point information
        self.waypoints = waypoints
        self.waypoint_index: int = 0
    
        self.current_lat: float                                     #Boat's current latitude 
        self.current_lon: float                                     #Boat's current longitude
        self.along_track_error: float = 0.0                         #Along track error
        self.cross_track_error: float = 0.0                         #Cross track error   
        self.waypoint_distance = float('inf')  
        self.distance_to_waypoint = float('inf')

        self.azimuth_angle: float = 0                               #The azimuth angle between current two waypoints
        self.rotation_matrix_trans = np.zeros((2,2))                #The rotation matrix

        self.time_last_call: float = 0.0                            #Last time local path planning function was called

        self.current_lat: float                                     #Boat's current latitude 
        self.current_lon: float                                     #Boat's current longitude

        self.waypoint_flag: bool = True                                 #Flag to check that boat is not within midpoint radius
        self.time_flag: bool = True                                     #Flag to check that update time is not reached
        self.midpoint_flag: bool = True                                 #Flag to check that boat is not at midpoint
        self.cross_track_flag: bool = True  
        self.next_waypoint_flag: bool = True

    def _update_pos(self):
        
        # #Current boat latitude and longitude
        self.current_lat = self.sailor.boat.gps.lat
        self.current_lon = self.sailor.boat.gps.lon

        self.current_location = [self.current_lat,self.current_lon]
        current_waypoint = self.waypoints[self.waypoint_index]
        next_waypoint = self.waypoints[self.waypoint_index + 1]
        self.waypoint_distance = haversine(current_waypoint, next_waypoint)
        self.distance_to_waypoint = haversine(self.current_location, next_waypoint)

    def _update_azimuth_values(self, current_waypoint: list, next_waypoint: list):
        self.azimuth_angle = atan2(next_waypoint[1] - current_waypoint[1], next_waypoint[0] - current_waypoint[0])
        
        rotation_matrix = np.array([[cos(self.azimuth_angle), - sin(self.azimuth_angle)], [sin(self.azimuth_angle), cos(self.azimuth_angle)]])
        self.rotation_matrix_transpose = np.transpose(rotation_matrix)

    def _update_errors(self):
        current_waypoint = self.waypoints[self.waypoint_index]
        current_lat_difference = haversine([current_waypoint[0],0], [self.current_location[0], 0])
        #Difference between current point and waypoint in longitude
        current_lon_difference = haversine([0, current_waypoint[1]],[0, self.current_location[1]])

        position_difference_vector = np.array([[current_lat_difference],[current_lon_difference]])
        position_error_vector = np.dot(self.rotation_matrix_transpose, position_difference_vector)
        
        self.along_track_error = position_error_vector[0][0]
        self.cross_track_error = position_error_vector[1][0]

    def _adaptive_line_of_sight(self):
        """
        Adaptive Line of Sight Algorithm:
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10087026

        """
        #Coefficients of significance
        waypoint_radius = 10                                        #Waypoint radius in meters
        lookahead_dist = 30                                       #Lookahead distance in meters
        adaptation_gain = 0.02                                      #Adaptation gain
        parameter_estimate: float = 0.0                             #Parameter estimate

        time_current_call: float = time.time()

        #Difference in time between last function call and current function call
        time_step = time_current_call - self.time_last_call

        #TODO: Figure out if north-east coordinate is used
        #TODO: Determine desired lookahead distance and waypoint radius
        #TODO: Need to figure out crab angle
        #TODO: Need to figure out units
        #TODO: add flag for obstacle avoidance

        
        self._update_pos()

        #Next waypoint from waypoint list
        next_waypoint = self.waypoints[self.waypoint_index + 1]
        current_waypoint = self.waypoints[self.waypoint_index]

        #update azimuth angle only if at new waypoint
        if not self.next_waypoint_flag:
            self._update_azimuth_values(current_waypoint, next_waypoint)

        #get cross track/along track errors
        self._update_errors()

        parameter_estimate_derivative = adaptation_gain * (lookahead_dist / magnitude([lookahead_dist, self.cross_track_error])) * self.cross_track_error

        #If crab angle is changing significantly updats parameter estimate
        if abs(parameter_estimate_derivative) >= 2.0: 
            parameter_estimate += parameter_estimate_derivative * time_step
        
        desired_heading_angle = self.azimuth_angle - parameter_estimate - atan(self.cross_track_error / lookahead_dist)
        desired_heading_angle = (360 - degrees(desired_heading_angle)) % 360

        #Updates last time function was called
        self.time_last_call = time.time()
        
        return desired_heading_angle  

    def _run_flag_check(self):
        cross_track_check = 25 # change as necessary
        waypoint_radius = 10
        max_heading_update_time = 180 
        
        if self.distance_to_waypoint < waypoint_radius:
            
            if self.waypoint_index + 1 == len(self.waypoints)-1:
                self.waypoint_flag = False
                self.next_waypoint_flag = False
                #print('last waypoint reached',self.current_location_index)
                return False
            print('Boat has reached next waypoint ============================',self.waypoint_index)
            self.waypoint_index += 1
            self.along_track_error: float = 0.0                         #Along track error
            self.cross_track_error: float = 0.0                         #Cross track error   
            self.waypoint_distance = 0  
            self.distance_to_waypoint = float('inf')
            
            self.next_waypoint_flag = False
        elif abs(self.cross_track_error) > cross_track_check: # if boat is too far off path
            print('Boat has veered too far off path ============================')
            self.cross_track_flag = False
        elif abs(self.along_track_error) >= self.waypoint_distance/2: #if boat is more than halfway
            print('Boat has passed midpoint of current waypoints ============================')
            self.midpoint_flag = False
        current_time = time.time()

        if (current_time - self.time_last_call) >= max_heading_update_time:
            print('Time between updating heading is too large ============================')
            self.time_flag = False

        return True

        
    def _flag_reset(self):
        self.waypoint_flag: bool = True                                 #Flag to check that boat is not within midpoint radius
        self.time_flag: bool = True                                     #Flag to check that update time is not reached
        self.midpoint_flag: bool = True                                 #Flag to check that boat is not at midpoint
        self.cross_track_flag: bool = True  
        self.next_waypoint_flag: bool = True
        self.run_flag = True
        self.send_flag = True


    def run(self):
        self._flag_reset()
        self._update_azimuth_values(self.waypoints[0], self.waypoints[1]) #get inital azimuth and rmat
        desired_heading_angle = self._adaptive_line_of_sight()
        main_flag = self._run_flag_check()

        while main_flag:
            
            self.sailor.run(desired_heading_angle)
            #TODO: add appropriate sleep
            time.sleep(5) #let it cook

            #update position and errors
            self._update_pos() 
            self._update_errors()

            #update flags
            main_flag = self._run_flag_check()
            if main_flag == False:  
                break

            if not self.cross_track_flag or not self.midpoint_flag or not self.next_waypoint_flag or self.time_flag:
                self.send_flag = True
                
            else:
                self.send_flag = False
                
            if self.send_flag:
                desired_heading_angle = self._adaptive_line_of_sight()
                self.time_last_call = time.time() 
                print(f"{desired_heading_angle = }") 
            self._flag_reset() 
            

        if not main_flag:
            print("Boat has reached final waypoint! ============================ ")
        


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

    speed_control = speed_control.SpeedControl(test_boat)
    heading_control = heading_control.HeadingControl(test_boat)
    test_sailor = sailor.Sailor(test_boat, speed_control.extremum_seeking, heading_control.line_following_control)
    
    #Waypoints from optimal sailor
    test_waypoints = [
    [29.13233781818182, -94.98164200000001],
    [29.13233781818182, -94.98164200000001],
    [29.161428727272728, -94.95982381818182],
    [29.190519636363636, -94.93800563636364],
    [29.175974181818184, -94.93800563636364],
    [29.168701454545456, -94.93800563636364],
    [29.154156, -94.93800563636364],
    [29.117792363636365, -94.93800563636364],
    [29.146883272727273, -94.91618745454547],
    [29.175974181818184, -94.89436927272727],
    [29.20506509090909, -94.8725510909091],
    [29.234156000000002, -94.85073290909091],
    [29.255974181818182, -94.83618745454545],
    [29.248701454545454, -94.83618745454545],
    [29.21233781818182, -94.83618745454545],
    [29.190519636363636, -94.83618745454545],
    [29.168701454545456, -94.83618745454545],
    [29.197792363636363, -94.81436927272728],
    [29.226883272727274, -94.7925510909091],
    [29.255974181818182, -94.77073290909091],
    [29.285065090909093, -94.74891472727273],
    [29.306883272727273, -94.73436927272728],
    [29.328701454545456, -94.71982381818182],
    [29.335974181818184, -94.71982381818182],
    [29.350519636363636, -94.72709654545454],
    [29.357792363636364, -94.75618745454545],
    [29.36506509090909, -94.78527836363637],
    [29.379610545454547, -94.80709654545454],
    [29.408701454545454, -94.82891472727273],
    [29.437792363636365, -94.85073290909091],
    [29.466883272727273, -94.8725510909091],
    [29.488701454545456, -94.88709654545455],
    [29.510519636363636, -94.90164200000001],
    [29.52506509090909, -94.90891472727273],
    [29.554156, -94.92346018181819],
    [29.58324690909091, -94.93800563636364],
    [29.60506509090909, -94.94527836363636],
    [29.626883272727273, -94.9525510909091],
    [29.648701454545453, -94.95982381818182],
    [29.670519636363636, -94.96709654545455]
    
]

    test_local_pathfinder = LocalPathfinder(test_sailor, test_waypoints)
    test_local_pathfinder.run()