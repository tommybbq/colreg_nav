# from sailor import sailor
# from sailor import heading_control
# from sailor import speed_control
# from boat import boat

from helper_functions import magnitude, haversine
from math import atan, atan2, cos, sin, sqrt, degrees, radians
import numpy as np

import time
import threading

class LocalPathfinder:
    def __init__(self, current_location: list, waypoints: list) -> None:
        #self.sailor = sailor

        #Waypoint information
        self.waypoints = waypoints
        self.waypoint_index: int = 0
        self.current_waypoint = self.waypoints[self.waypoint_index]     #Starting waypoint
        self.next_waypoint = self.waypoints[self.waypoint_index + 1]    #Next waypoint

        self.current_location = current_location
        self.current_lat: float = current_location[0][0]                   #Boat's current latitude 
        self.current_lon: float = current_location[0][1]                   #Boat's current longitude
        
        self.along_track_error: float = 0.0                             #Along track error
        self.cross_track_error: float = 0.0                             #Cross track error   
        self.successive_waypoint_distance = float('inf')                #Distance between two current waypoints
        self.distance_to_waypoint = float('inf')                        #Distance between boat and next waypoint

        self.azimuth_angle: float = 0                                   #The azimuth angle between current two waypoints
        self.rotation_matrix_transpose = np.zeros((2,2))                #The rotation matrix

        self.time_last_call: float = time.time()                        #Last time local path planning function was called

        self.waypoint_flag: bool = True                                 #Flag to check that boat is not within midpoint radius
        self.time_flag: bool = True                                     #Flag to check that update time is not reached
        self.midpoint_flag: bool = True                                 #Flag to check that boat is not at midpoint
        self.cross_track_flag: bool = True                              #Flag to check that boat is within allowable cross track

        self.send_flag: bool = True                                     #Flag to send desired heading angle                                  


    def adaptive_line_of_sight(self):
        """
        Adaptive Line of Sight Algorithm:
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10087026

        """
        #Coefficients of significance
        waypoint_radius = 10                                        #Waypoint radius in meters
        lookahead_dist = 30                                         #Lookahead distance in meters
        adaptation_gain = 0.02                                      #Adaptation gain
        parameter_estimate: float = 0.0                             #Parameter estimate

        #TODO: Determine tune lookahead distance and waypoint radius

        time_current_call: float = time.time()

        #Difference in time between last function call and current function call
        time_step = time_current_call - self.time_last_call

        #Current location list. These values are updated in run before function is called
        current_location = [self.current_lat, self.current_lon]

        #Update azimuth angle only if at new waypoint
        if not self.waypoint_flag:
            self._update_azimuth_values(self.current_waypoint, self.next_waypoint)

        #Get cross track/along track errors
        self._update_errors()

        parameter_estimate_derivative = adaptation_gain * (lookahead_dist / magnitude([lookahead_dist, self.cross_track_error])) * self.cross_track_error

        #If crab angle is changing significantly updats parameter estimate
        if abs(parameter_estimate_derivative) >= 2.0: 
            parameter_estimate += parameter_estimate_derivative * time_step

        parameter_estimate = radians(parameter_estimate)

        desired_heading_angle = self.azimuth_angle - parameter_estimate - atan(self.cross_track_error / lookahead_dist)
        desired_heading_angle = (360 - degrees(desired_heading_angle)) % 360
        
        return desired_heading_angle
    

    def _update_position(self):
        """
        Updates current position of boat and calculates distance to next waypoint
        """
        #Current boat latitude and longitude
        #self.current_lat = self.sailor.boat.gps.lat
        #self.current_lon = self.sailor.boat.gps.lon
        self.distance_to_waypoint = haversine([self.current_lat, self.current_lon], self.next_waypoint)


    def _update_errors(self):
        #Difference between current point and waypoint in latitude
        current_lat_difference = haversine([self.current_waypoint[0],0], [self.current_lat, 0])
        #Difference between current point and waypoint in longitude
        current_lon_difference = haversine([0, self.current_waypoint[1]],[0, self.current_lon])

        position_difference_vector = np.array([[current_lat_difference],[current_lon_difference]])
        position_error_vector = np.dot(self.rotation_matrix_transpose, position_difference_vector)

        self.along_track_error = position_error_vector[0][0]
        self.cross_track_error = position_error_vector[1][0]


    def _update_azimuth_values(self, current_waypoint: list, next_waypoint: list):
        self.azimuth_angle = atan2(next_waypoint[1] - current_waypoint[1], next_waypoint[0] - current_waypoint[0])
        rotation_matrix = np.array([[cos(self.azimuth_angle), - sin(self.azimuth_angle)], [sin(self.azimuth_angle), cos(self.azimuth_angle)]])
        self.rotation_matrix_transpose = np.transpose(rotation_matrix)


    def _update_waypoints(self):
        """
        Changes waypoints and calculates new distance between them
        """
        #Increments the waypoint index by 1
        self.waypoint_index = self.waypoint_index + 1

        #Updates the current and enxt waypoints
        self.current_waypoint = self.waypoints[self.waypoint_index]
        self.next_waypoint = self.waypoints[self.waypoint_index + 1]

        #Calculates the distance between two waypoints
        self.successive_waypoint_distance = haversine([self.current_lat, self.current_lon], self.next_waypoint)


    def _flag_check(self):
        """
        Checks status of all flags
        """
        cross_track_width = 25                                              #Allowable cross track error in meters
        waypoint_radius = 10                                                #Radius around waypoint
        max_heading_update_time = 300                                       #Maximum time between function calls 

        if self.distance_to_waypoint < waypoint_radius:
            #TODO: Confirm this does not cause a problem on other side of waypoint
            
            if self.waypoint_index + 1 == len(self.waypoints)-1:
                self.waypoint_flag = False
                print('Last waypoint reached')
                return False

            self.waypoint_flag = False

        if abs(self.cross_track_error) > cross_track_width:               #Boat is too far off path
            print(f"{self.cross_track_error = }")
            print('Boat has veered too far off path')
            self.cross_track_flag = False

        if abs(self.along_track_error) >= self.distance_to_waypoint / 2:     #Boat is more than halfway
            print('Boat has reached more than halfway')
            print(f"{self.along_track_error = }")
            self.midpoint_flag = False
        
        current_time = time.time()

        if (current_time - self.time_last_call) >= max_heading_update_time:
            print('Time between updating heading is too large')
            print(f"{current_time - self.time_last_call = }")
            self.time_flag = False

        return True


    def run(self):
        #TODO: Would like to call speed control and heading control
        #TODO: Needs to receive flag from sailor
        idx = 1
        run_flag = True

        while run_flag:
            #Update position of boat 
            self._update_position()

            run_flag = self._flag_check()
            if not run_flag:
                break

            if not self.waypoint_flag:
                self._update_waypoints()
                desired_heading_angle = self.adaptive_line_of_sight()
                self.send_flag = True
                self.time_last_call = time.time()                             

            elif not self.midpoint_flag or not self.cross_track_error or not self.time_flag:
                desired_heading_angle = self.adaptive_line_of_sight()
                self.send_flag = True
                self.time_last_call = time.time()                             
            
            else:
                self.send_flag = False

            if self.send_flag:
                print(f"{desired_heading_angle = }")
                idx += 1
                if idx < len(self.current_location):  # Check if idx is within bounds
                    self.current_lat = self.current_location[idx][0]
                    self.current_lon = self.current_location[idx][1]
                else:
                    break
    

if __name__ == "__main__":

    waypoints = [
    [29.154156, -94.96709654545455],
    [29.154156, -94.96709654545455],
    [29.18324690909091, -94.94527836363636],
    [29.154156, -94.94527836363636],
    [29.117792363636365, -94.94527836363636],
    [29.146883272727273, -94.92346018181819],
    [29.175974181818184, -94.90164200000001],
    [29.20506509090909, -94.87982381818182],
    [29.234156000000002, -94.85800563636364],
    [29.197792363636363, -94.85800563636364],
    [29.190519636363636, -94.85800563636364],
    [29.175974181818184, -94.85800563636364],
    [29.146883272727273, -94.85800563636364],
    [29.13233781818182, -94.85800563636364],
    [29.125065090909093, -94.85800563636364],
    [29.117792363636365, -94.85800563636364],
    [29.146883272727273, -94.83618745454545],
    [29.175974181818184, -94.81436927272728],
    [29.20506509090909, -94.7925510909091],
    [29.234156000000002, -94.77073290909091],
    [29.26324690909091, -94.74891472727273],
    [29.292337818181817, -94.73436927272728],
    [29.306883272727273, -94.72709654545454],
    [29.335974181818184, -94.71982381818182],
    [29.350519636363636, -94.72709654545454],
    [29.357792363636364, -94.75618745454545],
    [29.36506509090909, -94.78527836363637],
    [29.37233781818182, -94.79982381818182],
    [29.386883272727275, -94.80709654545454],
    [29.415974181818182, -94.81436927272728],
    [29.44506509090909, -94.821642],
    [29.48142872727273, -94.821642],
    [29.510519636363636, -94.821642],
    [29.539610545454543, -94.821642],
    [29.54688327272727, -94.821642],
    [29.554156, -94.821642],
    [29.561428727272727, -94.821642],
    [29.568701454545455, -94.821642],
    [29.575974181818182, -94.821642],
    [29.58324690909091, -94.821642],
    [29.590519636363634, -94.821642],
    [29.597792363636362, -94.821642],
    [29.60506509090909, -94.821642],
    [29.612337818181818, -94.821642],
    [29.619610545454545, -94.821642],
    [29.626883272727273, -94.821642],
    [29.634156, -94.821642],
    [29.641428727272725, -94.821642],
    [29.648701454545453, -94.821642],
    [29.65597418181818, -94.821642],
    [29.66324690909091, -94.821642],
    [29.670519636363636, -94.821642],
    [29.677792363636364, -94.821642],
    [29.68506509090909, -94.821642],
    [29.692337818181816, -94.821642],
    [29.721428727272727, -94.81436927272728],
    [29.750519636363634, -94.80709654545454]
    ]
    
    def generate_fake_locations(waypoints, num_per_waypoint=5, max_offset=0.00005):
        fake_waypoints = []
        for lat, lon in waypoints:
            for _ in range(num_per_waypoint):
                lat_offset = np.random.uniform(-max_offset, max_offset)
                lon_offset = np.random.uniform(-max_offset, max_offset)
                fake_lat = lat + lat_offset
                fake_lon = lon + lon_offset
                fake_waypoints.append([fake_lat, fake_lon])
        return fake_waypoints
    fake_locations = generate_fake_locations(waypoints)
    print(fake_locations)

    test_lpp = LocalPathfinder(fake_locations, waypoints)
    test_lpp.run()