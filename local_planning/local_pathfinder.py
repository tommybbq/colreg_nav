from math import atan, atan2, cos, sin, sqrt, degrees, radians
from helper_functions import magnitude, haversine
import random

from sailor import sailor
from sailor import heading_control
from sailor import speed_control
from boat import boat
from lib import sqllogger

import numpy as np
import time
import threading

class LocalPathfinder:
    def __init__(self, sailor: sailor.Sailor, waypoints: list) -> None:
       
        self.sailor = sailor
        self.set_waypoints(waypoints)                               #Allow for update waypoints from global

        self.current_lat: float                                     #Boat's current latitude 
        self.current_lon: float                                     #Boat's current longitude
        self.along_track_error: float = 0.0                         #Along track error
        self.cross_track_error: float = 0.0                         #Cross track error   
        self.distance_to_waypoint = float('inf')  
        self.distance_between_waypoints = float('inf')

        self.azimuth_angle: float = 0                               #The azimuth angle between current two waypoints
        self.rotation_matrix_trans = np.zeros((2,2))                #The rotation matrix
        self.desired_heading_angle: float = 0.0

        self.time_last_call: float = 0.0                            #Last time local path planning function was called
        self._update_errors()
        # self.sql_sailor = sqllogger.SQLLogger("/home/antonio/code/guide-nav/databases/LakeTestDatabase.db")
        # self.lpp_log_data()
        # self.sql_sailor = sqllogger.SQLLogger("/home/antonio/guide-nav/databases/LakeTestDatabase.db")  


    def set_waypoints(self, waypoints: list):
        """
        If passed new waypoints from global pathfinder updates the relevant information
        """
        self.waypoints = waypoints
        self.waypoint_index: int = 0
        self.current_waypoint = self.waypoints[self.waypoint_index]
        self.next_waypoint = self.waypoints[self.waypoint_index + 1]
        self._update_positional()
        self.distance_to_next_waypoint = haversine(self.current_location, self.next_waypoint)
        self.distance_between_waypoints = haversine(self.current_waypoint, self.next_waypoint)
        self._update_azimuth_values()
        # self.sql_sailor.log_lpp_data(self.current_waypoint[0], self.current_waypoint[1], self.distance_to_waypoint, self.boat.gps.lat, self.boat.gps.lon, self.desired_heading)


    def _update_waypoints(self):
        """
        Update information using current waypoint and next waypoint
        """
        #Increase the waypoint index when reached
        self.waypoint_index += 1                        
        self.current_waypoint = self.waypoints[self.waypoint_index]
        self.next_waypoint = self.waypoints[self.waypoint_index + 1]
        self.distance_to_next_waypoint = haversine(self.current_location, self.next_waypoint)
        self.distance_between_waypoints = haversine(self.current_waypoint, self.next_waypoint)
        self._update_azimuth_values()
        # self.sql_sailor.log_lpp_data(self.current_waypoint[0], self.current_waypoint[1], self.distance_to_waypoint, self.boat.gps.lat, self.boat.gps.lon, self.desired_heading)


    def _update_azimuth_values(self):
        """
        Update information requiring azimuth angle
        """
        self.azimuth_angle = atan2(self.next_waypoint[1] - self.current_waypoint[1], self.next_waypoint[0] - self.current_waypoint[0])
        rotation_matrix = np.array([[cos(self.azimuth_angle), - sin(self.azimuth_angle)], [sin(self.azimuth_angle), cos(self.azimuth_angle)]])
        self.rotation_matrix_transpose = np.transpose(rotation_matrix)


    def _update_positional(self):
        """
        Update information based on boat position
        """
        self.current_lat = self.sailor.boat.gps.lat
        self.current_lon = self.sailor.boat.gps.lon
        self.current_location = [self.current_lat, self.current_lon] 
        self.distance_to_next_waypoint = haversine(self.current_location, self.next_waypoint)


    def _update_errors(self):
        """
        Update error based on location and waypoints
        """
        current_lat_difference = haversine([self.current_waypoint[0],0], [self.current_location[0], 0])
        #Difference between current point and waypoint in longitude
        current_lon_difference = haversine([0, self.current_waypoint[1]],[0, self.current_location[1]])

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
        
        #Update positional information
        self._update_positional()

        #Update errors
        self._update_errors()

        parameter_estimate_derivative = adaptation_gain * (lookahead_dist / magnitude([lookahead_dist, self.cross_track_error])) * self.cross_track_error

        #If crab angle is changing significantly updats parameter estimate
        if abs(parameter_estimate_derivative) >= 2.0: 
            parameter_estimate += parameter_estimate_derivative * time_step
        
        desired_heading_angle = self.azimuth_angle - parameter_estimate - atan(self.cross_track_error / lookahead_dist)
        self.desired_heading_angle = (360 - degrees(desired_heading_angle)) % 360

        print(desired_heading_angle)
        #Updates last time function was called
        self.time_last_call = time.time()
        # self.sql_sailor.log_lpp_data(self.current_waypoint[0], self.current_waypoint[1], self.distance_to_waypoint, self.boat.gps.lat, self.boat.gps.lon, self.desired_heading)
        
        #TODO:
        #   Add desired heading check
        return desired_heading_angle  
    
    # def lpp_log_data(self):
    #     self.sql_sailor.log_lpp_data(self.current_waypoint[0], self.current_waypoint[1], self.distance_to_waypoint, self.sailor.boat.gps.lat, self.sailor.boat.gps.lon, self.desired_heading_angle)

    

    # def invalidate_path(self):
    #     self.stop_condition = False


    def run(self):    #     
        global_offtrack_check = 100
        cross_track_check = 25 # change as necessary
        waypoint_radius = 10
        max_heading_update_time = 15 

        if abs(self.cross_track_error) > global_offtrack_check:
            print(f"{self.sailor.boat.gps.lat}, {self.sailor.boat.gps.lon}")
            return False

        #Check if boat is within next_waypoint radius
        if self.distance_to_next_waypoint < waypoint_radius:
            #Check if this is the final waypoint reached
            if self.waypoint_index + 1 == (len(self.waypoints) - 1):
                print(f"Last Waypoint Reached: {self.waypoints[self.waypoint_index+1]}")
                print(f"Current Location: {self.current_location}")
                return True
            #If not the final waypoint
            else:
                self._update_waypoints()
                self.along_track_error: float = 0.0                         #Along track error
                self.cross_track_error: float = 0.0                         #Cross track error   
                self._adaptive_line_of_sight()
                self.sailor.run(self.desired_heading_angle)

        elif abs(self.cross_track_error) > cross_track_check:
            print(f"Boat has veered too far off path: {self.cross_track_error}")
            self._adaptive_line_of_sight()
            self.sailor.run(self.desired_heading_angle)


        elif (time.time() - self.time_last_call) >= max_heading_update_time:
            print(f"Time between updating heading is too large: {time.time() - self.time_last_call}")
            self._adaptive_line_of_sight()
            self.sailor.run(self.desired_heading_angle)


        
        return None

    # def stop_run(self):
    #     self.stop_condition = True



if __name__ == "__main__":

    test_boat = boat.Boat()
    hc_method = heading_control.HeadingControl(test_boat)
    sc_method = speed_control.SpeedControl(test_boat)

    test_sailor = sailor.Sailor(test_boat, sc_method.simple_speed_control, hc_method.line_following_control)
    simple_waypoints = [[test_boat.gps.lat, test_boat.gps.lon],[29.480423, -94.939422],[29.480423, -94.939422],[29.481166907335947, -94.93880206600495],[29.481594794682366, -94.93874222718301]]
    test_lpp = LocalPathfinder(test_sailor, simple_waypoints)
    
    def read_and_log_data(lpp):
        sqll = sqllogger.SQLLogger("/home/sailor/code/guide-nav/databases/HandheldDatabase.db")
        while True:
            test_boat.update()
            test_boat.sql_log(sqll)
            sqll.log_lpp_data(lpp.current_waypoint[0], lpp.current_waypoint[1], lpp.distance_to_waypoint, lpp.sailor.boat.gps.lat, lpp.sailor.boat.gps.lon, lpp.desired_heading_angle)
            # sqllogger.log_lpp_data(test_lpp.current_waypoint[0], test_lpp.current_waypoint[1], test_lpp.distance_to_next_waypoint, test_lpp.current_lat, test_lpp.current_lon, desired_heading)
            time.sleep(0.08)                            

    
    def simple_lpp_test(boat, heading_control_method, speed_control_method):
        #test_sailor = sailor.Sailor(test_boat, sc_method.simple_speed_control, hc_method.line_following_control)
        
        #simple_waypoints = [[29.47939, -94.93916],[29.47858, -94.93964],[29.478374, -94.939770]]
        # initial_waypoints = [[29.47945047631579, -94.93916263928571], 
        #                     [29.47945047631579, -94.93916263928571], 
        #                     [29.47035062368421, -94.94441876785714], 
        #                     [29.46908675526316, -94.94587880357142]]
            
        # return_waypoints = [[29.46908675526316, -94.94587880357142], 
        #                     [29.46908675526316, -94.94558679642857], 
        #                     [29.475658871052634, -94.930402425], 
        #                     [29.47945047631579, -94.93916263928571]]

        #test_lpp = LocalPathfinder(test_sailor, simple_waypoints)
        # test_lpp = LocalPathfinder(test_sailor, initial_waypoints)

        destination_reached = False
        while not destination_reached:
            lpp_status = test_lpp.run()

            if lpp_status is not None:
                if not lpp_status:
                    #TODO:
                    #   How do we handle barriers/obstacles
                    #test_lpp.update_global(destination, [[29.561081, -95.061381],[29.558669, -95.060380],[29.555118, -95.059186]])
                    #full_lpp.set_waypoints(full_gpp.waypoints)
                    print(f"{test_lpp.cross_track_error}")
                    print(f"{boat.gps.lat},{boat.gps.lon}")
                    print("Too far off course")
                    pass
                else:
                    break
        
        # test_lpp.set_waypoints(return_waypoints)
        # destination_reached = False
        # while not destination_reached:
        #     lpp_status = test_lpp.run()

        #     if lpp_status is not None:
        #         if not lpp_status:
        #             #TODO:
        #             #   How do we handle barriers/obstacles
        #             #test_lpp.update_global(destination, [[29.561081, -95.061381],[29.558669, -95.060380],[29.555118, -95.059186]])
        #             #full_lpp.set_waypoints(full_gpp.waypoints)
        #             print("Too far off course")
        #             pass
        #         else:
        #             break
    

    # Create threads for reading/logging data and sending periodic commands
    read_thread = threading.Thread(target = read_and_log_data,args=(test_lpp,), daemon = True)
    send_thread = threading.Thread(target = simple_lpp_test, args=(test_boat, hc_method, sc_method), daemon = True)
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