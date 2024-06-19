import subprocess
from client_socket import client_socket
import os
import time
# from boat import boat
import global_planning.global_pathfinder_functions as glbl
import helper_functions as hf

import timeit

class GlobalPathfinder():
    def __init__(self, 
                 host,
                 port,
                 script_path,
                 subprocess_working_dir) -> None:
            # Path for script to call Optimal Sailor
        self.host = host
        self.port = port
        self.script_path = script_path
        self.subprocess_working_dir = subprocess_working_dir


    def launch_subprocess(self):
        subprocess_command = ["python3", self.script_path]

        # Launch the subprocess with the specified working directory
        process = subprocess.Popen(subprocess_command, cwd=self.subprocess_working_dir)

        # Wait for 5 seconds to ensure that main.py can access its requirements
        time.sleep(5)


    def initialize_global(self, boat_coords, destination):
        self.launch_subprocess()

        self.global_start_location = boat_coords
        self.destination = destination

        data_dict = {'command': 'INITIALIZATION', 'list1': self.global_start_location, 'list2': self.destination, 'list3': None}

        c_socket = client_socket.ClientSocket(self.host, self.port)
        if c_socket.ping_server():
            c_socket.send(data_dict)
            c_socket.receive()

            #Receive waypoint information
            self.waypoints = c_socket.data['waypoints']
            self.shore = c_socket.data['barriers']

            #Receive grid information
            self.min_lat = c_socket.data['min_lat']
            self.min_long = c_socket.data['min_long']
            self.max_lat = c_socket.data['max_lat']
            self.max_long = c_socket.data['max_long']
            self.grid_rows = c_socket.data['grid_rows']
            self.grid_columns = c_socket.data['grid_columns']
            
            c_socket.close()
            print("Socket closed")
            start = timeit.timeit()
            self.nearest_shore_coord = glbl.find_nearest_point(boat_coords, self.shore)
            self.distance_to_shore = hf.haversine(boat_coords, self.nearest_shore_coord)
            end = timeit.timeit()

            print(end-start)

            self.previous_location = boat_coords

        else:
            c_socket.close()
            return None
        

    def update_global(self, boat_coords):
        self.launch_subprocess()

        data_dict = {'command': 'UPDATE', 'list1': boat_coords, 'list2': self.destination, 'list3': self.barriers}

        c_socket = client_socket.ClientSocket(self.host, self.port)
        if c_socket.ping_server():
            c_socket.send(data_dict)
            c_socket.receive()

            self.waypoints = c_socket.data['waypoints']
        
        c_socket.close()

    
    def objected_detected(self, boat_coords, object_headings, object_distances):
        #Check to see if object intersects list of waypoints

        #If object intersects waypoints see if estimate of distance to shore needs to be recalculated

        # if ...

        pass


if __name__ == "__main__":

    #If on local:
    script_path = '/Users/antoniocrivello/Sea++/Optimal_Sailor/Variable_Wind_Astar/main.py'
    subprocess_working_dir = '/Users/antoniocrivello/Sea++/Optimal_Sailor/Variable_Wind_Astar'

    #If on RPi:
    # script_path = '/home/antonio/Optimal_Sailor/Variable_Wind_Astar/main.py'
    # subprocess_working_dir = '/home/antonio/Optimal_Sailor/Variable_Wind_Astar'

    # Communicate with the server
    host = 'localhost'
    port = 1234

    test_gpp = GlobalPathfinder(host, port, script_path, subprocess_working_dir)

    #Start location
    list2 = [29.479328, -94.939238]
    #Destination
    list1 = [29.468872, -94.945753]

    test_gpp.initialize_global(list1, list2)
    print(test_gpp.waypoints)
    print(test_gpp.nearest_shore_coord)
    print(test_gpp.distance_to_shore)

    # list3 = [[29.561081, -95.061381],[29.558669, -95.060380],[29.555118, -95.059186]]
    # list3 = [test_gpp.nearest_shore_coord]
    # list4 = [29.560456, -95.063304]

    # test_gpp.update_global(list4, list2, list3)
    # print(test_gpp.waypoints)

    # list3 = None




    