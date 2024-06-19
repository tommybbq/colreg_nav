import subprocess
from client_socket import client_socket
import os
import time
from boat import boat
import global_planning.global_pathfinder_functions as glbl
import helper_functions as hf

class GlobalPathfinder():
    def __init__(self, 
                 boat,
                 host,
                 port,
                 script_path,
                 subprocess_working_dir) -> None:
        self.boat = boat
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


    def initialize_global(self, destination):
        self.launch_subprocess()

        # data_dict = {'command': 'INITIALIZATION', 'list1': [self.boat.gps.lat, self.boat.gps.lon], 'list2': destination, 'list3': None}
        data_dict = {'command': 'INITIALIZATION', 'list1': [29.562185, -95.071397], 'list2': destination, 'list3': None}

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
            # self.nearest_shore_coord = glbl.find_nearest_point([self.boat.gps.lat, self.boat.gps.lon], self.shore)
            # self.distance_to_shore = hf.haversine([self.boat.gps.lat, self.boat.gps.lon], self.nearest_shore_coord)

            # self.previous_location = [self.boat.gps.lat, self.boat.gps.lon]

            self.nearest_shore_coord = glbl.find_nearest_point([29.562185, -95.071397], self.shore)
            self.distance_to_shore = hf.haversine([29.562185, -95.071397], self.nearest_shore_coord)

            self.previous_location = [29.562185, -95.071397]

        else:
            c_socket.close()
            return None
        

    def update_global(self, destination, barriers):
        self.launch_subprocess()

        data_dict = {'command': 'UPDATE', 'list1': [29.562185, -95.071397], 'list2': destination, 'list3': barriers}
        # data_dict = {'command': 'UPDATE', 'list1': [self.boat.gps.lat, self.boat.gps.lon], 'list2': destination, 'list3': barriers}

        c_socket = client_socket.ClientSocket(self.host, self.port)
        if c_socket.ping_server():
            c_socket.send(data_dict)
            c_socket.receive()

            self.waypoints = c_socket.data['waypoints']
        
        c_socket.close()

    
    def objected_detected(self, heading, distance):
        #Check to see if object intersects list of waypoints

        #If object intersects waypoints see if estimate of distance to shore needs to be recalculated
        pass


if __name__ == "__main__":

    #If on local:
    #script_path = '/Users/antoniocrivello/Sea++/Optimal_Sailor/Variable_Wind_Astar/main.py'
    #subprocess_working_dir = '/Users/antoniocrivello/Sea++/Optimal_Sailor/Variable_Wind_Astar'

    #If on RPi:
    script_path = '/home/antonio/Optimal_Sailor/Variable_Wind_Astar/main.py'
    subprocess_working_dir = '/home/antonio/Optimal_Sailor/Variable_Wind_Astar'

    # Communicate with the server
    host = 'localhost'
    port = 1336

    test_boat = boat.Boat()

    test_gpp = GlobalPathfinder(test_boat, host, port, script_path, subprocess_working_dir)

    list1 = [29.562185, -95.071397]
    list2 = [29.555021, -95.050772]

    test_gpp.initialize_global(list2)
    print(test_gpp.waypoints)
    print(test_gpp.nearest_shore_coord)
    print(test_gpp.distance_to_shore)

    list3 = [29.560456, -95.063304]
    list4 = [[29.561081, -95.061381],[29.558669, -95.060380],[29.555118, -95.059186]]
    # list3 = [test_gpp.nearest_shore_coord]

    test_gpp.update_global(list3, list4)
    print(test_gpp.waypoints)

    # list3 = None




    