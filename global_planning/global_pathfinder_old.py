import subprocess
from client_socket import client_socket
import os
import time
from boat import boat
import global_pathfinder_functions as glbl

class GlobalPathfinder():
    def __init__(self, 
                 boat,
                 host,
                 port,
                 script_path,
                 subprocess_working_dir) -> None:
            # Path for script to call Optimal Sailor
        self.boat = boat
        self.host = host
        self.port = port
        self.script_path = script_path
        self.subprocess_working_dir = subprocess_working_dir


    def launch_subprocess(script_path, subprocess_working_dir):
        subprocess_command = ["python3", script_path]

        # Launch the subprocess with the specified working directory
        process = subprocess.Popen(subprocess_command, cwd=subprocess_working_dir)

        # Wait for 5 seconds to ensure that main.py can access its requirements
        time.sleep(5)

    def initialize_global(self, destination):
        data_dict = {'command': 'INITIALIZATION', 'list1': self.boat.coords, 'list2': destination, 'list3': None}

        c_socket = client_socket.ClientSocket(host, port)
        c_socket.send(data_dict)
        c_socket.receive()

        #Recieve waypoint information
        self.waypoints = c_socket.data['waypoints']
        self.shore = c_socket.data['barriers']
        self.grid_interval = c_socket.data['grid_interval']
        
        c_socket.close()


    def update(self, list1, list2):
        data_dict = {'command': 'UPDATE', 'list1': self.boat.coords, 'list2': list2, 'list3': list3}

        c_socket = client_socket.ClientSocket(host, port)
        c_socket.send(data_dict)
        c_socket.receive()

        self.waypoints = c_socket.data['waypoints']
        
        c_socket.close()

    
    def objected_detected(self, list1, list2, list3):





if __name__ == "__main__":

    #If on local:
    script_path = '/Users/antoniocrivello/Sea++/Optimal_Sailor/Variable_Wind_Astar/main.py'
    subprocess_working_dir = '/Users/antoniocrivello/Sea++/Optimal_Sailor/Variable_Wind_Astar'

    #If on RPi:
    # script_path = '/home/antonio/Optimal_Sailor/Variable_Wind_Astar/main.py'
    # subprocess_working_dir = '/home/antonio/Optimal_Sailor/Variable_Wind_Astar'

    # Launch the subprocess
    launch_subprocess(script_path, subprocess_working_dir)

    # Communicate with the server
    host = 'localhost'
    port = 1234
    command = 'UPDATE'

    list1 = [29.552012, -95.067667]
    list2 = [29.556726, -95.060320]
    # list3 = [[29.561081, -95.061381],[29.558669, -95.060380],[29.555118, -95.059186]]
    list3 = None

    data_dict = {'command': command, 'list1': list1, 'list2': list2, 'list3': list3}

    c_socket = client_socket.ClientSocket(host, port)
    c_socket.send(data_dict)
    c_socket.receive()


    if command == "INITIALIZATION":
        if 'waypoints' in c_socket.data:
            waypoints = c_socket.data['waypoints']
        if 'barriers' in c_socket.data:
            shore = c_socket.data['barriers']
        print(shore)
        print(waypoints)
        
    else:
        waypoints = c_socket.data['waypoints']
        print(waypoints)

    # print(shore[0][0])
    # print(type(shore))
    # nearest = find_nearest_point(list1, shore)
    # print(f"Nearest point to {list1} is {nearest}")





    