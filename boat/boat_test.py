from boat import boat
from wing import wing
from lib.sqllogger import SQLLogger
from web_backend import websocketserver
import time 
from datetime import datetime
import threading


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
                    0, 0, 0)

def extremum_seeking_test():
    time.sleep(0.1)
    for i in range(100):
        test_boat.extremum_seeking()
        time.sleep(5)
        print(test_boat)

# def backup_speed_test():
#     time.sleep(0.1)
#     for i in range(100):
#         test_boat.backup_speed_control()
#         time.sleep(5)
#         print(test_boat)

def flap_test():
    test_boat.send_flap_angle(-40)
    time.sleep(5)

def user_input_test():
    while True:
        user_input = input("Enter sail tail angle: ")
        if user_input == "x":
            break
        user_angle = float(user_input)

        for i in range(10):
            test_boat.send_flap_angle(user_angle)
            time.sleep(5)


# Function to read data from the serial port and log it
def read_and_log_data():
    sqllogger = SQLLogger("/home/samatar/Code/guide-nav/databases/NavigationData.db")
    while True:
        print("a")
        test_boat.update()
        test_boat.sql_log(sqllogger)
        print(test_boat)
        time.sleep(0.08)


def flap_control(desired_wingsail_angle_of_attack):

    #https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9239150&tag=1
    input_flap_angle = desired_wingsail_angle_of_attack / 3.21
    test_boat.send_flap_angle(input_flap_angle)
    print(f"{input_flap_angle = }")

def rc_control():
    ws = websocketserver.WebSocketServer(addr="10.0.0.3")
    if not ws.start():
        print("Something went wrong")

    test_boat.serial.flap_timeout = 0.5

    while True:
        if ws.read_avaliable():
            part, data = ws.read_in_order()
            if part == "rudder":
                print("Rudder:", data)
            elif part == "sail":
                print("Flap:", data)
                test_boat.send_flap_angle(data)
            else:
                print("Bad Data?")


if __name__ == "__main__":



    # Create threads for reading/logging data and sending periodic commands
    read_thread = threading.Thread(target=read_and_log_data)
    send_thread = threading.Thread(target=rc_contro)

    # Start the threads
    read_thread.start()
    send_thread.start()

    # Wait for the threads to finish (you may choose to handle this differently)
    read_thread.join()
    send_thread.join()







# for i in range(100):

#     test_boat.control_flap_angle(330)
#     print(i)
# print(test_boat.wingsail.absolute_angle)
# print(test_boat.flap.relative_angle)

# print(test_boat.wingsail.absolute_angle)
# #test_boat.wingsail.update()
# time.sleep(5)
# print(test_boat.wingsail.absolute_angle)

# for i in (range(10)):
#     test_boat.error_test(30)
#     time.sleep(1)

