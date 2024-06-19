from boat import boat

import threading 
import time
import struct

def sensor_check(boat: boat.Boat):

    num_iterations = 1000

    sensor_data = {}

    for i in range(num_iterations):

        serial_status = False
        gps_status = False
        time.sleep(0.01)

        try:
            serial_status, received_data_dict = boat.serial.receive_all()
            #print(received_data_dict)
        except UnicodeDecodeError:
            print("Message incomplete or ill-formed")
        except struct.error as err:
            print("Incomplete packet")

        if serial_status:
            
            temp_euler = received_data_dict['euler']
            temp_magnetometer = received_data_dict['magnetometer']
            temp_acceleration = received_data_dict['acceleration']
            temp_wind_speed = received_data_dict['wind_speed']
            temp_wind_angle = received_data_dict['wind_angle']
            
            temp_wingsail_angle = received_data_dict["wingsail"]

            if temp_euler != None:
                roll_angle, pitch_angle, yaw_angle = temp_euler
                if i in sensor_data:
                    sensor_data[i].extend([roll_angle, pitch_angle, yaw_angle])
                else:
                    sensor_data[i] = [roll_angle, pitch_angle, yaw_angle]

            # if temp_magnetometer != None:
            #     magnetometer = temp_magnetometer
            #     print("==================================================")
            #     if i in sensor_data:
            #         sensor_data[i].extend([magnetometer])
            #     else:
            #         sensor_data[i] = [magnetometer]
                    
            if temp_wind_speed != None:
                app_wind_magnitude = temp_wind_speed
                if i in sensor_data:
                    sensor_data[i].extend([app_wind_magnitude])
                else:
                    sensor_data[i] = [app_wind_magnitude]
            if temp_wind_angle != None:
                app_wind_angle = temp_wind_angle
                if i in sensor_data:
                    sensor_data[i].extend([app_wind_angle])
                else:
                    sensor_data[i] = [app_wind_angle]

            boat.gps.update()
            if i in sensor_data:
                sensor_data[i].extend([boat.gps.lat, boat.gps.lon, boat.gps.speed])
            else:
                sensor_data[i] = [boat.gps.lat, boat.gps.lon, boat.gps.speed]

            if i in sensor_data:
                sensor_data[i].extend([temp_wingsail_angle])
            else:
                sensor_data[i] = [temp_wingsail_angle]
 
            boat.wingsail.update(received_data_dict)
            boat.flap.update(received_data_dict)

        print("Following Values:")
        print("Roll, Pitch, Yaw, App Wind Magnitude, App Wind Angle, Lat, Lon, Speed, Wingsail Angle")
        try:
            print(sensor_data[i])
        except:
            print("No sensor data yet")

    print("Sensor Check Complete")

def actuation_check(boat: boat.Boat):
    for i in range(boat.min_rudder_angle, boat.max_rudder_angle + 5,5):
        """
        Checks that boat is sending angles to rudders
        """
        boat.send_rudder_angle(i)
        time.sleep(2)

    for i in range(boat.min_flap_angle, boat.max_flap_angle + 4, 4):
        """
        Checks that boat is sending angles to linear actuator
        """
        boat.send_flap_angle(i)
        time.sleep(5)

    for i in range(boat.min_rudder_angle, boat.max_rudder_angle,5):
        for j in range(boat.min_flap_angle, boat.max_flap_angle, 2):
            boat.send_flap_angle(j)
            boat.send_rudder_angle(i)
            time.sleep(5)

    print("Actuator Check Complete")

def los_check():
    pass

def full_systems_check(boat: boat.Boat):
    sensor_check(boat)
    #actuation_check(boat)
    los_check()


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
    
    # Create threads for reading/logging data and sending periodic commands
    read_thread = threading.Thread(target = test_boat.read_and_log_data, daemon = True)
    send_thread = threading.Thread(target = full_systems_check, args = (test_boat, ), daemon = True)

    # Start the threads
    read_thread.start()
    #send_thread.start()

    # Wait for the threads to finish (you may choose to handle this differently)
    read_thread.join()
