import numpy as np

from boat import boat

import time

def sensor_collect(gps_lat_list: list,
                   gps_lon_list: list,
                   gps_speed_list: list,
                   yaw_list: list,
                   wind_mag_list: list,
                   wind_angle_list: list,
                   boat: boat.Boat):

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

            if temp_euler != None:
                roll_angle, pitch_angle, yaw_angle = temp_euler
                yaw_list.append(yaw_angle)

            # if temp_magnetometer != None:
            #     self.magnetometer = temp_magnetometer
            # if temp_acceleration != None:
            #     self.acceleration = temp_acceleration
            if temp_wind_speed != None:
                app_wind_magnitude = temp_wind_speed
                wind_mag_list.append(app_wind_magnitude)

            if temp_wind_angle != None:
                app_wind_angle = temp_wind_angle
                wind_angle_list.append(app_wind_angle)


            boat.gps.update()


            gps_lat_list.append(boat.gps.lat)
            gps_lon_list.append(boat.gps.lon)
            gps_speed_list.append(boat.gps.speed)

        return gps_lat_list, gps_lon_list, gps_speed_list, wind_mag_list, wind_angle_list, yaw_list
            

            #self.wingsail.update(received_data_dict)
            #self.flap.update(received_data_dict)

def calculate_standard_deviation(boat: boat.Boat):

    gps_lat_list = []
    gps_lon_list = []
    gps_speed_list = []
    wind_mag_list = []
    wind_angle_list = []
    yaw_list = []

    num_iterations = 7500

    for i in range(num_iterations):
        gps_lat_list, gps_lon_list, gps_speed_list, wind_mag_list, wind_angle_list, yaw_list = sensor_collect(
            gps_lat_list,
            gps_lon_list,
            gps_speed_list,
            yaw_list,
            wind_mag_list,
            wind_angle_list,
            boat
        )

    gps_lat_values = np.array(gps_lat_list)
    gps_lon_values = np.array(gps_lon_list)
    gps_speed_values = np.array(gps_speed_list)
    gps_yaw_values = np.array(yaw_list)
    wind_mag_values = np.array(wind_mag_list)
    wind_angle_values = np.array(wind_angle_list)

    gps_lat_std_dev = np.std(gps_lat_values)
    gps_lon_std_dev = np.std(gps_lon_values)
    gps_speed_std_dev = np.std(gps_speed_values)
    gps_yaw_std_dev = np.std(gps_yaw_values)
    wind_mag_std_dev = np.std(wind_mag_values)
    wind_angle_std_dev = np.std(wind_angle_values)

    with open("logs/standard_deviation.txt", 'a') as data:
        data.write(f'{gps_lat_std_dev = }, {gps_lon_std_dev = }, {gps_speed_std_dev = }, {gps_yaw_std_dev = }, {wind_mag_std_dev = }, {wind_angle_std_dev = }')
    

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

    calculate_standard_deviation(test_boat)