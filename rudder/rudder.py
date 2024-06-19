from math import pi, atan2, sqrt, radians, asin, atan, sin, cos, degrees
from helper_functions import cross_product, dot_product, magnitude

WATER_DENSITY = 1020 #Density of salt water kg/m^3 (not changing)

class Rudder:

    def __init__(self, plan_area: float, span: float) -> None:
        self.plan_area = plan_area
        self.span = span
        self.lift_coeff: float = 0.5                    #Lift coefficient is function of angle of attack
        self.drag_coeff: float = 0.06                   #Drag coefficient is function of angle of attack
        self.induced_drag_coeff: float = 0
        self.pos_x_rudder = -1                          #x position of rudder in b frame, - 1 m as place holder
        self.pos_y_rudder = 1                           #y position of rudder in b frame, 1 m as place holder
        self.pos_z_rudder = 1                           #z position of rudder in b frame, 1 m as place holder
        self.rudder_angle: float = 0                    #Angle of rudder in degrees
        self.angle_of_attack: float = 0                 #Angle of attack of rudder
        self.apparent_rudder_speed: float = 0                    #Speed of rudder
        self.apparent_rudder_angle: float = 0
        self.coefficient_dataset = [0,0]
        self.rudder_force: list = [0,0]                 #Rudder force magnitude and angle
    

    def _calculate_angle_of_attack(self, boat_speed_x, boat_speed_y, boat_yaw_angular_velocity, boat_roll_angular_velocity):
        #apparent angles with respect to boat speed

        apparent_rudder_speed_x = -boat_speed_x + boat_yaw_angular_velocity * self.pos_y_rudder
        apparent_rudder_speed_y = -boat_speed_y + boat_yaw_angular_velocity * self.pos_x_rudder + boat_roll_angular_velocity * self.pos_z_rudder

        self.apparent_rudder_speed = magnitude([apparent_rudder_speed_x, apparent_rudder_speed_y])
        self.apparent_rudder_angle = degrees(atan2(apparent_rudder_speed_y, -apparent_rudder_speed_x))
        self.angle_of_attack = self.apparent_rudder_angle - self.rudder_angle


    def _calculate_lift(self) -> list:
        lift_magnitude = 0.5 * WATER_DENSITY * self.plan_area * (self.apparent_rudder_speed ** 2) * self.lift_coeff

        if self.angle_of_attack > 0:
            lift_angle = (self.apparent_rudder_angle - 90) % 360
        elif self.angle_of_attack < 0:
            lift_angle = (self.apparent_rudder_angle + 90) % 360
        else:
            #CHECK
            lift_magnitude = 0
            lift_angle = 0

        return [lift_magnitude, lift_angle]

            
    def _calculate_drag(self) -> list:
        drag_magnitude = 0.5 * WATER_DENSITY * self.plan_area * (self.apparent_rudder_speed ** 2) * (self.drag_coeff + self.induced_drag_coeff)
        #Drag angle is colinear with apparent wind
        drag_angle = self.apparent_rudder_angle

        #Drag force vector
        drag_force = [drag_magnitude,drag_angle]
        return drag_force
    
    def _update_coefficients(self):
        self.induced_drag_coeff = pow(self.lift_coeff, 2) * self.plan_area / (pi * pow(self.span, 2))

    def _calculate_rudder_force(self, trueWind_x, trueWind_y, app_wind_X, app_wind_Y) -> list:
        #self.update_coefficients(angle_of_attack)

        lift = self._calculate_lift()
        drag = self._calculate_drag()

        lift_magnitude = lift[0]
        lift_angle = radians(lift[1])                                       #Converts lift angle to radians
        drag_magnitude = drag[0]
        drag_angle = radians(drag[1])                                       #Converts drag angle to radians
        boat_vel_angle_1 = dot_product([trueWind_x,trueWind_y],[app_wind_X, app_wind_Y]) / (magnitude([trueWind_x,trueWind_y]) * magnitude([app_wind_X,app_wind_Y]))
        boat_vel_angle = asin(boat_vel_angle_1) # angle between true wind and app wind, fig 4 setiawan

        #rudder force vector, eqs (7) and (8) from Setiawan
        rudder_force = [lift_magnitude * sin(boat_vel_angle)-drag_magnitude * cos(boat_vel_angle),\
                      -lift_magnitude * cos(boat_vel_angle)-drag_magnitude * sin(boat_vel_angle)]
        rudder_force_magnitude = sqrt(rudder_force[0] ** 2 + rudder_force[1] ** 2)
        rudder_force_angle = atan(rudder_force[1] / rudder_force[0])
        
        rudder_force = [rudder_force_magnitude, rudder_force_angle]
        return rudder_force

class Port(Rudder):
    """
    Creates a Class for Port Rudder
    - Subclass of Rudder
    """

    def update(self, received_data_dict: dict):
        """
        Updates all port rudder data
        
        received_data_dict: dictionary containing all sensor data
        """

        temp_wind_speed = received_data_dict['wind_speed']
        temp_wind_angle = received_data_dict['wind_angle']

        temp_port_angle = received_data_dict['port']


        if temp_wind_speed != None:
            app_wind_magnitude = temp_wind_speed
        
        if temp_wind_angle != None:
            app_wind_angle = temp_wind_angle

        if temp_port_angle != None:
            self.port_angle = temp_port_angle

        # self.calculate_angle_of_attack(app_wind_angle)
        self._update_coefficients()
        # self.rudder_force = self._calculate_sail_force(speed_x, speed_y, yaw_velocity, roll_velocity, trueWind_x, trueWind_y, app_wind_X, app_wind_Y)


class Starboard(Rudder):
    """
    Creates a Class for Starboard Rudder
    - Subclass of Rudder
    """
    def __init__(self) -> None:
        """Creates a Sailor"""
        super().__init__()
        self.starboard_angle: float = 0

    def update(self, received_data_dict: dict):
        """
        Updates all starboard rudder data

        received_data_dict: dictionary containing all sensor data
        """
        
        temp_wind_speed = received_data_dict['wind_speed']
        temp_wind_angle = received_data_dict['wind_angle']

        temp_starboard_angle = received_data_dict['starboard']

        if temp_wind_speed != None:
            app_wind_magnitude = temp_wind_speed
        
        if temp_wind_angle != None:
            app_wind_angle = temp_wind_angle

        if temp_starboard_angle != None:
            self.starboard_angle =  temp_starboard_angle
            
        # self.calculate_angle_of_attack(app_wind_angle)
        self._update_coefficients()
        #self.rudder_force = self._calculate_sail_force(app_wind_magnitude,app_wind_angle)


        

    

    



    
        






