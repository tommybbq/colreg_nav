from math import sqrt, atan, pi
from helper_functions import find_nearest_key, wrap_around_clip

AIR_DENSITY = 1.225                                         #Density of air kg/m^3
#https://www.dur.ac.uk/resources/ecs/research/technical_reports/2014_08.pdf
#Closed-jet dataset with Re = 1 x 10^5
COEFFICIENT_DATASET = {0: (0.0409, 0.0203), 2: (0.1988, 0.0293), 4: (0.4422, 0.0276),
                       6: (0.6491, 0.0421), 8: (0.7795, 0.0530), 10: (0.8784, 0.0632),
                       12: (0.9538, 0.0847), 14: (1.0019, 0.1109), 15: (0.3698, 0.2008),
                       16: (0.4280, 0.2317), 18: (0.4655, 0.2608), 20: (0.5521, 0.2828),
                       25: (0.6952, 0.3965), 30: (0.8417, 0.5591), 35: (1.0072, 0.8047),
                       40: (1.1955, 1.0996), 45: (1.3288, 1.3841), 50: (1.3649, 1.6509),
                       55: (1.3543, 1.9554), 60: (1.3022, 2.2291), 65: (1.2088, 2.4911)}


class Wing:
    """
    Holds information for NACA 0018 Airfoils:
    - Apparent Wind Angle provided in degrees
    - Apparent Wind Velocity provided in m/s
    """
    def __init__(self, chord: float, span: float) -> None:
        self.chord = chord                                  #Chord of wing
        self.span = span                                    #Span of wing
        self.area: float = chord * span                     #Lateral area (chord x wingspan) in m^2
        self.lift_coeff: float = 0                          #Lift coefficient
        self.drag_coeff: float = 0                          #Drag coefficient
        self.induced_drag_coeff: float = 0                  #Induced drag coefficient
        self.absolute_angle: float = 0                      #Absolute angle between boat and wing
        self.relative_angle: float = 0                      #Relative angle between wing and reference frame
        self.angle_of_attack: float = 0                     #Angle between wing chord and apparent wind relative to boat
        self.sail_force: list = [0,0]                       #Sail force magnitude and angle
        #TODO: clean up naming for absolute and relative angle
      
        
    def _calculate_angle_of_attack(self, app_wind_angle):
        '''
        app_wind_angle: wind wrt to boat
        '''
        #Heading angle is defined counterclockwise from bow of boat
        self.angle_of_attack = self.absolute_angle - app_wind_angle

    
    def _update_coefficients(self):
        coefficient_index = find_nearest_key(COEFFICIENT_DATASET, self.angle_of_attack)
        self.lift_coeff, self.drag_coeff = COEFFICIENT_DATASET[coefficient_index]
        self.induced_drag_coeff = (self.area * self.lift_coeff ** 2) / (pi * self.span ** 2)


    def _calculate_lift(self, app_wind_magnitude, app_wind_angle) -> list:
        lift_magnitude = 0.5 * AIR_DENSITY * self.area * (app_wind_magnitude ** 2) * self.lift_coeff

        if self.angle_of_attack > 0:
            lift_angle = (app_wind_angle - 90) % 360
        elif self.angle_of_attack < 0:
            lift_angle = (app_wind_angle + 90) % 360
        else:
            #CHECK
            lift_magnitude = 0
            lift_angle = 0
        return [lift_magnitude, lift_angle]
        
    
    def _calculate_drag(self, app_wind_magnitude, app_wind_angle) -> list:
        drag_magnitude = 0.5 * AIR_DENSITY * self.area * (app_wind_magnitude ** 2) * (self.drag_coeff + self.induced_drag_coeff)
        drag_angle = app_wind_angle
        return [drag_magnitude, drag_angle]


    def _calculate_sail_force(self, app_wind_magnitude, app_wind_angle):
        lift = self._calculate_lift(app_wind_magnitude, app_wind_angle)
        drag = self._calculate_drag(app_wind_magnitude, app_wind_angle)

        #IF drag 0, zero division
        if app_wind_magnitude != 0 and app_wind_angle != 0:
            sail_force_magnitude = sqrt(lift[0] ** 2 + drag[0] ** 2)
            sail_force_angle = atan(lift[1] / drag[1])
        else:
            sail_force_magnitude = 0
            sail_force_angle = 0

        sail_force = [sail_force_magnitude, sail_force_angle]
        return sail_force

    
    def calculate_desired_angle_of_attack(self, desired_wing_angle, app_wind_angle):
        desired_angle_of_attack = desired_wing_angle - app_wind_angle
        desired_angle_of_attack = wrap_around_clip(desired_angle_of_attack)
        return desired_angle_of_attack

    
    def __str__(self):
        return f"Lift Coefficient: {self.lift_coeff}, Drag Coefficient: {self.drag_coeff}, Absolute Angle: {self.absolute_angle}, Relative Angle: {self.relative_angle}, Angle of Attack: {self.angle_of_attack}"


class Flap(Wing):
    """
    Creates a Flap Class
    - Subclass of Wing 
    """

    def update(self, received_data_dict: dict):
        """
        Updates all wing data
        
        received_data_dict: dictionary containing all sensor data
        """

        temp_wind_speed = received_data_dict['wind_speed']
        temp_wind_angle = received_data_dict['wind_angle']

        temp_relative_angle = received_data_dict['flap']
        temp_wingsail_angle = received_data_dict['wingsail']

        if temp_wind_speed != None:
            app_wind_magnitude = temp_wind_speed
        
        if temp_wind_angle != None:
            app_wind_angle = temp_wind_angle

        if temp_relative_angle != None:
            self.relative_angle = temp_relative_angle

        if temp_relative_angle != None and temp_wingsail_angle != None:
            self.absolute_angle = temp_relative_angle + temp_wingsail_angle

        self._calculate_angle_of_attack(app_wind_angle)
        self._update_coefficients()
        sail_force = self._calculate_sail_force(app_wind_magnitude,app_wind_angle)


class Wingsail(Wing):
    """
    Creates a Wingsail Class
    - Subclass of Wing
    """

    def update(self, received_data_dict: dict):
        """
        Updates all wing data

        received_data_dict: dictionary containing all sensor data
        """
        
        temp_wind_speed = received_data_dict['wind_speed']
        temp_wind_angle = received_data_dict['wind_angle']

        temp_wingsail_angle = received_data_dict['wingsail']

        if temp_wind_speed != None:
            app_wind_magnitude = temp_wind_speed
        
        if temp_wind_angle != None:
            app_wind_angle = temp_wind_angle

        if temp_wingsail_angle != None:
            self.relative_angle = temp_wingsail_angle
            self.absolute_angle = temp_wingsail_angle
            
        self._calculate_angle_of_attack(app_wind_angle)
        self._update_coefficients()
        sail_force = self._calculate_sail_force(app_wind_magnitude,app_wind_angle)


        

    

    



    
        



