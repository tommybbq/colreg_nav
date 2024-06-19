#https://repositorio.ufsc.br/bitstream/handle/123456789/200131/PFC%20Miguel%20Budag%20Becker_2019-1.pdf?sequence=1&isAllowed=y

#https://www.ensta-bretagne.fr/jaulin/rapport_pfe_yann_musellec.pdf

#https://automaticaddison.com/extended-kalman-filter-ekf-with-python-code-example/

#x_vector = [x_position, y_position, yaw]

from boat import boat

import numpy as np

from time import time

class KalmanFilter:
    def __init__(self, boat: boat.Boat):
        self.boat = boat
        n = 4
        m = 4

        #State vector = [lat, lon, surge speed, sway speed]
        self.state_vector = np.empty(shape = (n, 1))
        self.state_covariance_matrix = np.empty(shape = (n, n))
        self.state_transition_matrix = np.empty(shape = (m, 1))
        self.state_to_measurement_matrix = np.empty(shape = (m, n))
        self.measurement_covariance_matrix = np.empty(shape = (m, m))
        self.process_noise_covariance_matrix = np.empty(shape = (n, n))
        self.kalman_gain = np.empty(shape = (n, m))


    def intialize_state(self):
        self.x_0 = self.boat.gps.lat
        self.y_0 = self.boat.gps.lon
        sigma_xm_2 = 0
        sigma_xym = 0
        sigma_ym_2 = 0
        self.P_1 = np.array([[sigma_xm_2, sigma_xym, 0, 0],\
                            [sigma_xym, sigma_ym_2, 0, 0],\
                            [0, 0, 0, 0]\
                            [0, 0, 0, 0]])
        
        self.time_tag = time.time()
        self.x_1 = np.array([[self.x_0],[self.y_0],[0],[0]])

    def reinitialize_state(self):
        self.x_1 = self.boat.gps.lat
        self.y_1 = self.boat.gps.lon
        sigma_xm_2 = 0
        sigma_xym = 0
        sigma_ym_2 = 0
        self.P_2 = np.array([[sigma_xm_2, sigma_xym, 0, 0],\
                            [sigma_xym, sigma_ym_2, 0, 0],\
                            [0, 0, 10 ** 4, 0]\
                            [0, 0, 0, 10 ** 4]])

        current_time = time.time()
        time_change = current_time - self.time_tag
        self.time_tag = current_time

        #Approximates velocity using linear interpolation
        self.x_2 = np.array([[self.x_1],[self.y_1],\
                                      [(self.x_1 - self.x_0) / time_change],
                                      [(self.y_1 - self.y_0) / time_change]]) 
        
    
    def predict_system_state(self):
        current_time = time.time()
        time_change = current_time - self.time_tag
        self.A = np.array([[1, 0, time_change, 0],
                           [0, 1, 0, time_change],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.Q = np.array([[10, 0, 0, 0],
                           [0, 10, 0, 0],
                           [0, 0, 25, 0],
                           [0, 0, 0, 25]])
        self.x_p = np.dot(self.A, self.x_2)
        self.P_p = np.matmul(np.matmul(self.A * self.P_2) * self.A.T) + self.Q

        

