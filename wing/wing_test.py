from wing import wing
from helper_functions import find_nearest_key
from lib import config_loader, sailor_serial
import time

tolerance = 1e-3
epsilon = 1

# serial_cfg = config_loader.ConfigLoader().load_file("config/serial.toml")
# serial = sailor_serial.SailorSerial(serial_cfg)


#Wingsail size properties in meters
wingsail_chord = 0.381
wingsail_span = 2.02563984

#Flap size properties in meters
flap_chord = 0.17778984
flap_span = 0.5586984


if __name__ == "__main__":

    wingsail = wing.Wing(wingsail_chord,wingsail_span)
    flap = wing.Wing(flap_chord,flap_span)
    
    wingsail._update_coefficients()
    wingsail.__repr__()

    # wingsail.update([30])
    # wingsail.calculate_angle_of_attack([15])
    # wingsail.__str__()


    # serial.send_sail_tail()
    





















# #Lever arms
# wingsail_lever_arm = 0
# flap_lever_arm = 1 #NEED TO GET ACTUAL MEASUREMENT

# #Angles off attack are both initially 0
# wingsail_angle_of_attack = 0
# flap_angle_of_attack = 0

# coefficient_dataset = [( -0.0082 , 0.0192 ),
#                         ( 0.2034 , 0.0343 ),( 0.3634 , 0.0392 ),( 0.4735 , 0.0529 ),( 0.5490 , 0.0633 ),\
#                         ( 0.6283 , 0.0754 ),( 0.7007 , 0.0779 ),( 0.7143 , 0.1071 ),( 0.2726 , 0.1651 ),\
#                         ( 0.2795 , 0.1754 ),( 0.2816 , 0.1858 ),( 0.3038 , 0.2104 ),( 0.3417 , 0.2350 ),\
#                         ( 0.3665 , 0.2608 ),( 0.3935 , 0.2906 ),( 0.4252 , 0.3232 ),( 0.4585 , 0.3612 ),\
#                         ( 0.4816 , 0.4025 ),( 0.5022 , 0.4453 ),( 0.5228 , 0.4961 ),( 0.5591 , 0.5491 ),\
#                         ( 0.5851 , 0.6050 ),( 0.6159 , 0.6504 ),( 0.6431 , 0.7234 ),( 0.6476 , 0.7866 ),\
#                         ( 0.6327 , 0.8374 ),( 0.3723 , 0.5214 ),( 0.3619 , 0.5406 ),( 0.3544 , 0.5606 ),\
#                         ( 0.3431 , 0.5806 ),( 0.3267 , 0.6006 ),( 0.3237 , 0.6206 ),( 0.3099 , 0.6406 ),\
#                         ( 0.2985 , 0.6606 ),( 0.2782 , 0.6806 ),( 0.2665 , 0.6950 ),( 0.2569 , 0.7171 ),\
#                         ( 0.2391 , 0.7306 ),( 0.2209 , 0.7416 ),( 0.1974 , 0.7546 ),( 0.1817 , 0.7706 ),\
#                         ( 0.1597 , 0.7850 ),( 0.1392 , 0.8000 ),( 0.1091 , 0.8061 ),( 0.0892 , 0.8191 ),\
#                         ( 0.0697 , 0.8251 ),( 0.0551 , 0.8261 ),( 0.0263 , 0.8229 ),( 0.0012 , 0.8164 ),\
#                         ( -0.0250 , 0.8160 ),( -0.0477 , 0.8043 ),( -0.0688 , 0.7952 ),( -0.0894 , 0.7884 ),\
#                         ( -0.1154 , 0.7761 ),( -0.1397 , 0.7684 ),( -0.1596 , 0.7540 ),( -0.1863 , 0.7324 ),\
#                         ( -0.2116 , 0.7254 ),( -0.2223 , 0.7022 ),( -0.2432 , 0.6857 ),( -0.2596 , 0.6592 ),\
#                         ( -0.2739 , 0.6351 ),( -0.2803 , 0.6186 ),( -0.3084 , 0.5965 ),( -0.3165 , 0.5731 ),\
#                         ( -0.3314 , 0.5535 ),( -0.3531 , 0.5464 ),( -0.5996 , 0.8372 ),( -0.6151 , 0.8132 ),\
#                         ( -0.6363 , 0.7709 ),( -0.6399 , 0.7314 ),( -0.6420 , 0.6723 ),( -0.6263 , 0.6226 ),\
#                         ( -0.6131 , 0.5674 ),( -0.5838 , 0.5136 ),( -0.5318 , 0.4496 ),( -0.4969 , 0.3894 ),\
#                         ( -0.4464 , 0.3234 ),( -0.4155 , 0.2798 ),( -0.3879 , 0.2451 ),( -0.3702 , 0.2139 ),\
#                         ( -0.3839 , 0.1958 ),( -0.4091 , 0.1840 ),( -0.4683 , 0.1624 ),( -0.5516 , 0.1278 ),\
#                         ( -0.5721 , 0.0858 ),( -0.4982 , 0.0613 ),( -0.3897 , 0.0519 ),( -0.2650 , 0.0533 ),\
#                         ( -0.1621 , 0.0565 ),( -0.0829 , 0.0592 ),( -0.0269 , 0.0544 ),]


# wingsail = wing.Wing(wingsail_chord, wingsail_span, wingsail_lever_arm, coefficient_dataset)
# flap = wing.Wing(flap_chord, flap_span, flap_lever_arm, coefficient_dataset)


# lift_force = wingsail.lift(0,10,10)
# print(lift_force)

# #https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9239150
# #Equation 1:
# wingsail_angle_of_attack = flap_angle_of_attack * 3.21

# # wingsail_angle_of_attack = 3.296*flap.angle_of_attack




# if __name__ == "__main__":
#     #will boat class have mast encoder angle
