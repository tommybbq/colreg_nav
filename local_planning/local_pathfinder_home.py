from math import atan, atan2, cos, sin, sqrt, degrees, radians
from helper_functions import magnitude, haversine
import random

#from sailor import sailor
#from sailor import heading_control
#from sailor import speed_control
#from boat import boat


import numpy as np
import time
import threading

class LocalPathfinder:
    def __init__(self, current_location: list,waypoints: list) -> None:
        #self.sailor = sailor

        #Way point information
        self.waypoints = waypoints
        self.waypoint_index: int = 0
    

        self.current_lat: float                                     #Boat's current latitude 
        self.current_lon: float                                     #Boat's current longitude
        self.along_track_error: float = 0.0                         #Along track error
        self.cross_track_error: float = 0.0                         #Cross track error   
        self.waypoint_distance = float('inf')  
        self.distance_to_waypoint = float('inf')

        self.azimuth_angle: float = 0                               #The azimuth angle between current two waypoints
        self.rotation_matrix_trans = np.zeros((2,2))                #The rotation matrix

        self.time_last_call: float = 0.0                            #Last time local path planning function was called

        #TODO: these locations can be removed
        self.current_location_points = current_location
        self.current_location_index: int = 0
        self.current_lat: float = current_location[0][0]                   #Boat's current latitude 
        self.current_lon: float = current_location[0][1]                                     #Boat's current longitude

        self.waypoint_flag: bool = True                                 #Flag to check that boat is not within midpoint radius
        self.time_flag: bool = True                                     #Flag to check that update time is not reached
        self.midpoint_flag: bool = True                                 #Flag to check that boat is not at midpoint
        self.cross_track_flag: bool = True  
        self.next_waypoint_flag: bool = True

    def _update_pos(self):
        #TODO: remove current location vector when time comes
        
        self.current_lat = self.current_location_points[self.current_location_index][0]
        self.current_lon = self.current_location_points[self.current_location_index][1]
        self.current_location = [self.current_lat,self.current_lon]
        current_waypoint = self.waypoints[self.waypoint_index]
        next_waypoint = self.waypoints[self.waypoint_index + 1]
        self.waypoint_distance = haversine(current_waypoint, next_waypoint)
        self.distance_to_waypoint = haversine(self.current_location, next_waypoint)

    def _update_azimuth_values(self, current_waypoint: list, next_waypoint: list):
        self.azimuth_angle = atan2(next_waypoint[1] - current_waypoint[1], next_waypoint[0] - current_waypoint[0])
        
        rotation_matrix = np.array([[cos(self.azimuth_angle), - sin(self.azimuth_angle)], [sin(self.azimuth_angle), cos(self.azimuth_angle)]])
        self.rotation_matrix_transpose = np.transpose(rotation_matrix)

    def _update_errors(self):
        current_waypoint = self.waypoints[self.waypoint_index]
        current_lat_difference = haversine([current_waypoint[0],0], [self.current_location[0], 0])
        #Difference between current point and waypoint in longitude
        current_lon_difference = haversine([0, current_waypoint[1]],[0, self.current_location[1]])

        position_difference_vector = np.array([[current_lat_difference],[current_lon_difference]])
        position_error_vector = np.dot(self.rotation_matrix_transpose, position_difference_vector)
        
        self.along_track_error = position_error_vector[0][0]
        self.cross_track_error = position_error_vector[1][0]

    def _adaptive_line_of_sight(self):
        """
        Adaptive Line of Sight Algorithm:
        https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10087026

        """
        #Coefficients of significance
        waypoint_radius = 10                                        #Waypoint radius in meters
        lookahead_dist = 30                                       #Lookahead distance in meters
        adaptation_gain = 0.02                                      #Adaptation gain
        parameter_estimate: float = 0.0                             #Parameter estimate

        time_current_call: float = time.time()

        #Difference in time between last function call and current function call
        time_step = time_current_call - self.time_last_call

        #TODO: Figure out if north-east coordinate is used
        #TODO: Determine desired lookahead distance and waypoint radius
        #TODO: Need to figure out crab angle
        #TODO: Need to figure out units
        #TODO: add flag for obstacle avoidance

        #Current boat latitude
        #self.boat_current_lat = self.sailor.boat.gps.lat
        #print(f"{self.boat_current_lat = }")
        #Current boat longitude
        #self.boat_current_lon = self.sailor.boat.gps.lon
        #print(f"{self.boat_current_lon = }")
       
        #current_location = [self.boat_current_lat, self.boat_current_lon]
        
        self._update_pos()
        #Next waypoint from waypoint list
        next_waypoint = self.waypoints[self.waypoint_index + 1]
        current_waypoint = self.waypoints[self.waypoint_index]

        #update azimuth angle only if at new waypoint
        if not self.next_waypoint_flag:
            self._update_azimuth_values(current_waypoint, next_waypoint)

        #get cross track/along track errors
        self._update_errors()

        parameter_estimate_derivative = adaptation_gain * (lookahead_dist / magnitude([lookahead_dist, self.cross_track_error])) * self.cross_track_error

        #If crab angle is changing significantly updats parameter estimate
        if abs(parameter_estimate_derivative) >= 2.0: 
            parameter_estimate += parameter_estimate_derivative * time_step
        
        desired_heading_angle = self.azimuth_angle - parameter_estimate - atan(self.cross_track_error / lookahead_dist)
        desired_heading_angle = (360 - degrees(desired_heading_angle)) % 360

        #Updates last time function was called
        self.time_last_call = time.time()
        
        return desired_heading_angle  

    def _run_flag_check(self):
        cross_track_check = 25 # change as necessary
        waypoint_radius = 10
        max_heading_update_time = 180 
        
        if self.distance_to_waypoint < waypoint_radius:
            
            if self.waypoint_index + 1 == len(self.waypoints)-1:
                self.waypoint_flag = False
                self.next_waypoint_flag = False
                #print('last waypoint reached',self.current_location_index)
                return False
            print('Boat has reached next waypoint',self.waypoint_index)
            self.waypoint_index += 1
            self.along_track_error: float = 0.0                         #Along track error
            self.cross_track_error: float = 0.0                         #Cross track error   
            self.waypoint_distance = 0  
            self.distance_to_waypoint = float('inf')
            
            self.next_waypoint_flag = False
        elif abs(self.cross_track_error) > cross_track_check: # if boat is too far off path
            print('Boat has veered too far off path')
            self.cross_track_flag = False
        elif abs(self.along_track_error) >= self.waypoint_distance/2: #if boat is more than halfway
            print('Boat has passed midpoint of current waypoints')
            self.midpoint_flag = False
        current_time = time.time()

        if (current_time - self.time_last_call) >= max_heading_update_time:
            print('Time between updating heading is too large')
            self.time_flag = False

        return True

        
    def _flag_reset(self):
        self.waypoint_flag: bool = True                                 #Flag to check that boat is not within midpoint radius
        self.time_flag: bool = True                                     #Flag to check that update time is not reached
        self.midpoint_flag: bool = True                                 #Flag to check that boat is not at midpoint
        self.cross_track_flag: bool = True  
        self.next_waypoint_flag: bool = True
        self.run_flag = True
        self.send_flag = True

    def run(self):
        
        self._flag_reset()
        self._update_azimuth_values(self.waypoints[0], self.waypoints[1]) #get inital azimuth and rmat
        desired_heading_angle = self._adaptive_line_of_sight()
        main_flag = self._run_flag_check()

        while main_flag:
            
            #self.sailor.run(desired_heading_angle)
            if self.current_location_index < len(self.current_location_points):  # Check if idx is within bounds
                #print('hi')
                self.current_location_index += 1
            time.sleep(0)
            self._update_pos() 
            self._update_errors()
            main_flag = self._run_flag_check()
            if main_flag == False:
                
                break

            if not self.cross_track_flag or not self.midpoint_flag or not self.next_waypoint_flag or self.time_flag:
                self.send_flag = True
                
            else:
                self.send_flag = False
            if self.send_flag:
                desired_heading_angle = self._adaptive_line_of_sight()
                self.time_last_call = time.time() 
                print(f"{desired_heading_angle = }") 
            self._flag_reset() 
            

        if not main_flag:
            print("Boat has reached final waypoint!")
        


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

    # #test_boat = boat.Boat(boat_width, 
    #                     boat_length, 
    #                     boat_mass,
    #                     cargo_mass,
    #                     wingsail_chord,
    #                     wingsail_span,
    #                     flap_chord,
    #                     flap_span,
    #                     rudder_plan_area,
    #                     rudder_plan_area,
    #                     inertia_xx,
    #                     inertia_yy,
    #                     inertia_zz)

    #speed_control = speed_control.SpeedControl(test_boat)
    #heading_control = heading_control.HeadingControl(test_boat)
    
    #Waypoints from optimal sailor
    
    waypoints = [
    [29.154156, -94.96709654545455],
    [29.18324690909091, -94.94527836363636],[29.117792363636365, -94.94527836363636]
    ]
    
    def generate_fake_locations(waypoints, num_per_waypoint=5, max_offset=0.0000005):
        fake_waypoints = []
        for lat, lon in waypoints:
            for _ in range(num_per_waypoint):
                lat_offset = np.random.uniform(-max_offset, max_offset)
                lon_offset = np.random.uniform(-max_offset, max_offset)
                fake_lat = lat + lat_offset
                fake_lon = lon + lon_offset
                fake_waypoints.append([fake_lat, fake_lon])
        return fake_waypoints
    fake_locations = generate_fake_locations(waypoints)
    #print(fake_locations)
    test_points = [[29.154156169009998, -94.96709657348266], [29.158, -94.96709666802587],[29.154155678335865, -94.96709666802587], [29.15415589192478, -94.96709689413363], [29.15415642620672, 
-94.96709660972977], [29.154155838376713, -94.9670960581525], [29.18324689090205, -94.9452782507665], [29.183247046119195, -94.94527794599699], [29.183247365980655, -94.9452784774041], [29.18324671995424, -94.94527796516381], [29.183246671772643, -94.94527793901297], [29.154156060622487, -94.94527790615798], [29.1541564741837, -94.94527854879291], [29.15415572762446, -94.9452781312552], [29.15415648581165, -94.9452787367832], [29.154156325597825, -94.94527821621519], [29.117792270267056, -94.94527826483953], [29.1177922476054, -94.94527787281646], [29.117791996314377, -94.94527801488928], [29.117792102162543, -94.94527789679252], [29.117792780007075, -94.94527855706544], [29.146882954766788, -94.92346030391982], [29.14688300119124, -94.92346007317839], [29.146883256532902, -94.92346044626757], [29.146883747458627, -94.92345985786923], [29.14688348265838, -94.9234601244442], [29.175973737032574, -94.90164239469108], [29.175974001922178, -94.90164213283704], [29.175974031568362, -94.90164154015814], [29.175974602722505, -94.90164227894789], [29.17597410736935, -94.90164233205903], [29.20506477912145, -94.87982408031272], [29.205065196446572, -94.87982365256163], [29.20506495322424, -94.87982361980649], [29.205065193331983, -94.8798234755031], [29.20506459420589, -94.8798240988502], [29.234155847985836, -94.85800585473795], [29.234156218081786, -94.85800525140192], [29.234155782249672, -94.85800521553048], [29.234155757380233, -94.8580060508467], [29.234155973249596, -94.85800604132895], [29.197791879354845, -94.85800573789821], [29.19779254281267, -94.85800568818453], [29.19779274585514, -94.858006080645], [29.19779285909912, -94.85800597821414], [29.19779281238206, -94.85800583346888], [29.19051946302414, -94.85800595355177], [29.19051948798558, -94.8580060448409], [29.190519635021655, -94.8580057792402], [29.190519197641734, -94.85800581000348], [29.190519961524792, -94.858005718306], [29.175974031069888, -94.85800574621581], [29.175974076519196, -94.85800609818207], [29.17597376395545, -94.85800558246106], [29.175974164318177, -94.85800591177167], [29.17597381307063, -94.85800542304058], [29.146883259522358, -94.85800592274475], [29.14688358609994, -94.85800556810764], [29.14688368225769, -94.85800593457255], [29.146883007382034, -94.85800596920974], [29.14688327686096, -94.85800538096706], [29.13233829179063, -94.85800597778577], [29.132338089928215, -94.85800556537377], [29.13233826709476, -94.85800528624122], [29.132338052465666, -94.8580056336787], [29.132337506069902, -94.85800543428145], [29.12506525961372, -94.85800514309919], [29.12506546754281, -94.85800525711149], [29.125065409663108, -94.85800531654488], [29.125065137706194, -94.85800589706486], [29.12506525246422, -94.85800560870372], [29.117791889657106, -94.85800599865982], [29.117792423226508, -94.85800568533422], [29.117792450175017, -94.85800564151594], [29.11779219930642, -94.85800587101484], [29.117791913245817, -94.85800513734229], [29.146882881452207, -94.83618755782467], [29.14688311065851, -94.83618721879488], [29.146883353976747, -94.83618770487867], [29.146882878753726, -94.836187168798], [29.146882827001768, -94.83618729302998], [29.175974225811924, -94.8143694832922], [29.175974422567123, -94.81436900322421], [29.175974284105255, -94.81436968898224], [29.175973732443826, -94.8143690869311], [29.175974465196017, -94.81436892859894], [29.20506509847459, -94.79255117212475], [29.20506508945696, -94.79255139153274], [29.20506467242124, -94.79255128599125], [29.205065111521648, -94.79255109304114], [29.20506546137841, -94.79255142232395], [29.234155671209624, -94.77073275077741], [29.23415605069485, -94.77073300855747], [29.234156499116292, -94.7707324439476], [29.23415580794714, -94.77073250430588], [29.234156459036154, -94.77073308004667], [29.26324651704923, -94.74891514408779], [29.263247000327873, -94.74891509263698], [29.26324725912376, -94.74891520227908], [29.26324675338893, -94.74891445765628], [29.26324729909418, -94.74891504351027], [29.292338007331878, -94.73436881460056], [29.292337328451953, -94.73436970529424], [29.292337477247532, -94.73436976353578], [29.29233782638461, -94.73436880153662], [29.292337950013856, -94.7343689208731], [29.306883085668137, -94.7270967704458], [29.306883202086457, -94.72709691895767], [29.306883138490388, -94.72709681892347], [29.30688376371147, -94.72709612860817], [29.306883466926514, -94.72709679046106], [29.335973935551255, -94.7198238311659], [29.33597407141839, -94.71982352889336], [29.335974648410218, -94.71982354517826], [29.335974279196705, -94.71982396722821], [29.3359740722946, -94.71982347551733], [29.350519968735387, -94.72709617699232], [29.35051963010154, -94.72709638322414], [29.350519211510367, -94.72709686596062], [29.350519312809244, -94.72709612266853], [29.350519711589314, -94.72709691416043], [29.357792650373643, -94.75618726812914], [29.357792241840112, -94.75618749961183], [29.357792235267176, -94.75618780866776], [29.357791914238227, -94.75618775398011], [29.35779198973239, -94.75618791903896], [29.365065158744745, -94.78527873011855], [29.365064597554202, -94.78527802617951], [29.365065047791177, -94.78527878962933], [29.365064753753245, -94.78527797890858], [29.3650653978448, -94.78527811223599], [29.372337607091808, -94.79982407307745], [29.372337343600552, -94.79982332752493], [29.372337636464458, -94.79982343313446], [29.372338010400124, -94.79982368705168], [29.372338033628484, -94.79982339998857], [29.386883261209938, -94.80709685419288], [29.38688329675252, -94.80709632299312], [29.386882860319982, -94.80709661117449], [29.386883759355403, -94.80709605950803], [29.386883069152393, -94.80709625845431], [29.41597445438564, -94.81436913928225], [29.415974339580217, -94.81436912010517], [29.415974476442322, -94.81436976753595], [29.41597454199806, -94.81436945892412], [29.41597421569304, -94.81436948407917], [29.445065510072606, -94.82164161663304], [29.44506494292715, -94.82164228364692], [29.445065193456546, -94.82164156629992], [29.445064790637964, -94.82164195996353], [29.445065080386225, -94.82164212498134], [29.48142909312584, -94.8216416613381], [29.481429209846773, -94.82164207854186], [29.48142917504679, -94.8216417928598], [29.48142882519947, -94.82164245629293], [29.481428742570674, -94.82164200795637], 
[29.510519936052873, -94.82164150634931], [29.510519507418277, -94.82164209499113], [29.51051983118231, -94.82164168361439], [29.51051955043901, -94.82164198798388], [29.51051969209793, -94.8216416021618], [29.539610548639114, -94.82164181028722], [29.53961096939429, -94.82164210729876], [29.5396100509394, -94.82164157161904], [29.539610204693158, -94.82164154584093], [29.53961050576614, -94.82164217258416], [29.54688282452981, -94.82164208294763], [29.546882855485467, -94.82164158155115], [29.54688369790855, -94.82164166538955], [29.546883431292468, -94.82164178197743], [29.54688376975619, -94.821641790995], [29.554156470549728, -94.82164170993178], [29.554156242567196, -94.82164220514059], [29.55415595477304, -94.8216422104837], [29.55415551617935, -94.82164198222944], [29.55415585832066, -94.82164197554917], [29.561429023811087, -94.82164247415184], [29.561428256213286, -94.82164170127292], [29.561428589970596, -94.82164190908661], [29.561428571501004, -94.82164185405753], [29.561428230682814, -94.82164241497452], [29.568701676986077, -94.82164213854368], [29.568701253564935, -94.82164154824115], [29.568701815603024, -94.82164230852096], [29.568701259139775, -94.82164159051524], [29.56870189410567, -94.82164232773633], [29.575974522759214, -94.82164195419256], [29.575973934601368, -94.82164204108871], [29.57597408434728, -94.82164209654967], [29.575973826319647, -94.82164176637878], [29.575973714979114, -94.82164185981543], [29.583246506074357, -94.82164160627617], [29.58324707025208, -94.82164218688658], [29.58324662856699, -94.821641924785], [29.583246871776634, -94.82164181690854], [29.58324651979889, -94.8216416633536], [29.590519583857798, -94.82164248368109], [29.590519518316082, -94.82164242551272], [29.590519697002012, -94.82164170340184], [29.5905196849785, -94.82164206060347], [29.590520055515274, -94.82164195346591], [29.59779234843291, -94.82164159378411], [29.59779232741954, -94.82164238611298], [29.597792555226835, -94.82164186981748], [29.59779260755741, -94.82164158835917], [29.597791917526628, -94.82164168180323], [29.605065232624057, -94.82164198787962], [29.605065038851173, -94.82164245200062], [29.605064778601594, -94.82164194010033], [29.60506550516383, -94.82164200239322], [29.605065195081217, -94.82164155221128], [29.61233757407559, -94.82164220959366], [29.61233820952968, -94.82164216666959], [29.612337714260505, -94.82164167370813], [29.61233770121792, -94.82164222424379], [29.61233778759956, -94.82164201125653], [29.619610859135438, -94.82164202248737], [29.61961034406793, -94.82164231707661], [29.6196101732834, -94.82164230484376], [29.619610364080987, -94.82164150604477], [29.61961074732571, -94.82164166479545], [29.62688368773979, -94.8216415505894], [29.62688373465377, -94.8216417092499], [29.62688355227806, -94.82164164608074], [29.62688302095826, -94.82164247438733], [29.626883254755796, -94.82164191779552], [29.634156345455423, -94.82164220091154], [29.634155839140444, -94.82164239770056], [29.63415649584526, -94.8216424475457], [29.634156012945972, -94.82164196412822], [29.63415597296591, -94.8216422740492], [29.64142860110614, -94.82164209024282], [29.641428362310176, -94.82164181937466], [29.641429195468668, -94.82164193569773], [29.64142892252675, -94.82164221746739], [29.64142878895264, -94.82164200228652], [29.64870173421234, -94.82164224233725], [29.64870168126512, -94.82164201139422], [29.648701443274366, -94.82164151242647], [29.64870146099294, -94.82164207477679], [29.648701373273568, -94.8216418595477], [29.655973919128098, -94.8216421529908], [29.655974519850062, -94.82164225030164], [29.655974563219242, -94.8216421830335], [29.655973746623765, -94.82164214873282], [29.6559737962051, -94.82164170576507], [29.66324659917172, -94.82164221287037], [29.663246995018433, -94.82164150334503], [29.66324734642483, -94.82164227925928], [29.66324709869546, -94.82164207107577], [29.663246967022808, -94.82164202798134], [29.670519839738606, -94.82164232017449], [29.670519851887516, -94.82164171147427], [29.670520126568228, -94.82164183652118], [29.670519522107448, -94.821641934574], [29.670519617154046, -94.82164196544184], [29.677791966612627, -94.82164236856181], [29.677792340136513, -94.82164171585892], [29.677792655531448, -94.82164206479202], [29.6777927635428, -94.82164163838836], [29.677792055572542, -94.82164183699886], [29.685065452159325, -94.8216417092157], [29.685065266968387, -94.82164156363169], [29.685064730027516, -94.82164247706467], [29.68506509972021, -94.8216418744128], [29.685064894706667, -94.82164182821207], [29.692337333767266, -94.82164231477918], [29.692338269581523, -94.82164183855451], [29.692337608450437, -94.82164231043163], [29.692338301361804, -94.82164229867828], [29.692337404378904, -94.82164188778826], [29.721428527031623, -94.81436962821726], [29.721428515007954, -94.814369125723], [29.72142848841934, -94.81436892222818], [29.721428500199014, -94.81436964849999], [29.721428836547002, -94.81436942976573], [29.750519304828064, -94.80709622732113], [29.750519886475885, -94.80709647962935], [29.750519451832044, -94.80709632004304], [29.750519539016047, -94.80709660824158], [29.750519973922035, -94.807096477019]]
    test_lpp = LocalPathfinder(fake_locations, waypoints)
    test_lpp.run()