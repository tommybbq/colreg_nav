from rudder import rudder
speed_x = 5
speed_y = 3
yaw_velocity =2
roll_velocity = 0 
trueWind_x = 11
trueWind_y = 10
app_wind_X = 16
app_wind_Y = 10
coefficient_dataset = [1,1]
rudder = rudder.Rudder(coefficient_dataset)
print(rudder.rudder_force(speed_x, speed_y,yaw_velocity, roll_velocity,trueWind_x,trueWind_y,app_wind_X, app_wind_Y))
#print(rudder.moment(speed_x, speed_y,yaw_velocity, roll_velocity,trueWind_x,trueWind_y, app_wind_X, app_wind_Y))