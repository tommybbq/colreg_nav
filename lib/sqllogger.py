import sqlite3
import uuid

from boat import boat
import threading
import time


# generate a random UUID

random_uuid = uuid.uuid4()

class SQLLogger():
    
    def __init__(self, database):
        self.db_path = database
        self.session_id = str(uuid.uuid4())
        
        # try to connect to database, raise error if it cannot
        try:
            self.conn = sqlite3.connect(self.db_path, isolation_level=None)
            self.conn.execute("pragma journal_mode=wal")
            self.cursor = self.conn.cursor()

        except Exception:
            # place holder, replace 
            print("Cannot connect to database")
    
    def new_session(self):
        self.session_id = str(uuid.uuid4())
        
    def log_motor_data(self, motor_name: str, angle: float, raw_angle=None):
        self.cursor.execute("INSERT INTO MotorData (session_id, motor_name, angle, raw_angle) VALUES (?, ?, ?, ?)",
            (self.session_id, motor_name, angle, raw_angle)
        )
        self.conn.commit()
    
    def log_wind_data(self, speed: str, angle: float):
        self.cursor.execute("INSERT INTO WindData (session_id, wind_speed, wind_direction) VALUES (?, ?, ?);",
            (self.session_id, speed, angle)
        )
        self.conn.commit()
    
    def log_gps_data(self, latitude, longitude, speed):
        self.cursor.execute("INSERT INTO GPSData (session_id, latitude, longitude, speed) VALUES (?, ?, ?, ?);",
            (self.session_id, latitude, longitude, speed)
        )
        self.conn.commit()

    def log_imu_data(self, pitch, roll, yaw):
        self.cursor.execute("INSERT INTO IMUData(session_id, euler_x, euler_y, euler_z) VALUES (?, ?, ?, ?);",
           (self.session_id, pitch, roll, yaw)
        )
        self.conn.commit()

    def log_lpp_data(self, current_waypoint_latitude, current_waypoint_longitude, distance_to_waypoint, latitude, longitude, desired_heading):
        self.cursor.execute("INSERT INTO LPPData(session_id, current_waypoint_latitude, current_waypoint_longitude, distance_to_waypoint, latitude, longitude, desired_heading) VALUES (?, ?, ?, ?, ?, ?, ?);",
            (self.session_id, current_waypoint_latitude, current_waypoint_longitude, distance_to_waypoint, latitude, longitude, desired_heading)
        )
        self.conn.commit()

if __name__ == "__main__":
    pass
    # print("Starting")
    # sqllogger = SQLLogger("/home/antonio/guide-nav/databases/NavigationData_copy.db")
    # sqllogger.log_lpp_data(30, 30, 0.5, 20, 20, 300)
    # # sqllogger.log_motor_data("test1", 1, 2)
    # #sqllogger.log_motor_data("test2", 3)
    # test_boat = boat.Boat()
    # # def read_and_log_data():
    # sqllogger = SQLLogger("/home/antonio/guide-nav/databases/LakeTestDatabase.db")
    # for i in range(0,10):
    #     print(i)
    #     test_boat.update()
    #     test_boat.sql_log(sqllogger)
    #     sqllogger.log_lpp_data(30, 30, 0.5, 20, 20, 300)
            #qllogger.log_motor_data(3)

        # time.sleep(0.08) 


    # read_thread = threading.Thread(target = read_and_log_data, daemon = True)
    # read_thread.start()
    # read_thread.join()