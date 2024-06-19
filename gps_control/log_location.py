import gps
import sqlite3

# Listen on port 2947 (gpsd) of localhost
session = gps.gps("localhost", "2947")
session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

sqlCommand = 'INSERT INTO GPSData (satellites, fix, quality, longitude, latitude, angle, speed_knots, alt_cm, rec_time, rec_date) VALUES (3, 1, 1, 0.0, 0.0, 0.14, 161.23, 275.8, '20:24:11.0','16/4/2012');'

while True:
    try:
        report = session.next()
        # Wait for a 'TPV' report and display the current time
        # To see all report data, uncomment the line below
        #print(report)
        print(report.keys());
        if report['class'] == 'TPV':
            if hasattr(report, 'time'):
                print(report.time)
            if hasattr(report, 'track'):
                print(report.track)
            if hasattr(report, 'speed'):
                print(report.speed * gps.MPS_TO_KPH)
            if hasattr(report, 'lat'):
                print(report.lat)
            if hasattr(report, 'lon'):
                print(report.lon)

    except KeyError:
        pass
    except KeyboardInterrupt:
        quit()
    except StopIteration:
        session = None
        print("GPSD has terminated")
