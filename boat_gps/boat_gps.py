import gps
import time
import threading

MIN_REFRESH = 0.1

class BoatGPS(threading.Thread):

    """ If the GPS isn't connecting, try restarting the daemon 
    $ sudo gpsd /dev/ttyUSB1 -F /var/run/gpsd.sock"""

    def __init__(self):
        threading.Thread.__init__(self)

        # Listen on port 2947 (gpsd) of localhost
        self.session = gps.gps("localhost", "2947")
            

        self.session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        self.last_call_time = 0

        self.lat = 0
        self.lon = 0
        self.speed = 0
        self.track = 0
        self.time = 0

        self.start()

    def run(self):
        # Check every 0.2 seconds
        time.sleep(MIN_REFRESH)

        while True:
            try:
                report = self.session.next()

                # Wait for a 'TPV' report and display the current time
                # To see all report data, uncomment the line below
                if report['class'] == 'TPV':
                    if hasattr(report, 'time'):
                        self.time = report.time
                        #print(self.time)
                    if hasattr(report, 'lat'):
                        self.lat = report.lat
                        #print(report.lat)
                    if hasattr(report, 'lon'):
                        self.lon = report.lon
                        #print(report.lon)
                    if hasattr(report, 'speed'):
                        self.speed = report.speed
                        #print(report.speed)
                    if hasattr(report, 'track'):
                        self.track = report.track
                        #print(report.track)

                    self.last_call_time = time.time()

                    return True, 
                        
            except KeyError:
                pass
            except StopIteration:
                self.session = None
                print("GPSD has terminated")

    def update(self):
        # :) for compatibility
        pass
        
if __name__ == "__main__":
    myGPS = BoatGPS()
