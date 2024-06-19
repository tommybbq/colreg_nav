import time

from lib import config_loader, sailor_serial

if __name__ == "__main__":
    serial_cfg = config_loader.ConfigLoader().load_file("config/serial.toml")
    boat_cfg = config_loader.ConfigLoader().load_file("config/boat.toml")
    sailor = sailor_serial.SailorSerial(serial_cfg, boat_cfg)

    while True:
        user_input = input("Enter starboard angle: ")
        if user_input == "x":
            break
        user_angle = float(user_input)

        #sailor.send_flap(user_angle)

        sailor.rudder_angle(user_angle)

        status_code, data = sailor.receive_all()
        time.sleep(0.01)

        print(status_code, data)
