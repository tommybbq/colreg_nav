import math
import struct
import time

import serial

import lib.config_loader as config_loader
import lib.not_bad_serial as not_bad_serial


class SailorSerial(not_bad_serial.NotBadSerial):
    def __init__(
        self,
        serial_config: config_loader.ConfigLoader,
        boat_config: config_loader.ConfigLoader,
    ) -> None:
        self.serial_config = serial_config.data
        self.cmds = self.serial_config.commands
        self.boat_constraints = boat_config.data.physical_constraints
        con_settings = self.serial_config.connection

        self.serial_connection = serial.Serial(
            port=con_settings.port,
            baudrate=con_settings.baud_rate,
            bytesize=con_settings.byte_size,
            parity=con_settings.parity,
            stopbits=con_settings.stop_bits,
            timeout=con_settings.read_timeout,
            write_timeout=con_settings.write_timeout,
        )

        self.flap_timeout = 5

        self.last_flap_input_time = 0
        self.last_rudder_input_time = 0

        if not self.serial_connection.is_open:
            self.serial_connection.open()

        super().__init__(self.serial_connection)

    def send_begin(self) -> None:
        self.send_message(self.cmds.begin, self.string_t)

    def send_end(self) -> None:
        self.send_message(self.cmds.end, self.string_t)

    def send_safe_turn_off(self) -> None:
        self.send_message(self.cmds.off, self.string_t)

    def send_rudder(self, angle: float, target: str) -> None:
        int_angle = round(angle)
        if int_angle > self.boat_constraints.max_rudder_angle:
            int_angle = self.boat_constraints.max_rudder_angle
        elif int_angle < self.boat_constraints.min_rudder_angle:
            int_angle = self.boat_constraints.min_rudder_angle

        if target == "multi":
            self.send_message(self.cmds.multi, self.string_t)
        elif target == "port":
            self.send_message(self.cmds.port, self.string_t)
        elif target == "starboard":
            self.send_message(self.cmds.starboard, self.string_t)
        else:
            raise Exception(
                "Invalid target provided. Must be one of multi, port, or starboard"
            )

        self.send_message(int_angle, self.int_t)

    def send_flap(self, angle) -> None:
        flap_time_diff = time.time() - self.last_flap_input_time
        if flap_time_diff < self.flap_timeout:
            print("Attempted to write flap angles too quickly. Aborted")
            print(f"{flap_time_diff =}")
            return False

        if angle > self.boat_constraints.max_flap_angle:
            angle = self.boat_constraints.max_flap_angle
        elif angle < self.boat_constraints.min_flap_angle:
            angle = self.boat_constraints.min_flap_angle

        angle = 90 - angle

        # between 0 and 4 inches
        length = (
            math.sqrt(
                (self.boat_constraints.lin_actuator_mounting_len**2)
                + (self.boat_constraints.flap_mount_arm**2)
                - (
                    2
                    * self.boat_constraints.lin_actuator_mounting_len
                    * self.boat_constraints.flap_mount_arm
                    * math.cos(math.radians(angle))
                )
            )
            - self.boat_constraints.lin_actuator_retracted_len
        )

        int_length = round(length * 1171.0 / 4.0)
        if int_length > 1171:
            int_length = 1171
        elif int_length < 0:
            int_length = 0

        self.last_flap_input_time = time.time()

        self.send_message(self.cmds.flap, self.string_t)
        self.send_message(int_length, self.int_t)

    def receive_all(self) -> tuple[bool | dict]:
        try:
            current_command = self.receive_message(self.string_t)[1]
            read_count = 1
            while current_command != self.cmds.begin:
                current_command = self.receive_message(self.string_t)[1]
                read_count += 1

                # read timeout failure
                if read_count >= self.serial_config.messaging.max_read_count:
                    return False, None

            # success!
            received_data = {
                "port": None,
                "starboard": None,
                "flap": None,
                "wingsail": None,
                "initialization": None,
                "euler": None,
                "magnetometer": None,
                "acceleration": None,
                "wind_speed": None,
                "wind_angle": None,
            }

            while current_command != self.cmds.end:
                current_command = self.receive_message(self.string_t)[1]

                if current_command == self.cmds.port:
                    received_data["port"] = self.receive_message(self.int_t)[1]
                elif current_command == self.cmds.starboard:
                    received_data["starboard"] = self.receive_message(self.int_t)[1]
                elif current_command == self.cmds.flap:
                    received_data["flap"] = self.receive_message(self.int_t)[1]
                elif current_command == self.cmds.initialization:
                    received_data["initialization"] = [
                        self.receive_message(self.int_t)[1]
                    ]
                    received_data["initialization"].append(
                        self.receive_message(self.int_t)[1]
                    )
                    received_data["initialization"].append(
                        self.receive_message(self.int_t)[1]
                    )
                    received_data["initialization"].append(
                        self.receive_message(self.int_t)[1]
                    )
                elif current_command == self.cmds.uler:
                    received_data["euler"] = [self.receive_message(self.float_t)[1]]
                    received_data["euler"].append(self.receive_message(self.float_t)[1])
                    received_data["euler"].append(self.receive_message(self.float_t)[1])
                elif current_command == self.cmds.magnetometer:
                    received_data["magnetometer"] = [
                        self.receive_message(self.float_t)[1]
                    ]
                    received_data["magnetometer"].append(
                        self.receive_message(self.float_t)[1]
                    )
                    received_data["magnetometer"].append(
                        self.receive_message(self.float_t)[1]
                    )
                elif current_command == self.cmds.acceleration:
                    received_data["acceleration"] = [
                        self.receive_message(self.float_t)[1]
                    ]
                    received_data["acceleration"].append(
                        self.receive_message(self.float_t)[1]
                    )
                    received_data["acceleration"].append(
                        self.receive_message(self.float_t)[1]
                    )
                elif current_command == self.cmds.wind_speed:
                    received_data["wind_speed"] = self.receive_message(self.float_t)[1]
                elif current_command == self.cmds.wind_angle:
                    received_data["wind_angle"] = self.receive_message(self.float_t)[1]
                elif current_command == self.cmds.wingsail:
                    received_data["wingsail"] = self.receive_message(self.float_t)[1]
                elif current_command != "E":
                    return False, None

            return True, received_data

        except UnicodeDecodeError:
            print("UnicodeDecodeError")
            return False, None
        except struct.error as err:
            print("struct.error")
            return False, None


if __name__ == "__main__":
    import time

    cfg = config_loader.ConfigLoader().load_file("config/serial.toml")
    boat_cfg = config_loader.ConfigLoader().load_file("config/boat.toml")
    test_connection = SailorSerial(cfg, boat_cfg)

    while True:
        for angle in (-15, -10, -5, 0, 5, 10, 15):
            test_connection.send_message(angle, "int16_t")
            time.sleep(0.1)

        status_code, data = test_connection.receive_message("int16_t")
        print(status_code, data)
