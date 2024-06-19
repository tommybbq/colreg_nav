import struct
from typing import Any, Union

import serial
import time

SERIAL_BYTE_ORDER = "little"

MAX_STRING_RECEIVE_SIZE = 64
HEADER = b"\x28"
FOOTER = b"\x29"


class NotBadSerial:
    float_t = "float"
    uint_t = "uint16_t"
    int_t = "int16_t"
    string_t = "String"

    _datatype_to_length = {"float": 4, "uint16_t": 2, "int16_t": 2}
    _datatype_to_struct_format = {
        "float": "<f",
        "uint16_t": "<H",
        "int16_t": "<h",
    }  # < prefix means little endian

    def __init__(self, serial_connection: serial.Serial) -> None:
        self.serial_connection = serial_connection

    # Fletcher's checksum
    def _calculate_checksum(self, data: bytes) -> bytes:
        "Performs equivalently"
        c0, c1 = 0, 0
        length = len(data)
        while length > 0:
            block_length = length
            if block_length > 5802:
                block_length = 5802
            length -= block_length

            for i in range(block_length):
                c0 = c0 + data[i]
                c1 = c1 + c0

            c0 = c0 % 255
            c1 = c1 % 255

        result = (c0 << 8) | c1
        return result.to_bytes(2, byteorder=SERIAL_BYTE_ORDER, signed=False)

    def send_message(self, data: Union[str, float], datatype: str) -> None:
        if datatype == "String":
            length = len(data)
            bytes_data = data.encode("utf-8")
        elif (datatype in self._datatype_to_struct_format) and (
            datatype in self._datatype_to_length
        ):
            length = self._datatype_to_length[datatype]
            bytes_data = struct.pack(self._datatype_to_struct_format[datatype], data)
        else:
            raise Exception("Invalid datatype provided")

        bytes_length = length.to_bytes(2, byteorder=SERIAL_BYTE_ORDER, signed=False)

        message = bytearray(HEADER)
        message.extend(bytes_length)
        message.extend(bytes_data)
        message.extend(self._calculate_checksum(bytes_data)[::-1])  # little endian :)
        message.extend(FOOTER)

        try:
            self.serial_connection.write(message)
        except serial.serialutil.SerialTimeoutException:
            print("SerialTimeoutException")
            self.serial_connection.reset_input_buffer()
            time.sleep(0.01)
            self.serial_connection.write(message)


    def receive_message(self, datatype: str):
        available_bytes = self.serial_connection.in_waiting
        available_bytes = 20  # TODO????????

        while self.serial_connection.read() != HEADER and available_bytes > 0:
            available_bytes -= 1

        # ran out of bytes before receiving the header
        if available_bytes <= 0:
            return 1, None

        length_buffer = self.serial_connection.read(size=2)
        data_length = int.from_bytes(
            length_buffer, byteorder=SERIAL_BYTE_ORDER, signed=False
        )

        temp_buffer = self.serial_connection.read(size=data_length)
        calculated_checksum = self._calculate_checksum(temp_buffer)

        if datatype == "String":
            message_data = temp_buffer.decode("utf-8")
        elif datatype in self._datatype_to_struct_format:
            # unpack returns a tuple
            message_data = struct.unpack(
                self._datatype_to_struct_format[datatype], temp_buffer
            )[0]
        else:
            raise Exception("Invalid datatype provided")

        backwards_received_checksum = self.serial_connection.read(size=2)
        received_checksum = backwards_received_checksum[::-1]
        # checksums don't match, likely data corruption
        if calculated_checksum != received_checksum:
            return 3, message_data

        if self.serial_connection.read() != FOOTER:
            return 4, None

        return 0, message_data


if __name__ == "__main__":
    # import sailor_serial

    # serial_port = sailor_serial.init_serial()
    # not_bad_serial = NotBadSerial(serial_port)
    # not_bad_serial.send_message(0, "uint16_t")

    n = NotBadSerial(None)
    print(n._calculate_checksum(b"\x01\x02"))
