import socket
import pickle
import struct
import time
import subprocess

class ClientSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_socket = None


    def ping_server(self, attempts=5, delay=2):
        for attempt in range(attempts):
            try:
                if self.client_socket:
                    self.client_socket.close()

                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.host, self.port))
                    
                ping_message = pickle.dumps("ping")
                message_length = struct.pack('>I', len(ping_message))
                self.client_socket.sendall(message_length + ping_message)
                
                # Wait for and process the pong response
                raw_length = self.client_socket.recv(4)
                if not raw_length:
                    raise Exception("Failed to receive data")
                data_length = struct.unpack('>I', raw_length)[0]
                
                received_data = b''
                while len(received_data) < data_length:
                    packet = self.client_socket.recv(data_length - len(received_data))
                    if not packet:
                        raise Exception("Failed to receive data")
                    received_data += packet
                    
                response = pickle.loads(received_data)
                if response == "pong":
                    print("Ping successful.")
                    return True
                
            except Exception as e:
                print(f"Ping attempt {attempt + 1} failed: {e}")
                print(subprocess.check_output(["lsof", "-i", f":{1234}"]))
                time.sleep(delay)
        return False


    def send(self, data_dict):
        serialized_data = pickle.dumps(data_dict)
        #Length of the pickled data
        length_prefix = struct.pack('>I', len(serialized_data))

        try:
            self.client_socket.sendall(length_prefix + serialized_data)
            print("Data sent")
        except Exception as e:
            print(f"Error: {e}")

    
    def receive(self):
        #Read the length of the pickled data
        raw_length = self.client_socket.recv(4)
        if not raw_length:
            raise RuntimeError("Socket connection broken")
        data_length = struct.unpack('>I', raw_length)[0]

        try:
            received_data = b''
            while len(received_data) < data_length:
                packet = self.client_socket.recv(data_length - len(received_data))
                if not packet:
                    raise RuntimeError("Socket connection broken")
                received_data += packet
            # received_data = self.client_socket.recv(1024)

            self.data = pickle.loads(received_data)

        except Exception as e:
            print(f"Error: {e}")
            # return None


    def close(self):
        self.client_socket.close()
