import subprocess


class Camera:
    base_argument = ["ptpcam"]


    def run_command(self, arguments: list[str]) -> subprocess.CompletedProcess:
        full_command = self.base_argument + arguments    
        result = subprocess.run(full_command, capture_output=True)
        return result


    def set_property(self, prop: str, value: str) -> subprocess.CompletedProcess:
        command = ["--set-property=" + prop, "--val=" + value]
        result = self.run_command(command)
        return result


    def __init__(self) -> None:
        # wake the camera from sleep
        self.set_property("0xD80E", "0x00")

        # turn off auto power-off
        self.set_property("0xD81B", "0")

        # set to single shot shooting mode
        self.set_property("0x5013", "0x0001")


    def take_picture(self):
        self.run_command("-c")

if __name__ == "__main__":
    cam = Camera()
