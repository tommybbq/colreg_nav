import os

import gphoto2 as gp


class Camera:
    def __init__(self) -> None:
        self.camera = gp.Camera()
        self.camera.init()

    def capture(self, save_directory: str = "./") -> None:
        file_path = self.camera.capture(gp.GP_CAPTURE_IMAGE)
        target_location = os.path.join(save_directory, file_path.name)
        camera_file = self.camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL)
        camera_file.save(target_location)

    # def delete(self, path="/") -> None:
    #     result = []
    #     for name, value in self.camera.folder_list_files(path):
    #         result.append(os.path.join(path, name))

    #     folders = []
    #     for name, value in self.camera.folder_list_folders(path):
    #         folders.append(name)

    #     for name in folders:
    #         result.extend(self.delete(os.path.join(path, name)))

    #     return result


if __name__ == "__main__":
    cam = Camera()
    cam.capture()
