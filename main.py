import datetime
import math
import os
import queue
import threading
import time

import numpy as np

import helper_functions as hf
from boat import boat
from lib import config_loader, sqllogger
from local_planning import local_pathfinder
from global_planning import global_pathfinder
from object_avoidance import camera, detect
from sailor import heading_control, sailor, speed_control
from web_backend import websocketserver

PICTURE_DIR = "/home/antonio/Pictures/"


def clear_directory(dir_path: str) -> None:
    for file_name in os.listdir(dir_path):
        file_path = os.path.join(dir_path, file_name)
        os.unlink(file_path)


def read_and_log_data(auto_boat: boat.Boat) -> None:
    sql = sqllogger.SQLLogger("/home/antonio/guide-nav/databases/NavigationData.db")
    while True:
        auto_boat.update()
        auto_boat.sql_log(sql)
        time.sleep(0.08)


def log_pictures(cam: camera.Camera, picture_taken: threading.Event) -> None:
    while True:
        if not picture_taken.is_set():
            cam.capture(PICTURE_DIR)
            picture_taken.set()

            # TODO: sleep?


def detect_objects(
    detector: detect.Detect,
    detection_queue: queue.Queue,
    picture_taken: threading.Event,
    object_detected: threading.Event,
) -> None:
    while True:
        picture_taken.wait()

        # SHOULD now just have one file in it
        detections = detector.inference(PICTURE_DIR)
        clear_directory(PICTURE_DIR)
        picture_taken.clear()

        for detection in detections[0]:
            if detection:
                detection_queue.put(detection)
                object_detected.set()


def run(
    boat: boat.Boat,
    heading_control,
    speed_control,
    host,
    port,
    destination,
    detection_queue: queue.Queue,
    objected_detected: threading.Event,
):
    # Path for script to call Optimal Sailor
    script_path = (
        "/home/antonio/Optimal_Sailor/Variable_Wind_Astar/main.py"
    )
    subprocess_working_dir = (
        "/home/antonio/Optimal_Sailor/Variable_Wind_Astar"
    )

    full_sailor = sailor.Sailor(
        boat,
        speed_control.extremum_seeking,
        heading_control.line_following_control,
        default_speed_control_method=speed_control.rc_control,
        default_heading_control_method=heading_control.rc_control,
    )

    full_gpp = global_pathfinder.GlobalPathfinder(boat, host, port, script_path, subprocess_working_dir)
    full_gpp.initialize_global(destination)

    full_lpp = local_pathfinder.LocalPathfinder(full_sailor, full_gpp.waypoints)
    print(full_lpp.waypoints)

    destination_reached = False
    path_still_valid = True
    object_in_path = False

    while not destination_reached:
        #Check if object is detected and then if in path

        if object_detected.is_set():
            all_detections = []
            object_headings = []
            object_distances = []
            #Goes through queue of detections for each time model is run
            while not detection_queue.empty():
                detection = detection_queue.get()
                all_detections.append(detect)

            #Goes through the detections and returns the heading and distance of each obstacle
            for detection in all_detections:
                object_headings.append(detection.relative_headings())
                object_distances.append(detection.estimated_distance(full_gpp.distance_to_shore))

            object_in_path = full_gpp.objected_detected(object_headings, object_distances)
            if object_in_path:
                full_lpp.set_waypoints(full_gpp.waypoints)
                print("Object detected in path!")
                #Reset object_in_path
                object_in_path = False

            objected_detected.clear()

        lpp_status = full_lpp.run()

        if lpp_status is not None:
            if not lpp_status:
                #TODO:
                #   How do we handle barriers/obstacles
                #full_gpp.update_global(destination, [[29.561081, -95.061381],[29.558669, -95.060380],[29.555118, -95.059186]])
                #full_lpp.set_waypoints(full_gpp.waypoints)
                pass
            else:
                break


    # station_keeping()


if __name__ == "__main__":
    # Communicate with the server
    host = "localhost"
    port = 1336

    # Boat destination
    destination = [29.558518, -95.048904]

    # Create instance of boat
    auto_boat = boat.Boat()

    # Create instances of heading control and speed control
    full_heading_control = heading_control.HeadingControl(auto_boat)
    full_speed_control = speed_control.SpeedControl(auto_boat)

    #Prepare for photos
    cam = camera.Camera()
    clear_directory(PICTURE_DIR)
    picture_taken = threading.Event()

    #Prepare model
    detection_queue = queue.Queue()
    object_detected = threading.Event()
    cv_config = config_loader.ConfigLoader().load_file("config/object_avoidance.toml")
    cv_model = detect.Detect(cv_config)

    # Define threads
    read_thread = threading.Thread(target=read_and_log_data, args=(auto_boat,), daemon=True)
    send_thread = threading.Thread(
        target=run,
        args=(
            auto_boat,
            full_heading_control,
            full_speed_control,
            host,
            port,
            destination,
            detection_queue,
            object_detected,
        ),
        daemon=True,
    )
    camera_thread = threading.Thread(
        target=log_pictures, args=(cam, picture_taken), daemon=True
    )
    detection_thread = threading.Thread(
        target=detect_objects,
        args=(cv_model, detection_queue, picture_taken, object_detected),
        daemon=True,
    )


    # Start the threads
    read_thread.start()
    send_thread.start()
    camera_thread.start()
    detection_thread.start()

    # Wait for the threads to finish (you may choose to handle this differently)
    read_thread.join()
    send_thread.join()
    camera_thread.join()
    detection_thread.join()
 
