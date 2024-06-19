import subprocess
from client_socket import client_socket
import os
import time
from math import radians, degrees, sin, cos, atan2, sqrt
import numpy as np

def find_nearest_point(coords, list):
    """
    Find the nearest point to a query_point in a list of points.
    Uses squared Euclidean distance for efficiency.
    """
    nearest_point = None
    min_distance = float('inf')
    for point in list:
        distance = squared_euclidean_distance(coords, point)
        if distance < min_distance:
            nearest_point = point
            min_distance = distance
    return nearest_point


def squared_euclidean_distance(point1, point2):
    """
    Calculate the squared Euclidean distance between two points.
    Avoids the square root for performance.
    """
    return (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2


def generate_infeasible_grids(current_coords, object_heading, distance, interval):
    """
    Generate a list of latitude and longitude of grid squares that the object may be in
    :param current_coords: Coordinates of boat at time of object detection
    :param object_heading: Angle from True North counterclockwise object is detected
    :param distance: Maximum distance object could be
    :param interval: Spacing between waypoints
    """
    pass

def calculate_bearing(start_point, end_point):
    lat1, lat2 = start_point[0], end_point[0]
    dlon = end_point[1] - start_point[1]
    lat1, lat2, dlon = map(radians, [lat1, lat2, dlon])

    #Heading Formula Calculation
    y = sin(dlon) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)

    bearing = degrees(atan2(y, x))
    bearing = (360 - bearing) % 360
    #bearing = (bearing + 360) % 360
    return bearing


def check_intersection(start_point, heading, waypoints):

    start_bearing = calculate_bearing(start_point, waypoints[0])

    for i in range(1, len(waypoints)):
        segment_end = waypoints[i]

        end_bearing = calculate_bearing(start_point, segment_end)

        print(start_bearing)
        print(end_bearing)

        if abs(start_bearing - heading) < 10 or abs(end_bearing - heading) < 10:
            return True
        
        start_bearing = end_bearing
        
    return False


# # Ensure to import the necessary math functions at the top of your script.
if __name__ == "__main__":
    # waypoints = [[29.562167146341462, -95.07125905365854], 
    #              [29.562167146341462, -95.07125905365854], 
    #              [29.56186651219512, -95.07056642369338], 
    #              [29.56156587804878, -95.06987379372822], 
    #              [29.561265243902437, -95.06918116376306], 
    #              [29.560964609756095, -95.0684885337979], 
    #              [29.560663975609756, -95.06779590383276], 
    #              [29.560363341463415, -95.0671032738676], 
    #              [29.560062707317073, -95.06641064390244], 
    #              [29.55976207317073, -95.06571801393729], 
    #              [29.55946143902439, -95.06502538397213], 
    #              [29.559160804878047, -95.06433275400697], 
    #              [29.558860170731705, -95.06364012404181], 
    #              [29.558559536585364, -95.06294749407665], 
    #              [29.55825890243902, -95.0622548641115], 
    #              [29.557958268292683, -95.06156223414634], 
    #              [29.55765763414634, -95.06086960418118], 
    #              [29.557357, -95.06017697421603], 
    #              [29.557056365853658, -95.05948434425088], 
    #              [29.556755731707316, -95.05879171428572], 
    #              [29.556455097560974, -95.05809908432056], 
    #              [29.556154463414632, -95.0574064543554], 
    #              [29.55585382926829, -95.05671382439024], 
    #              [29.55555319512195, -95.05602119442509], 
    #              [29.55525256097561, -95.05532856445993], 
    #              [29.55525256097561, -95.05498224947735], 
    #              [29.55525256097561, -95.05428961951219], 
    #              [29.55525256097561, -95.05325067456445], 
    #              [29.55525256097561, -95.05290435958187], 
    #              [29.55525256097561, -95.05255804459931], 
    #              [29.55525256097561, -95.05221172961673], 
    #              [29.55525256097561, -95.05186541463415], 
    #              [29.55525256097561, -95.05151909965157], 
    #              [29.55525256097561, -95.05117278466899], 
    #              [29.55525256097561, -95.05082646968641], 
    #              [29.55495192682927, -95.05082646968641]]
    waypoints = [[29.562167146341462, -95.07125905365854], [29.562167146341462, -95.07125905365854], [29.55525256097561, -95.05532856445993], [29.55525256097561, -95.05082646968641], [29.55495192682927, -95.05082646968641]]
    start_point = [29.553887, -95.060789]

    print(check_intersection(start_point, 40, waypoints))