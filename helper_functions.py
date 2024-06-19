import math
import csv

#CONFIRM
def convert_tics_to_degrees(tics) -> float:
    length = (4/1171)*tics
    converted_degrees = 90 - math.degrees(math.acos((10.6 ** 2 + 3 ** 2 - (length + 8.94) ** 2) / (2 * 10.6 * 3))) 
    
    return converted_degrees

def wrap_around_clip(value) -> float:

    if value > 180:
        value -= 360
    elif value < -180:
        value += 360
    return value

def find_nearest_key(dataset, angle_of_attack):
    """
    Finds the key in the dictionary for the value nearest to the angle_of_attack

    Parameters:
    - dataset: dict: Key is angle of attack and values are lift and drag coefficients
    - angle_of_attack: Angle of attack of wing

    Returns:
    - int: Key

    """

    differences = float('inf')
    best_key = None

    for key in dataset:
        temp_diff = abs(key - angle_of_attack)
        if temp_diff < differences:
            differences = temp_diff
            best_key = key

    return best_key

def haversine(coordinate_1: list, coordinate_2: list):
    """
    Calculates the distance between two points on the eart (specified in decimal degrees)
    """

    lat1, lon1 = coordinate_1
    lat2, lon2 = coordinate_2

    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    distance_lat = lat2 - lat1
    distance_lon = lon2 - lon1

    a = math.sin(distance_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(distance_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    r = 6371000                                                 #Radius of Earth in meters
    return c * r

def find_bearing(coordinate_1: list, coordinate_2: list):
    
    lat1, lon1 = coordinate_1
    lat2, lon2 = coordinate_2
    
    distance_lon = math.radians(lon2 - lon1)
    y = math.sin(distance_lon) * math.cos(math.radians(lat2))
    x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))\
          - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(distance_lon)
    bearing = math.atan2(y,x)
    bearing = math.degrees(bearing)
    
    return bearing

def cross_product(vector_1 : list, vector_2 :list):
    #Calculates cross product of two vectors
    vector_1.append(0)
    vector_2.append(0)
    output = []
    output.append(vector_1[1] * vector_2[2] - vector_1[2] * vector_2[1])
    output.append(vector_1[2] * vector_2[0] - vector_1[0] * vector_2[2])
    output.append(vector_1[0] * vector_2[1] - vector_1[1] * vector_2[0])

    return output

def cross_product3(a, b):
    result = [a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]

    return result

def magnitude(vector): 
    return math.sqrt(sum(pow(element, 2) for element in vector))

def dot_product(vec1, vec2):
    """
    input:
        vec1 - 1 x p vector represented by an array
        vec2 - p x 1 vector represented by an array
            *p is a positive integers
    output:
        dot - dot product between the two vectors
    function takes the dot product of two vectors
    """
    dot = 0
    for i in range(len(vec1)):
        prod = vec1[i] * vec2[i]
        dot += prod
    return dot

def read_csv_to_list(filename):
    """
    Reads data from a CSV file and converts it into a list of lists.

    Args:
    - filename: The path to the CSV file.

    Returns:
    - A list of lists where each inner list represents a row of data from the CSV.
    """
    data = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            row_float = [float(value) for value in row]
            data.append(row_float)
    return data


