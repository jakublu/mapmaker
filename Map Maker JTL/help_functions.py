#coding:utf-8
import math
import communicator_robot
import sys

# Coordinate accuracy, the actual coordinates will be converted to 0.5 precision


# def EulerOrientation(array):
#     w = array['W']
#     x = array['X']
#     y = array['Y']
#     z = array['Z']
#     phi = communicator_robot.atan2(2 * (w * z + x * y), 1 - 2 * (pow(y, 2) + pow(z, 2)))
#     return phi

def quaternion2Euler(array):
    w = array['Pose']['Orientation']['W']
    x = array['Pose']['Orientation']['X']
    y = array['Pose']['Orientation']['Y']
    z = array['Pose']['Orientation']['Z']
    phi = communicator_robot.atan2(2 * (w * z + x * y), 1 - 2 * (pow(y, 2) + pow(z, 2)))
    return phi

def getData():


    MRDS_URL = sys.argv[1]#'localhost:50000'#robot.argv[1]
    x1 = sys.argv[2]#communicator_robot.argv[2]
    y1 = sys.argv[3]#communicator_robot.argv[3]
    x2 = sys.argv[4]#communicator_robot.argv[4]
    y2 = sys.argv[5]#communicator_robot.argv[5]
    showGUI = sys.argv[6]#communicator_robot.argv[6]

    if (x1 >= x2) or (y1 >= y2):
        print("Something is wrong here")
        print("Format: url, x1, y1, x2, y2,ShowGUI give the coordinates of the lower left and upper right corners of the area.")
        exit(1)
    return MRDS_URL, x1, y1, x2, y2, showGUI


def distance_2(x1, y1, x2, y2):
    return (math.sqrt(pow(x1-x2,2)+pow(y1-y2,2)))



def neighbours_ban(ban_list, range_var, x_ban, y_ban):
    for i in range(range_var):
        for j in range(range_var):
            ban_list.append((x_ban+i - 1,y_ban+j))
            ban_list.append((x_ban+i,y_ban+j - 1))
            ban_list.append((x_ban+i - 1, y_ban+j - 1))
            ban_list.append((x_ban+i + 1, y_ban+j))
            ban_list.append((x_ban+i,y_ban+j + 1))
            ban_list.append((x_ban+i + 1, y_ban+j + 1))
            ban_list.append((x_ban+i + 1, y_ban+j - 1))
            ban_list.append((x_ban+i - 1,y_ban+j + 1))
    return ban_list



def _floor(self, number):
    return math.floor(float(number)/self.resolution)*self.resolution

def _ceil(self, number):
    return math.ceil(float(number) / self.resolution) * self.resolution

def Grid_to_World(self, x, y):

    temp_x = int((self._floor(x * self.resolution)) + self.x_min)
    temp_y = int((self._ceil(y*self.resolution)) + self.y_min)

    return [temp_x, temp_y]
