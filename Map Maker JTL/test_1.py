# import sys, time
# # from threading import Thread
# # from Cartographer import Cartographer
# # from communicator import Communicator
# # from Planner import Planner
# # from reactive import Reactive
# #
# #
import show_map
import distutils
import communicator_robot
#import planner_new
import planner_revised
import navigator
import time
import help_functions
from random import randint
#import help_functions
import Cartographer

resolution = 2

def _scanthearea(map, map_display):
    tick = 0

    while tick < 2:
        map.getMap()
        communicator_robot.Com.postSpeed(0.8 , 0.0)
        communicator_robot.time.sleep(0.5)
        communicator_robot.Com.postSpeed(0.0, 0.0)
        tick = tick + 0.1
        print("scanning the area")
        ###update the map and the map display
        position = communicator_robot.Com.getPosition()
        x = position['X']
        y = position['Y']
        x_r, y_r = map.World_to_Grid(x,y)
        map_display.updateMap(map.grid,15,x_r,y_r,(0,0))
    communicator_robot.Com.postSpeed(0.0,0.0)

def _init():
    ##showgui data here
    #communicator_robot.Com.__init__(url)
    ##end
    _null, x1, y1, x2, y2, showGUI = help_functions.getData()
    map = Cartographer.Cartographer()
    #map.initialize(-40,-40,30,30) ##TUTAJ INPUT WSZYSTKIE 4 WARTOSCI
    map.initialize(int(x1),int(y1),int(x2),int(y2))
    if showGUI == 'True':
        showGUI = True
    if showGUI == 'False':
        showGUI = False
    if showGUI == '0':
        showGUI = False
    if showGUI == '1':
        showGUI = True
    #print(type(showGUI))
    #print(showGUI)
    map_display = show_map.ShowMap(map.rows,map.columns,showGUI)


    tick = 0

    while tick < 2:
        map.getMap()
        communicator_robot.Com.postSpeed(0.8 , 0.0)
        communicator_robot.time.sleep(0.5)
        communicator_robot.Com.postSpeed(0.0, 0.0)
        tick = tick + 0.1
        ###update the map and the map display
        position = communicator_robot.Com.getPosition()
        x = position['X']
        y = position['Y']
        #z = position['Z']
        #print("istnieje",z)
        x_r, y_r = map.World_to_Grid(x,y)
        map_display.updateMap(map.grid,15,x_r,y_r,(0,0))
    communicator_robot.Com.postSpeed(0.0,0.0)
    return [map,map_display]


def main_loop(map, map_display):

    banned_list = []
    while True:
        centroids = []
        print("beep beep algorithm is running")
        position = communicator_robot.Com.getPosition()
        x = position['X']
        y = position['Y']
        print("Robot x",x,"Robot y",y)
        x_r, y_r = map.World_to_Grid(x, y)
        map.enlarge_obstacles()
        frontiers = map.get_frontiers(x_r, y_r)

        #print(frontiers)
        for frontier in frontiers:
            centroids.append(map.get_centroid_new(frontier))

        closest_centroid_x = -9999
        closest_centroid_y = -9999
        if len(centroids) > 1:
            for centroid in centroids:
                x, y = centroid
                if help_functions.distance_2(x,y,x_r,y_r) < help_functions.distance_2(closest_centroid_x,closest_centroid_y,x_r,y_r):
                    if (x,y) not in banned_list:
                        closest_centroid_x = x
                        closest_centroid_y = y
            banned_list.append((closest_centroid_x,closest_centroid_y))
            banned_list = help_functions.neighbours_ban(banned_list,2,closest_centroid_x,closest_centroid_y)
        if len(centroids) == 1:
            for centroid in centroids:
                closest_centroid_x,closest_centroid_y = centroid
                banned_list.append((closest_centroid_x, closest_centroid_y))
                banned_list = help_functions.neighbours_ban(banned_list, 2, closest_centroid_x, closest_centroid_y)
        if len(centroids) == 0:
            print("nothing left to explore")
            map_display.close()
            return True
        print("bann list below")
        print(banned_list)
        print("goind towards:")
        print(closest_centroid_x, closest_centroid_y)
        #planner = planner_new.PathPlanner()
        planner_2 = planner_revised.PathPlanner()
        #map.enlarge_obstacles()
        path = planner_2.astar(map,(x_r,y_r),(closest_centroid_x,closest_centroid_y))
        #path = planner.astar(map,(x_r,y_r),(x_goal, y_goal))
        goal = path[0]
        path.reverse()
        ##preparing path for deployment
        path_final = []
        temp = 0
        while (temp < len(path)-1):
            path_final.append(path[temp])
            temp = temp + 2
        if len(path) == 1:
            path_final.append(path)
        #print("The original path is:",path)
        print("The shortened path is: ",path_final)
        time.sleep(0.25)
        ###update map here
        position = communicator_robot.Com.getPosition()
        x = position['X']
        y = position['Y']
        x_r, y_r = map.World_to_Grid(x, y)

        map_display.updateMap_print_path(map.grid_copy, 15, x_r, y_r, path_final)
        time.sleep(2)

        for i in range(len(path_final)):
            goal_x, goal_y = (path_final[i])
            ##goal points are changed to global coordinates inside robot_to_goal function
        #robot_orientation = communicator_robot.Com.getPose()['Pose']['Orientation']
            test = navigator.robot_to_goal(map,path,goal_x,goal_y,map_display)
            position = communicator_robot.Com.getPosition()
            x = position['X']
            y = position['Y']
            x_r, y_r = map.World_to_Grid(x, y)
            map_display.updateMap(map.grid, 15, x_r, y_r, (goal_x, goal_y))
            print("Goal reached",goal_x,goal_y)
            communicator_robot.Com.postSpeed(0.0,0.0)
        #_scanthearea(map, map_display)

def frontier_test(map,map_display):
    centroids = []
    print("beep beep algorithm is running")
    position = communicator_robot.Com.getPosition()
    x = position['X']
    y = position['Y']
    print("Robot x", x, "Robot y", y)
    x_r, y_r = map.World_to_Grid(x, y)
    map.enlarge_obstacles()
    frontiers = map.get_frontiers(x_r,y_r)


    # for p in frontiers[0]:
    #     x,y = p
    #     print(x)
    #     print(y)

    for p in frontiers:
        centroids.append(map.get_centroid_new(p))
        print(p)
    #map_display.updateMap_print_frotniers(map.grid, 15, x_r, y_r, frontiers)
    print(centroids)
    #got (32 14) (64 33) (75 34)
    map_display.updateMap_print_frotniers(map.grid, 15, x_r, y_r, frontiers)
    input("ESSA")
    #map = show_map.ShowMap(80,80,1)
    #position = communicator_robot.Com.getPosition()
    #print(position)
    #x = position['X']h

    #y = position['Y']
    print('LOL')
    #print(y)
""""""
#testing
""""""
#
url = 0

(map, map_display) = _init()
a = main_loop(map, map_display)
