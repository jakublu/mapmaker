import numpy as np
import help_functions
import communicator_robot
import math
from copy import copy


##function responsible for steering the robot towards a goal
def robot_to_goal(map, point_List, x_goal, y_goal, display):
    x_goal_grid = copy(x_goal)
    y_goal_grid = copy(y_goal)
    [x_goal,y_goal] =map.Grid_to_World(x_goal,y_goal)
    dist_diff = 50
    laser = communicator_robot.Com.getLaser()
    laser = laser['Echoes']
    min_angle_obstacle = np.argmin(laser)
    distance = 2 ##change so the obstacle avoidance never kicks in
    #distance = laser[min_angle_obstacle]
    #if (min_angle_obstacle > 45 and min_angle_obstacle <135):
    while dist_diff>3: ##helps to get rid of goal obsession
        dist_diff = distance_diff(x_goal,y_goal)
        if distance < 1:
            while distance<1:
                laser = communicator_robot.Com.getLaser()
                laser = laser['Echoes']
                min_angle_obstacle = np.argmin(laser)

                if (min_angle_obstacle >=135 and min_angle_obstacle < 225):
                    communicator_robot.Com.postSpeed(-0.2, 0.0)
                elif (min_angle_obstacle > 45 and min_angle_obstacle < 135):
                    communicator_robot.Com.postSpeed(0.2, 0.0)
                else:
                    communicator_robot.Com.postSpeed(0.0,0.7)
                    communicator_robot.time.sleep(0.5)
                    communicator_robot.Com.postSpeed(0.0,0.0)
                ##updatemaphere
                map.getMap()
                position = communicator_robot.Com.getPosition()
                # print(position)
                x = position['X']
                y = position['Y']
                x_r, y_r = map.World_to_Grid(x, y)
                display.updateMap(map.grid, 15, x_r, y_r, [x_goal_grid,y_goal_grid])
                ##endofupdatemaphere
                laser = communicator_robot.Com.getLaser()
                laser = laser['Echoes']
                min_angle_obstacle = np.argmin(laser)
                distance = laser[min_angle_obstacle]
        else:
            if distance >=1:


                #Difference_angle_to_goal, Max_distance_to_goal = initial_attributes(goalx, goaly)
                ##updatemaphere
                map.getMap()
                position = communicator_robot.Com.getPosition()
                # print(position)
                x = position['X']
                y = position['Y']
                z = position['Z']
                print("Z =",z)
                x_r, y_r = map.World_to_Grid(x, y)
                display.updateMap(map.grid, 15, x_r, y_r, [x_goal_grid, y_goal_grid])
                alpha = angle_diff(x_goal, y_goal)
                dist_diff = distance_diff(x_goal, y_goal)
                ##endofupdatemaphere
                alpha = angle_diff(x_goal, y_goal)
                dist_diff = distance_diff(x_goal, y_goal)
                print("angle and distance difference is",alpha,dist_diff)
                time_stop = abs(alpha) / 17.1
                print("time for rotation =",time_stop)
                if time_stop<0.09 or 21.052 - time_stop<0.09:
                    communicator_robot.time.sleep(0.2)
                if alpha > 0:
                    if alpha <= 180:
                        communicator_robot.Com.postSpeed(0.2, 0.0)
                        communicator_robot.time.sleep(time_stop)
                        communicator_robot.Com.postSpeed(0.0, 0.0)
                    if alpha > 180:
                        communicator_robot.Com.postSpeed(-0.2, 0.0)
                        communicator_robot.time.sleep(21.052 - time_stop)
                        communicator_robot.Com.postSpeed(0.0, 0.0)
                elif alpha < 0:
                    if alpha >=-180:
                        communicator_robot.Com.postSpeed(-0.2, 0.0)
                        communicator_robot.time.sleep(time_stop)
                        communicator_robot.Com.postSpeed(0.0, 0.0)
                    if alpha <-180:
                        communicator_robot.Com.postSpeed(0.2, 0.0)
                        communicator_robot.time.sleep(21.052 - time_stop)
                        communicator_robot.Com.postSpeed(0.0, 0.0)
                else:
                    communicator_robot.Com.Speed(0.0, 0.0)

                ##updatemaphere
                map.getMap()
                position = communicator_robot.Com.getPosition()
                # print(position)
                x = position['X']
                y = position['Y']
                x_r, y_r = map.World_to_Grid(x, y)
                display.updateMap(map.grid, 15, x_r, y_r, [x_goal_grid, y_goal_grid])
                ##endofupdatemaphere
                laser = communicator_robot.Com.getLaser()
                laser = laser['Echoes']
                min_angle_obstacle = np.argmin(laser)
                distance = laser[min_angle_obstacle]

            communicator_robot.Com.postSpeed(0.0, 0.5)
            communicator_robot.time.sleep(0.5)


    print(min_angle_obstacle)
    return min_angle_obstacle


def distance_closest_obstacle():
    #this function returns the distance to the closest obstalce detected by the laser scanner
    laser = communicator_robot.Com.getLaser()
    laser = laser['Echoes']
    min_angle_obstacle = np.argmin(laser)
    return(laser[min_angle_obstacle])


def distance_diff(goal_x, goal_y):
    #this function returns the distance between some points and the robot in local coordinates
    position = communicator_robot.Com.getPosition()
    x = position['X']
    y = position['Y']

    local_x = goal_x - x
    local_y = goal_y - y

    return (math.sqrt((local_x ** 2) + (local_y ** 2)))

def angle_diff(goal_x, goal_y):
    ##this function returns the angle difference between some points and the robot in local coordinates

    pose = communicator_robot.Com.getPose()
    x = pose['Pose']['Position']['X']
    y = pose['Pose']['Position']['Y']

    local_x = goal_x - x
    local_y = goal_y - y

    theta_goal = (180 / communicator_robot.pi) * communicator_robot.atan2(local_y, local_x)
    theta_robot = (180 / communicator_robot.pi) * help_functions.quaternion2Euler(pose)

    return(theta_goal-theta_robot)