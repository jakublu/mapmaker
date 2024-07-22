# a class used to scan, create and visualise the map

import math, threading, time, decimal, communicator_robot
from PIL import Image as img
import copy
from collections import deque


class Cartographer:

    def __init__(self):
        self.display = True
        self.fullness = 0
        ## settings for finding frontiers
        self.emptyLimit = 4
        # scale = 1 when 1 pix per 1 m2, 10 when 1 pix per dm2 etc
        self.scale = 2#10
        self.resolution = 0.5
        self.threshold_number_of_points = 15 #10 dziala ok

    def _floor(self, number):
        return math.floor(float(number)/self.resolution)*self.resolution

    def _ceil(self, number):
        return math.ceil(float(number) / self.resolution) * self.resolution

    # BTLR - borders, BT - X, LR - Y, com - communicator class instance
    def initialize(self, x_bottom, y_bottom, x_top, y_top):
        self.x_min = x_bottom #-10
        self.y_min = y_bottom #-10
        self.x_max = x_top #20
        self.y_max = y_top #20
        self.scale = 2
        self.grid_copy = 0

        self.columns = int((self._ceil(self.x_max)-self._floor((self.x_min)))/self.resolution)
        self.rows = int((self._ceil(self.y_max)-self._floor((self.y_min)))/self.resolution)
        #print (self.columns,self.rows)

        # + 2 for safety of not going beyond array borders, [0,0] in left, bottom corner; Y are top-bottom, X left-right
        self.map_x = int ((self.x_max - self.x_min ) / self.resolution)
        self.map_y = int ((self.y_max - self.y_min ) / self.resolution)

        # grid updated wqith HIMM method. Values between 0 and 15.
        self.grid = [[7 for i in range(0, self.rows)] for j in range(0, self.columns)]
        #self.grid = [[7 for i in range(0, self.Ny)] for j in range(0, self.Nx)]
        #print('wielkosc mapy jest rowna',len(self.grid))
        # grid with information about visited places. Values between 0 and 15. Every time it's visited it increase
        self.visited = [[0 for i in range(0, self.rows)] for j in range(0, self.columns)]
        #self.visited = [[0 for i in range(0, self.Ny)] for j in range(0, self.Nx)]
        self.map = img.new('L', (self.rows, self.columns))
        #self.map = img.new('L', (self.Ny, self.Nx))

    # function for transforming the coordinates from MRDS to map indexes

    def World_to_Grid(self, x ,y):

        temp_x = int((self._floor(x) - self.x_min)/self.resolution)
        temp_y = int((self._floor(y) - self.y_min)/self.resolution)

        return [temp_x, temp_y]

    # function for transforming the map indexes to coordinates from MRDS

    def Grid_to_World(self, x, y):

        temp_x = int((self._floor(x * self.resolution)) + self.x_min)
        temp_y = int((self._ceil(y*self.resolution)) + self.y_min)

        return [temp_x, temp_y]





    def getHeading(self):
        psih = self.COM.getBearing()
        psi = math.atan2(psih['Y'], psih['X'])
        return psi



    def getMap(self):
        # scan surrounding and updates a global map
        #HIMM
        # own pose
        pose = communicator_robot.Com.getPose()['Pose']['Position']
        x0 = pose['X']
        y0 = pose['Y']
        psi = communicator_robot.Com.getHeading()
        #print('psi wynosi')
        #print(psi)
       # # scanning
        laser = communicator_robot.Com.getLaser()['Echoes']# self.COM.getLaser()['Echoes']
        laserAngles = communicator_robot.Com.getLaserAngles()#self.COM.getLaserAngles()
        #print(laser)
        #print(laserAngles)
        # get the obstacle position and update a grid with that knowledge
        for angle, dist in zip(laserAngles, laser):

            ##experiment
            if dist > 39:
                continue
            #print psi * 57.2957795, angle*57.2957795
            #print(angle)
            #print(dist)
            #print(math.pi)
            angle += psi
            if angle > math.pi:
                angle -= 2*math.pi
            #print angle*57.2957795
            # from 90 to 130 degrees
            if abs(angle) > math.pi/2 and angle > 0:
                #print "1: ", angle * 57.2957795
                angle -= math.pi/2
                xd = x0 - dist * math.sin(angle)
                yd = y0 + dist * math.cos(angle)
                #print(xd, yd)
                #print('weszlo 1')
            #from 0 to 90
            elif abs(angle) <= math.pi/2 and angle >= 0:
                #print "2: ", angle * 57.2957795
                xd = x0 + dist * math.cos(angle)
                yd = y0 + dist * math.sin(angle)
                #print(xd, yd)
                #print('weszlo 2')
            #from -90 to 0
            elif abs(angle) <= math.pi/2 and angle < 0:
                #print "3: ", angle * 57.2957795
                angle  = abs(angle)
                #print(angle)
                xd = x0 + dist * math.cos(angle)
                yd = y0 - dist * math.sin(angle)
                #print(xd, yd)
                #print('weszlo 3')
            #from -130 to 0
            elif abs(angle) > math.pi/2 and angle < 0:
                #print "4: ", angle * 57.2957795
                angle = abs(angle) - math.pi/2
                xd = x0 - dist * math.sin(angle)
                yd = y0 - dist * math.cos(angle)
                #print('weszlo 4')
            #print('x i y do przypsianie wynisoi', xd, yd)
            [i0, j0] = self.World_to_Grid(x0, y0)
            [iD, jd] = self.World_to_Grid(xd, yd)
            #print x0, y0, i0, j0, xd, yd, iD, jd
            self.new_bresenham(i0, j0, iD, jd)
        # update the map once a while
        #time.sleep(0.2)



    def _empty(self, x, y):
        #print(x)
        #print(y)
        if self.grid[x][y] > 0:
            self.grid[x][y] -= 1
        if self.visited[x][y] < 15:
            self.fullness += 1
        self.visited[x][y] = min(15, self.visited[x][y]+1)

    
    def _occupied(self, x,y):
        self.grid[x][y] = min(15, self.grid[x][y] + 3)
        self.GRO_new(x,y)
        if self.visited[x][y] < 15:
            self.fullness += 1
        self.visited[x][y] = min(15, self.visited[x][y]+1)

    # updates grid along the line from robot to obstacle
    #xr, yr - robots ij cooridinates, xd, yd - the same for obstacle

    def new_bresenham(self, xr, yr, xd, yd):
        # https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        dx = xd-xr
        dy = yd-yr
        if dx < 0:
            xi = -1
        else:
            xi = 1
        dx = abs(dx)
        if dy < 0:
            yi = -1
        else:
            yi = 1
        dy = abs(dy)
        # first ones - not obstacles
        x = xr
        y = yr
        self._empty(x,y)
        # checking what should be a leading axis
        if dx > dy:
            ai = (dy-dx)*2
            bi = dy*2
            d = bi - dx
            while x != xd - xi:
                if d >= 0:
                    x += xi
                    y += yi
                    d += ai
                else:
                    d += bi
                    x += xi
                if x < 0 or x >= self.map_x or y < 0 or y >= self.map_y:
                    break
                self._empty(x,y)
        else:
            ai = (dx-dy)*2
            bi = dx*2
            d = bi - dy
            while y != yd - yi:
                if d >= 0:
                    x += xi
                    y += yi
                    d += ai
                else:
                    d += bi
                    y += yi
                if x < 0 or x >= self.map_x or y < 0 or y >= self.map_y:
                    break
                self._empty(x,y)
        if not ( xd < 0 or xd >= self.map_x or yd < 0 or yd >= self.map_y):
            self._occupied(xd, yd)
        for i in range(1, 10):
            self._empty(xr, yr)

    # perform Groth Rate Operator for HIMM algorithm


    def GRO_new(self, x, y):
        sum = self.grid[x][y]
        if x > 0:
            sum += 0.5*self.grid[x-1][y]
            if y > 0:
                sum += 0.5*self.grid[x - 1][y - 1]
            if y < self.map_y-1:
                sum += 0.5*self.grid[x-1][y+1]
        if x < self.map_x-1:
            sum += 0.5 * self.grid[x +1][y]
            if y > 0:
                sum += 0.5*self.grid[x + 1][y - 1]
            if y < self.map_y - 1:
                sum += 0.5*self.grid[x + 1][y + 1]
        if y > 0:
            sum += 0.5*self.grid[x][y-1]
        if y < self.map_y - 1:
            sum+= 0.5*self.grid[x][y+1]
        self.grid[x][y] = int(min(15, sum))


    # function for searching the closest sure space in the grid


    def enlarge_obstacles(self):
        self.grid_copy = copy.deepcopy(self.grid)
        #print("wartosc")
        #print(self.grid_copy[1][1])
        #print(type(self.grid_copy[1][1]))
        neighbours = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]
        neighbours_2 = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1], [0, 2],[0, -2],[1,2],
                        [1,-2],[-1,-2],[-1,2],[2,-2],[2,-1],[2,0],[1,2],[2,2],[-2,2],[-2,1],[-2,0],[-2,-1],[-2,-2]]
        for i in range(1, self.map_x-2):
            for j in range(1, self.map_y-2):
                a = self.grid_copy[i][j]
                #print(a)
                #print(type(a))
                if self.grid[i][j] >= 13:
                    for n in neighbours_2:
                        self.grid_copy[i+n[0]][j+n[1]] = self.grid[i][j]

    # function responsible for finding frontiers using Wavefront Frontier Detector
    def get_frontiers(self, robot_x, robot_y):
        frontiers = []
        queue_m = deque([])
        map_open = set([])
        map_closed = set([])
        frontier_open = set([])
        frontier_closed = set([])
        robot_position = (robot_x, robot_y)

        queue_m.append(robot_position)
        map_open.add(robot_position)

        while queue_m:
            point = queue_m.popleft()

            if point in map_closed:
                continue

            if self.check_frontier_point(point):
                queue_f = deque([])
                new_frontier = set([])

                queue_f.append(point)
                frontier_open.add(point)

                while queue_f:
                    q = queue_f.popleft()

                    if q in map_closed and q in frontier_closed:
                        continue

                    if self.check_frontier_point(q):
                        new_frontier.add(q)
                        for w in self.get_neighbours(q):
                            if (w not in frontier_open and w not in frontier_closed and w not in map_closed):
                                queue_f.append(w)
                                frontier_open.add(w)
                    frontier_closed.add(q)

                if len(new_frontier) > self.threshold_number_of_points:
                    frontiers.append(new_frontier)

                for point_1 in new_frontier:
                    map_closed.add((point_1))

            for v in self.get_neighbours(point):
                if (v not in map_open and v not in map_closed and self.check_open_neighbours(v)):
                    queue_m.append(v)
                    map_open.add(v)

            map_closed.add(point)

        #centroid = self.get_centroid_new(frontiers)

        return frontiers

    def check_frontier_point(self, point):
        x, y = point

        diff = 3 ##dla 3 dzialalo ok
        if abs(self.grid_copy[x][y] - 7) > diff:
            return False

        for poynt in self.get_neighbours(point):
            x, y = poynt
            #print('the values are')
            #print(x,y)
           # print("wartosc w polu")
            #print (self.grid_copy[x][y])
            if self.grid_copy[x][y] <= 4:
                #print("weszlo")
                return True

        return False

    def get_neighbours(self, point):
        a = 1
        b = 1

        x,y = point
        neighbours = set([])

        if x < self.columns-1 and y > 0:#
            neighbours.add((x + 1, y - 1))
        if x > 0 and y < self.rows-1:#
            neighbours.add((x - 1, y + 1))
        if y < self.rows-1:#
            neighbours.add((x, y + 1))
        if x < self.columns-1:#
            neighbours.add((x + 1, y))
        if x < self.columns-1 and y < self.rows-1:
            neighbours.add((x + 1, y + 1))
        if y > 0:#
            neighbours.add((x, y - 1))
        if x > 0:#
            neighbours.add((x - 1, y))
        if x > 0 and y > 0:#
            neighbours.add((x - 1, y - 1))



        return neighbours

    def check_open_neighbours(self, point):

        neighbours = self.get_neighbours(point)

        for poynt in neighbours:
            x, y = poynt
            if self.grid_copy[x][y] <= 4: #dla 2 dzialalo OK
                return True

        return False

    def get_centroid_new(self, frontier):

        x_c = 0
        y_c = 0

        for p in frontier:
            x, y = p
            x_c += x
            y_c += y

        x_c = x_c // len(frontier)
        y_c = y_c // len(frontier)

        return (x_c, y_c)