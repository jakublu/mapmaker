from PIL import Image
import numpy as np
import time
import threading

"""
ShowMap creates a Gui for showing the progress of the created map and saves it to file every 5 second
Author Peter Hohnloser

Updated by Ola Ringdahl 2019-01-08 (fixed that the plot window hangs in Windows) 
"""


class ShowMap(object):
    def __init__(self, gridHeight, gridWidth, showGUI):
        """
        Constructor for ShowMap

        Args:
            param gridHeight the height of the grid (no. of rows)
            param gridWidth the width of the grid (no. of columns)
            param ShowGUI if true showing the map
        """
        import matplotlib
        if not showGUI:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        self.saveMapTime = 5.0
        self.mapName = 'map.png'
        self.first = True
        self.__robot_size = 6
        self.__path_size = 2
        self.__size = (gridHeight, gridWidth)

        # create a grayscale image
        data = np.ones(shape=self.__size)
        self.__image = Image.fromarray(data * 0.5 * 255)

        # remove the toolbar from plot
        plt.rcParams['toolbar'] = 'None'

        # using matplotlib to show an image in a subplot
        self.__fig, self.__ax = plt.subplots(1, 1)
        self.__fig.suptitle('Show Map')

        # remove the x and y tick in figure
        self.__ax.set_xticks([])
        self.__ax.set_yticks([])

        # Show image window
        self.__implot = self.__ax.imshow(self.__image)

        plt.show(block=False)
        self.__fig.canvas.draw()

        saveMap(self.__fig, self.mapName)
        self.start_time = time.time()

    def updateMap(self, grid, maxValue, robot_row, robot_col, goal):
        """
        Creates a new BufferedImage from a grid with integer values between 0 - maxVal,
        where 0 is black and maxVal is white, with a grey scale in between. Negative values are shown as gray.
        Call this Method after you have updated the grid.

        Args:
            param grid is the updated grid (numpy matrix or a two-dimensional array)
            param maxVal is the max value that is used in the grid
            param robot_row is the current position of the robot in grid row
            param robot_col is the current position of the robot in grid column
        """
        # print("Yes I am",goals[0])
        # convert grid to a numpy matrix
        grid = np.matrix(grid)
        # mapping the grid to an Image
        #print(self.__size[1])
        for col in range(0, self.__size[1]):
            for row in range(0, self.__size[0]):
                #print(grid[row,col])
                value = grid[row, col]
                # if value is <0 draw a gray pixel else mapping the value between 0 - 255
                # where 0 is black and 255 is white
                if value < 0:
                    # set pixel value to gray
                    self.__image.putpixel((col, row), 127)
                else:
                    # set pixel value
                    self.__image.putpixel((col, row), abs(value * 255 / maxValue - 255))

        # update the plot withe new image
        self.__ax.clear()
        self.__implot = self.__ax.imshow(self.__image)

        # remove the x and y tick in figure
        self.__ax.set_xticks([])
        self.__ax.set_yticks([])

        # plot the robot pose
        self.__ax.plot((robot_col), (robot_row), 'rs', markersize=self.__robot_size)
        self.__ax.plot((goal[1]), (goal[0]), 'gs', markersize=self.__robot_size)

        # draw new figure
        self.__fig.canvas.draw()

        # Start a time that saves the image ever n seconds
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.saveMapTime:
            self.t = threading.Thread(target=saveMap, args=(self.__fig, self.mapName,))
            self.t.start()
            self.start_time = time.time()

        # wait a bit while the figure is updated (avoids a freeze in Windows)
        import matplotlib.pyplot as plt
        plt.pause(0.01)

    def updateMap_print_path(self, grid, maxValue, robot_row, robot_col, path):
        """
        Creates a new BufferedImage from a grid with integer values between 0 - maxVal,
        where 0 is black and maxVal is white, with a grey scale in between. Negative values are shown as gray.
        Call this Method after you have updated the grid.

        Args:
            param grid is the updated grid (numpy matrix or a two-dimensional array)
            param maxVal is the max value that is used in the grid
            param robot_row is the current position of the robot in grid row
            param robot_col is the current position of the robot in grid column
        """
        print(path)
        # print("Yes I am",goals[0])
        # convert grid to a numpy matrix
        grid = np.matrix(grid)
        # mapping the grid to an Image
        #print(self.__size[1])
        for col in range(0, self.__size[1]):
            for row in range(0, self.__size[0]):
                #print(grid[row,col])
                value = grid[row, col]
                # if value is <0 draw a gray pixel else mapping the value between 0 - 255
                # where 0 is black and 255 is white
                if value < 0:
                    # set pixel value to gray
                    self.__image.putpixel((col, row), 127)
                else:
                    # set pixel value
                    self.__image.putpixel((col, row), abs(value * 255 / maxValue - 255))
                if (row,col) in path:
                    #print("dziala")
                    self.__image.putpixel((col, row), 0)


        # update the plot withe new image
        self.__ax.clear()
        self.__implot = self.__ax.imshow(self.__image)

        # remove the x and y tick in figure
        self.__ax.set_xticks([])
        self.__ax.set_yticks([])

        # plot the robot pose
        self.__ax.plot((robot_col), (robot_row), 'rs', markersize=self.__robot_size)
        self.__ax.plot((74), (19), 'gs', markersize=self.__robot_size)

        # draw new figure
        self.__fig.canvas.draw()

        # Start a time that saves the image ever n seconds
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.saveMapTime:
            self.t = threading.Thread(target=saveMap, args=(self.__fig, self.mapName,))
            self.t.start()
            self.start_time = time.time()

        # wait a bit while the figure is updated (avoids a freeze in Windows)
        import matplotlib.pyplot as plt
        plt.pause(0.01)

    def updateMap_print_frotniers(self, grid, maxValue, robot_row, robot_col, frontiers):

        """
        Creates a new BufferedImage from a grid with integer values between 0 - maxVal,
        where 0 is black and maxVal is white, with a grey scale in between. Negative values are shown as gray.
        Call this Method after you have updated the grid.

        Args:
            param grid is the updated grid (numpy matrix or a two-dimensional array)
            param maxVal is the max value that is used in the grid
            param robot_row is the current position of the robot in grid row
            param robot_col is the current position of the robot in grid column
        """
        print(frontiers)
        # print("Yes I am",goals[0])
        # convert grid to a numpy matrix
        grid = np.matrix(grid)
        # mapping the grid to an Image
        #print(self.__size[1])
        for col in range(0, self.__size[1]):
            for row in range(0, self.__size[0]):
                #print(grid[row,col])
                value = grid[row, col]
                # if value is <0 draw a gray pixel else mapping the value between 0 - 255
                # where 0 is black and 255 is white
                if value < 0:
                    # set pixel value to gray
                    self.__image.putpixel((col, row), 127)
                else:
                    # set pixel value
                    self.__image.putpixel((col, row), abs(value * 255 / maxValue - 255))
                # for point in frontiers:
                #     x, y = point
                #     print("dziala")
                #     self.__image.putpixel((x, y), 0)


        # update the plot withe new image
        self.__ax.clear()
        self.__implot = self.__ax.imshow(self.__image)

        # remove the x and y tick in figure
        self.__ax.set_xticks([])
        self.__ax.set_yticks([])

        # plot the robot pose

        self.__ax.plot((robot_col), (robot_row), 'rs', markersize=self.__robot_size)
        self.__ax.plot((13), (31), 'gs', markersize=self.__robot_size)
        self.__ax.plot((34), (76), 'gs', markersize=self.__robot_size)
        #self.__ax.plot((33), (64), 'rs', markersize=self.__robot_size)
        #self.__ax.plot((74), (19), 'gs', markersize=self.__robot_size)
        # for col in range(0, self.__size[1]):
        #     for row in range(0, self.__size[0]):
        #         if (row, col) in frontiers:
        #             print("dziala")
        #             self.__ax.plot((col), (row), 'gs', markersize=5)


        # draw new figure
        self.__fig.canvas.draw()

        # Start a time that saves the image ever n seconds
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.saveMapTime:
            self.t = threading.Thread(target=saveMap, args=(self.__fig, self.mapName,))
            self.t.start()
            self.start_time = time.time()

        # wait a bit while the figure is updated (avoids a freeze in Windows)
        import matplotlib.pyplot as plt
        plt.pause(0.01)

    def close(self):
        """ Saves the last image before closing the application """
        saveMap(self.__fig, self.mapName)
        import matplotlib.pyplot as plt
        plt.close()


def saveMap(fig, mapName):
    """ Saves the drawn Map to an Image """
    data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    img = Image.fromarray(data)
    img.convert('RGB').save(mapName, 'PNG')
