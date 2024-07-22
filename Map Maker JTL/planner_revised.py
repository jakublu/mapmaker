from heapq import heappush, heappop


class PathPlanner:
    """
    @ description = A* algorithm to generate the set of path points given the start and the goal points
    """

    # direction of the neighbour grids
    neighbour_direction_points = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    def __init__(self, max_depth=2500):
        self.__max_depth = max_depth

    def astar(self, map, start, goal):

        def heuristicDistance(n1, n2):
            return ((n2[0] - n1[0]) ** 2 + (n2[1] - n1[1]) ** 2)

        closed_nodes_set = set()
        came_from = {}
        gScore = {start: 0}
        fScore = {start: heuristicDistance(start, goal)}

        depth = 1
        oheap_ = []

        heappush(oheap_, (fScore[start], start))  # binary heap makes the list sorted

        while oheap_:

            current = heappop(oheap_)[1]
            if current == goal or heuristicDistance(current, goal) < 5:
                return self.construct_path(came_from, current)

            if depth > self.__max_depth:
                return self.construct_path(came_from, current)

            closed_nodes_set.add(current)
            for i, j in PathPlanner.neighbour_direction_points:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gScore[current] + heuristicDistance(current, neighbor)
                if 0 <= neighbor[0] < map.columns:
                    if 0 <= neighbor[1] < map.rows:
                        if map.grid_copy[neighbor[0]][neighbor[1]] > 12 or (
                                map.grid_copy[neighbor[0]][neighbor[1]] == 7):  # and map[neighbor[0]][neighbor[1]] <= 0.6):
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                if neighbor in closed_nodes_set and tentative_g_score >= gScore.get(neighbor, 0):
                    continue
                depth += 1

                # if the neighbor is not in opened nodes or its score is better, store the path and scores
                if tentative_g_score < gScore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap_]:
                    came_from[neighbor] = current
                    gScore[neighbor] = tentative_g_score
                    fScore[neighbor] = tentative_g_score + heuristicDistance(neighbor, goal)
                    heappush(oheap_, (fScore[neighbor], neighbor))

        return None

    def construct_path(self, came_from, current):  # function to define points
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path

