import math
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid
from vtk.util.colors import beige

# directions: up, down, right, left
DIRECTIONS = [(0, 1), (0, -1), (1, 0), (-1, 0)]

class Node:
    def __init__(self, x, y, type=100):
        self.x = x
        self.y = y
        self.type = type

# convert OccupancyGrid message into a 2D (grid) of node objects
def process_grid(occupancy_grid_msg):
    width = occupancy_grid_msg.width
    height = occupancy_grid_msg.height

    occupancy_grid = np.array(occupancy_grid_msg.data).reshape((height, width))
    grid = []

    # fill up board with data
    for row_idx in range(height):
        row_nodes = []
        for col_idx in range(width):
            cell_type = occupancy_grid[row_idx, col_idx]
            node = Node(row_idx, col_idx, cell_type)
            row_nodes.append(node)
        grid.append(row_nodes)
    # returns a 2D list of node objects
    return grid


def add_obstacle_buffer(grid, buffer_size, start_node, end_node):
    rows = len(grid)
    cols = len(grid[0])
    count = 0

    for y in range(rows):
        for x in range(cols):
            print(f'Adding buffer zone: {(count / (len(grid) * len(grid[0]))) * 100:.2f} %', end='\r')
            count += 1
            if grid[y][x].type > 50:
                for dy in range(-buffer_size, buffer_size + 1):
                    for dx in range(-buffer_size, buffer_size + 1):
                        ny = y + dy
                        nx = x + dx
                        if 0 <= ny < rows and 0 <= nx < cols:
                            # prevent from overwriting start/end or already marked obstacles
                            if grid[ny][nx] != start_node and grid[ny][nx] != end_node and grid[ny][nx].type <= 50:
                                grid[ny][nx] = 51
    return grid

# returns Manhattan distance between node A and node B
def heuristic(node_a, node_b):
    return abs(node_a.x - node_b.x) + abs(node_a.y - node_b.y)

# constructs a path from the starting point using the end points trajectory route
def reconstruct_path(parents, current_node):
    path = []
    while current_node in parents:
        path.append(current_node)
        current_node = parents[current_node]
    path.append(current_node)
    path.reverse()
    return path

# search algorithm - takes in occupancy_grid msg, start and end cells, the buffer size around objects
def a_star(occupancy_grid_msg, start_coords, end_coords, buffer_zone=25):
    grid = process_grid(occupancy_grid_msg)
    start_node = grid[start_coords[1]][start_coords[0]]
    end_node = grid[end_coords[1]][end_coords[0]]

    grid = add_obstacle_buffer(grid, buffer_zone, start_node, end_node)

    rows = len(grid)
    cols = len(grid[0])

    # cost from start to some arbitrary n node
    g_scores = np.full((rows, cols), np.inf, dtype=float)
    g_scores[start_node.y, start_node.x] = 0.0

    # estimated cost of the cheapest path using the sum of g_score and heuristic
    f_scores = np.full((rows, cols), np.inf, dtype=float)
    f_scores[end_node.y, end_node.x] = heuristic(start_node, end_node)

    open_set = []
    heapq.heappush(open_set, (f_scores[start_node.y, start_node.x], (start_node.x, start_node.y)))

    parents = {}

    while open_set:
        _, (current_x, current_y) = heapq.heappop(open_set)
        current_node = grid[current_y][current_x]

        # the goal was found so return the path
        if current_node.x == end_node.x and current_node.y == end_node.y:
            final_path_node = reconstruct_path(parents, current_node)
            return [(node.x, node.y) for node in final_path_node]

        # explore neighbors
        for dir_x, dir_y in DIRECTIONS:
            neighbor_x = current_x + dir_x
            neighbor_y = current_y + dir_y

            if 0 <= neighbor_x < cols and 0 <= neighbor_y < rows:
                neighbor_node = grid[neighbor_y][neighbor_x]

                # continue if obstacle is encountered
                if neighbor_node.type > 50:
                    continue

                cost_g_score = g_scores[neighbor_y, neighbor_x] + 1.0

                # choose alternative pass
                if cost_g_score < g_scores[neighbor_y, neighbor_x]:
                    parents[neighbor_node] = current_node
                    g_scores[neighbor_y, neighbor_x] = cost_g_score
                    f_scores[neighbor_y, neighbor_x] = cost_g_score + heuristic(neighbor_node, end_node)
                    heapq.heappush(open_set, (f_scores[neighbor_y, neighbor_x], (neighbor_x, neighbor_y)))

    print("No path found!!")
    return None