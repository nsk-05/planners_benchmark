import heapq
import numpy as np

# Define the size of the grid
ROW = 0
COL = 0
rows=0
cols=0
class Cells():
    def __init__(self) -> None:
        self.g: float = float("inf")
        self.parent_i: int = 0
        self.parent_j: int = 0

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def get_fronterior_points(open_list):
    points=[]
    for i in open_list:
        points.append(i[1])
    return points

def is_valid(row, col):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

def line_of_sight(grid,s, s_prime,robot_radius):
        x0, y0 = s
        x1, y1 = s_prime
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while (x0, y0) != (x1, y1):
            # if grid[x0, y0] == 1:
            #     return False
            if(not is_collision_free(grid,x0,y0,robot_radius)):
                return False
            
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return is_collision_free(grid,x0,y0,robot_radius) #grid[x0, y0] == 0

def is_collision_free(grid, row, col,robot_radius):

    for i in range(-robot_radius, robot_radius + 1):
        for j in range(-robot_radius, robot_radius + 1):
            check_row = row + i
            check_col = col + j
            # Check if the cell is within grid bounds
            if (is_valid(check_row,check_col)):
                # If any cell within the robot's radius is an obstacle, return False
                if grid[check_row][check_col] == 1:
                    return False
            else:
                return False
    return True

def get_neighbors(node):
        global ROW,COL
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            r, c = node[0] + dr, node[1] + dc
            if 0 <= r < ROW and 0 <= c < COL:
                neighbors.append((r, c))
        return neighbors

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        path.append(current)
        current = came_from[current]
        
    path.reverse()
    return path

def make_plan(grid, start, goal,is_generator,robot_radius):
    global ROW, COL
    ROW, COL = len(grid),len(grid[0])
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    came_from = {start: None}
    closed_list = [[False for _ in range(len(grid[0]))] for _ in range(len(grid))]
    cell_details = [[Cells() for _ in range(len(grid[0]))] for _ in range(len(grid))]
    cell_details[start[0]][start[1]].g=0
    cell_details[start[0]][start[1]].parent_i=start[0]
    cell_details[start[0]][start[1]].parent_j=start[1]
    closed_list[start[0]][start[1]]=True
    explored_points = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            row = goal[0]
            col = goal[1]
            path = []

            while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
                path.append((row, col))
                temp_row = cell_details[row][col].parent_i
                temp_col = cell_details[row][col].parent_j
                row = temp_row
                col = temp_col
            path.append((row, col))
            path.reverse()
            yield path, None, None
            
        explored_points.add(current)

        for neighbor in get_neighbors(current):
            if (not is_collision_free(grid,neighbor[0],neighbor[1],robot_radius)):
                continue
            tent_g_score=cell_details[current[0]][current[1]].g + (2*grid[current[0]][current[1]]) + heuristic(current, neighbor)
            
            if closed_list[neighbor[0]][neighbor[1]]: #neighbor in g_score:
                if tent_g_score < cell_details[neighbor[0]][neighbor[1]].g:
                    cell_details[neighbor[0]][neighbor[1]].g=tent_g_score
                    closed_list[current[0]][current[1]]=True
                    current_node_parent=(cell_details[current[0]][current[1]].parent_i,cell_details[current[0]][current[1]].parent_j)

                    if line_of_sight(grid,current_node_parent, neighbor,robot_radius):
                        cell_details[neighbor[0]][neighbor[1]].parent_i,cell_details[neighbor[0]][neighbor[1]].parent_j = current_node_parent
                    else:
                        cell_details[neighbor[0]][neighbor[1]].parent_i,cell_details[neighbor[0]][neighbor[1]].parent_j=current
                        # came_from[neighbor] = current
                    heapq.heappush(open_set, (cell_details[neighbor[0]][neighbor[1]].g + heuristic(neighbor, goal), neighbor))
            else:
                closed_list[neighbor[0]][neighbor[1]]=True
                cell_details[neighbor[0]][neighbor[1]].g=tent_g_score
                # came_from[neighbor] = current
                cell_details[neighbor[0]][neighbor[1]].parent_i,cell_details[neighbor[0]][neighbor[1]].parent_j=current
                heapq.heappush(open_set, (cell_details[neighbor[0]][neighbor[1]].g + heuristic(neighbor, goal), neighbor))

        if(is_generator):
            fronterior_points=get_fronterior_points(open_set)
            yield None, explored_points,fronterior_points
    if(is_generator):
        yield None, explored_points,fronterior_points
