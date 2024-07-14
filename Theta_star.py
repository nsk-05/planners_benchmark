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

def line_of_sight(grid,s, s_prime):
        x0, y0 = s
        x1, y1 = s_prime
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while (x0, y0) != (x1, y1):
            if grid[x0, y0] == 1:
                return False
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return grid[x0, y0] == 0

def get_neighbors(node):
        global rows,cols
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            r, c = node[0] + dr, node[1] + dc
            if 0 <= r < rows and 0 <= c < cols:
                neighbors.append((r, c))
        return neighbors

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        path.append(current)
        current = came_from[current]
        
    path.reverse()
    return path
def make_plan(grid, start, goal):
    global rows, cols
    rows, cols = len(grid),len(grid[0])
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    came_from = {start: None}
    closed_list = [[False for _ in range(len(grid[0]))] for _ in range(len(grid))]
    cell_details = [[Cells() for _ in range(len(grid[0]))] for _ in range(len(grid))]
    cell_details[start[0]][start[1]].g=0
    cell_details[start[0]][start[1]].parent_i=start[0]
    cell_details[start[0]][start[1]].parent_j=start[1]
    closed_list[start[0]][start[1]]=True
    # g_score = {start: 0}
    explored_points = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # path = reconstruct_path(came_from, current)
            # yield path, None,None
            row = goal[0]
            col = goal[1]
            # cell_details[current[0]][current[1]].parent_i = current[0]
            # cell_details[current[0]][current[1]].parent_j = current[1]
            path = []

            while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
                path.append((row, col))
                temp_row = cell_details[row][col].parent_i
                temp_col = cell_details[row][col].parent_j
                row = temp_row
                col = temp_col

            print(path)
            path.append((row, col))
            path.reverse()
            yield path, None, None

        explored_points.add(current)

        for neighbor in get_neighbors(current):
            if grid[neighbor[0], neighbor[1]] == 1:
                continue
                
            tent_g_score=cell_details[current[0]][current[1]].g + (2*grid[current[0]][current[1]]) + heuristic(current, neighbor)
            
            if closed_list[neighbor[0]][neighbor[1]]: #neighbor in g_score:
                if tent_g_score < cell_details[neighbor[0]][neighbor[1]].g:
                    cell_details[neighbor[0]][neighbor[1]].g=tent_g_score
                    closed_list[current[0]][current[1]]=True
                    current_node_parent=(cell_details[current[0]][current[1]].parent_i,cell_details[current[0]][current[1]].parent_j)
                    # if line_of_sight(grid,came_from[current], neighbor):
                    #     came_from[neighbor] = came_from[current]

                    if line_of_sight(grid,current_node_parent, neighbor):
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

        fronterior_points=get_fronterior_points(open_set)
        yield None, explored_points,fronterior_points

    yield None, explored_points,fronterior_points
