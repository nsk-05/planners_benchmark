import heapq

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell (g + h)
        self.g = float('inf')  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination
 
# Define the size of the grid
ROW = 0
COL = 0

def is_valid(row, col):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == 0

def show_cell_cost(cell_details):
    for i in cell_details:
        for j in i:
            print(j.f,end=" ")
        print()

# Trace the path from source to destination
def trace_path(cell_details, dest):
    path = []
    row = dest[0]
    col = dest[1]
 
    # Trace the path from destination to source using parent cells
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col
 
    # Add the source cell to the path
    path.append((row, col))
    # Reverse the path to get the path from source to destination
    path.reverse()

    return path

def get_explored_points(closed_list):
    points=[]
    for i in range(len(closed_list)):
        for j in range(len(closed_list[0])):
            if(closed_list[i][j]):
                points.append((i,j)) 
    return points

def get_fronterior_points(open_list):
    points=[]
    for i in open_list:
        # print(i)
        points.append((i[1],i[2]))
    return points

def make_plan(grid, start, goal):
    # create a opne list
    global ROW,COL
    open_list=[]
    ROW=len(grid)
    COL=len(grid[0])
    # closed_list=[]
    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]
    neighbours=[(0,1),(0,-1),(1,0),(-1,0)]
    i=start[0]
    j=start[1]
    cell_details[i][j].f=0
    cell_details[i][j].g=0
    cell_details[i][j].h=0
    cell_details[i][j].parent_i=i
    cell_details[i][j].parent_j=j

    #intialize open list
    heapq.heappush(open_list,(0.0,i,j))

    #reached dest flag
    found_dest=False

    while len(open_list)>0:
        p=heapq.heappop(open_list)

        #mark the cell as visited
        i=p[1]
        j=p[2]
        closed_list[i][j]=True

        for dir in neighbours:
            new_i=i+dir[0]
            new_j=j+dir[1]

            if is_valid(new_i,new_j) and is_unblocked(grid,new_i,new_j) and not closed_list[new_i][new_j]:
                if new_i==goal[0] and new_j==goal[1]:
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    path=trace_path(cell_details,goal)
                    # show_cell_cost(cell_details)
                    found_dest = True
                    yield path, None, None
                else:
                    g_new=cell_details[i][j].g+1.0
                    h_new=heuristics((new_i,new_j),goal)
                    f_new=g_new+h_new

                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f>f_new:
                        heapq.heappush(open_list,(f_new,new_i,new_j))

                        cell_details[new_i][new_j].f=f_new
                        cell_details[new_i][new_j].g=g_new
                        cell_details[new_i][new_j].h=h_new
                        cell_details[new_i][new_j].parent_i=i
                        cell_details[new_i][new_j].parent_j=j

            explored_points=get_explored_points(closed_list)
            fronterior_points=get_fronterior_points(open_list)
            # print(fronterior_points)
        yield None, explored_points, fronterior_points
    yield None, explored_points, fronterior_points


    # while True:

    #     pass
def heuristics(a,b):
    h = abs(a[0]-b[0]) + abs(a[1]-b[1])
    return h

def main():
    # Define the grid (1 for unblocked, 0 for blocked)
    grid = [
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
        [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
        [1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
        [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 0, 0, 1]
    ]
 
    # Define the source and destination
    src = [8, 0]
    dest = [0, 0]
 
    # Run the A* search algorithm
    make_plan(grid, src, dest)
 
if __name__ == "__main__":
    main()