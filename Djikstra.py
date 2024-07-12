import heapq

class Cells():
    def __init__(self) -> None:
        # self.f : float = 
        self.g : float = float("inf")
        self.parent_i : int = 0
        self.parent_j : int = 0 
# Define the size of the grid
ROW = 0
COL = 0

def is_valid(row, col):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == 0

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

def make_plan(grid,start, goal):
    global ROW,COL
    ROW = len(grid)
    COL = len(grid[0])
    open_list=[]
    closed_list=[[False for _ in range(len(grid[0]))] for _ in range(len(grid))]
    cell_details=[[Cells() for _ in range(len(grid[0]))] for _ in range(len(grid))]
    i= start[0]
    j= start[1]
    path=[]
    neighbours=[(0,1),(0,-1),(1,0),(-1,0)]
    cell_details[i][j].parent_i=i
    cell_details[i][j].parent_j=j
    cell_details[i][j].f=0
    cell_details[i][j].g=0
    path_reached=False
    heapq.heappush(open_list,(0,i,j))
    while len(open_list)>0:
        p=heapq.heappop(open_list)
        i=p[1]
        j=p[2]
        closed_list[i][j]=True

        for dir in neighbours:
            new_i=i+dir[0]
            new_j=j+dir[1]
            if is_valid(new_i,new_j) and is_unblocked(grid,new_i,new_j) and not closed_list[new_i][new_j]:
                if(new_i==goal[0] and new_j==goal[1]):
                    row=goal[0]
                    col=goal[1]
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    path=[]
                    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
                        path.append((row,col))
                        temp_row=cell_details[row][col].parent_i
                        temp_col=cell_details[row][col].parent_j
                        row=temp_row
                        col=temp_col
                    path.append((row,col))
                    path.reverse()
                    found_dest=True
                    yield path,None,None
                else:
                    new_g=cell_details[i][j].g+1
                    if not closed_list[new_i][new_j] and cell_details[new_i][new_j].g>new_g:
                        heapq.heappush(open_list,(new_g,new_i,new_j))
                        cell_details[new_i][new_j].g=new_g
                        cell_details[new_i][new_j].parent_i=i
                        cell_details[new_i][new_j].parent_j=j
        explored_points=get_explored_points(closed_list)
        fronterior_points=get_fronterior_points(open_list)
        yield None, explored_points,fronterior_points
    yield None, explored_points,fronterior_points


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
    path=make_plan(grid, src, dest)
 
if __name__ == "__main__":
    main()