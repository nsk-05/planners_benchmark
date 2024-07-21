import random
import math

ROW = 0
COL = 0

class Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.cost = 0.0
class RRT:
    def __init__(self, max_iterations=10000, step_size=3,exploration_constant=0.9):
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.exploration_constant=exploration_constant
        self.path = []

    def distance(self, node1, node2):
        return math.sqrt((node1.position[0] - node2.position[0]) ** 2 + (node1.position[1] - node2.position[1]) ** 2)

    def is_valid(self,row, col):
        return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

    def is_collision_free(self, node1, node2,robot_radius):
        x1, y1 = node1.position
        x2, y2 = node2.position
        line_points = self.bresenham(x1, y1, x2, y2)
        for x, y in line_points:
            # if self.grid[int(x)][int(y)] == 1:
            if(not self._is_collision_free(self.grid,x,y,robot_radius)):
                return False
        return True
    
    def _is_collision_free(self,grid, row, col,robot_radius):

        for i in range(-robot_radius, robot_radius + 1):
            for j in range(-robot_radius, robot_radius + 1):
                check_row = row + i
                check_col = col + j
                # Check if the cell is within grid bounds
                if (self.is_valid(check_row,check_col)):
                    # If any cell within the robot's radius is an obstacle, return False
                    if grid[check_row][check_col] == 1:
                        return False
                else:
                    # If the cell is out of grid bounds, consider it as an obstacle
                    return False
        return True

    def bresenham(self, x1, y1, x2, y2):
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return points

    def get_random_node(self):
        while True:
            if(random.randint(0, self.grid.shape[1] - 1)>(self.grid.shape[1]*self.exploration_constant)):
                return self.goal
            x = random.randint(0, self.grid.shape[1] - 1)
            y = random.randint(0, self.grid.shape[0] - 1)
            if self.grid[y, x] == 0:
                return Node((y, x))

    def get_nearest_node(self, random_node):
        nearest_node = self.nodes[0]
        min_distance = self.distance(nearest_node, random_node)
        for node in self.nodes:
            dist = self.distance(node, random_node)
            if dist < min_distance:
                nearest_node = node
                min_distance = dist
        return nearest_node

    def make_plan(self,grid, start, goal,is_generator,robot_radius):
        global ROW,COL
        ROW=len(grid)
        COL=len(grid[0])
        print("rrt plannrer invoked")
        self.grid = grid
        self.start = Node(tuple(start))
        self.start.cost=-10
        self.goal = Node(tuple(goal))
        self.nodes = [self.start]
        node_list=[]
        fronterior_points=[]
        node_graph=[]

        for i in range(self.max_iterations):
            random_node = self.get_random_node()
            nearest_node = self.get_nearest_node(random_node)
            theta = math.atan2(random_node.position[0] - nearest_node.position[0], random_node.position[1] - nearest_node.position[1])
            new_position = (int(nearest_node.position[0] + self.step_size * math.sin(theta)),
                            int(nearest_node.position[1] + self.step_size * math.cos(theta)))

            if (new_position[0] < 0) or (new_position[0] >= self.grid.shape[0]) or (new_position[1] < 0) or (new_position[1] >= self.grid.shape[1]):
                continue

            new_node = Node(new_position)
            if not self.is_collision_free(nearest_node, new_node,robot_radius):
                continue
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node) + (2*grid[new_position[0]][[new_position[1]]])
            # nearby_nodes = self.get_nearby_nodes(new_node)
            self.nodes.append(new_node)
            node_list.append(new_node.position)
            fronterior_points.append(new_node.position)
            node_graph.append([new_node.position,new_node.parent.position])

            if (self.distance(new_node, self.goal) <= self.step_size) and (self.is_collision_free(new_node, self.goal,robot_radius)):
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + self.distance(new_node, self.goal)
                self.nodes.append(self.goal)
                self.path = self.extract_path()
                yield self.path, node_graph, fronterior_points
            node_graph=[]
            for i in self.nodes:
                if(i.parent!=None):
                    node_graph.append([i.position,i.parent.position])
            
            if(is_generator):      
               yield None, node_graph, fronterior_points
        if(is_generator):
            yield None, None, None

    def extract_path(self):
        path = []
        node = self.goal
        while node.parent is not None:
            path.append(node.position)
            node = node.parent
        path.append(self.start.position)
        path.reverse()
        return path