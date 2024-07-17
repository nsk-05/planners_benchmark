import random
import math

class Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.cost = 0.0
class RRT:
    def __init__(self, max_iterations=10000, step_size=5, search_radius=10):
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.search_radius = search_radius
        self.path = []

    def distance(self, node1, node2):
        return math.sqrt((node1.position[0] - node2.position[0]) ** 2 + (node1.position[1] - node2.position[1]) ** 2)

    def is_collision_free(self, node1, node2):
        x1, y1 = node1.position
        x2, y2 = node2.position
        line_points = self.bresenham(x1, y1, x2, y2)
        for x, y in line_points:
            if self.grid[int(x)][int(y)] == 1:
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

    def get_nearby_nodes(self, new_node):
        nearby_nodes = []
        for node in self.nodes:
            if self.distance(node, new_node) <= self.search_radius:
                nearby_nodes.append(node)
        return nearby_nodes

    def rewire(self, new_node, nearby_nodes):
        for nearby_node in nearby_nodes:
            if (self.is_collision_free(new_node, nearby_node) )and (new_node.cost + self.distance(new_node, nearby_node) < nearby_node.cost):
                nearby_node.parent = new_node
                nearby_node.cost = new_node.cost + self.distance(new_node, nearby_node)

    def make_plan(self,grid, start, goal,is_generator):
        print("rrt plannrer invoked")
        self.grid = grid
        self.start = Node(tuple(start))
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
            if not self.is_collision_free(nearest_node, new_node):
                continue
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node) + (2*grid[new_position[0]][[new_position[1]]])
            nearby_nodes = self.get_nearby_nodes(new_node)
            self.rewire(new_node, nearby_nodes)
            self.nodes.append(new_node)
            node_list.append(new_node.position)
            fronterior_points.append(new_node.position)
            node_graph.append([new_node.position,new_node.parent.position])

            if (self.distance(new_node, self.goal) <= self.step_size) and (self.is_collision_free(new_node, self.goal)):
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + self.distance(new_node, self.goal)
                self.nodes.append(self.goal)
                self.path = self.extract_path()
                yield self.path, node_graph, fronterior_points
            
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