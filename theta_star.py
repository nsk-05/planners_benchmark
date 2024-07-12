import heapq
import numpy as np

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def get_fronterior_points(open_list):
    points=[]
    for i in open_list:
        points.append(i[1])
    return points

def make_plan(grid, start, goal):
    rows, cols = len(grid),len(grid[0])
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    came_from = {start: None}
    g_score = {start: 0}
    explored_points = set()

    def line_of_sight(s, s_prime):
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

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = reconstruct_path(came_from, current)
            yield path, None,None

        explored_points.add(current)

        for neighbor in get_neighbors(current):
            if grid[neighbor[0], neighbor[1]] == 1:
                continue

            tentative_g_score = g_score[current] + heuristic(current, neighbor)

            if neighbor in g_score:
                if tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    if line_of_sight(came_from[current], neighbor):
                        came_from[neighbor] = came_from[current]
                    else:
                        came_from[neighbor] = current
                    heapq.heappush(open_set, (g_score[neighbor] + heuristic(neighbor, goal), neighbor))
            else:
                g_score[neighbor] = tentative_g_score
                came_from[neighbor] = current
                heapq.heappush(open_set, (g_score[neighbor] + heuristic(neighbor, goal), neighbor))

        fronterior_points=get_fronterior_points(open_set)
        yield None, explored_points,fronterior_points

    yield None, explored_points,fronterior_points
