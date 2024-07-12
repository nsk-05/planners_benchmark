from collections import deque

def bfs_search(grid, start, goal):
    rows, cols = grid.shape
    queue = deque([start])
    came_from = {start: None}
    explored_points = set()

    while queue:
        current = queue.popleft()
        explored_points.add(current)

        if current == goal:
            path = reconstruct_path(came_from, current)
            yield path, explored_points

        for neighbor in get_neighbors(current, rows, cols):
            if grid[neighbor[0], neighbor[1]] == 1 or neighbor in came_from:
                continue

            queue.append(neighbor)
            came_from[neighbor] = current

        yield None, explored_points

    yield None, explored_points

def get_neighbors(node, rows, cols):
    neighbors = []
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
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
