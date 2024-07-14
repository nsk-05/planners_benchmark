import pygame

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREY = (200, 200, 200)
ORANGE = (255, 165, 0)

def draw_grid(screen, grid, cell_size, width):
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            color=255*(1-grid[row, col])
            color=((color,color,color))
            # color = WHITE
            # if grid[row, col] == 1:
            #     color = BLACK
            pygame.draw.rect(screen, color, (col * cell_size, row * cell_size, cell_size, cell_size))

def draw_start_goal(screen, start, goal, cell_size):
    pygame.draw.circle(screen, GREEN, (start[1] * cell_size + cell_size // 2, start[0] * cell_size + cell_size // 2), cell_size // 2)
    pygame.draw.circle(screen, RED, (goal[1] * cell_size + cell_size // 2, goal[0] * cell_size + cell_size // 2), cell_size // 2)

def draw_path(screen, path, cell_size):
    if len(path) > 1:
        path_points = [(point[1] * cell_size + cell_size // 2, point[0] * cell_size + cell_size // 2) for point in path]
        pygame.draw.lines(screen, BLUE, False, path_points, 3)

def draw_fronteriors_points(screen, fronteriors_points, cell_size):
    if(len(fronteriors_points)>0):
        for row, col in fronteriors_points:
            pygame.draw.rect(screen, BLUE, (col * cell_size, row * cell_size, cell_size, cell_size))

def draw_explored_points(screen, explored_points, cell_size):
    for row, col in explored_points:
        pygame.draw.rect(screen, ORANGE, (col * cell_size, row * cell_size, cell_size, cell_size))

def draw_side_panel(screen, start_x, width, setting_start, setting_goal, setting_obstacle, clearing_obstacle,algorithm):
    pygame.draw.rect(screen, GREY, (start_x, 0, width, screen.get_height()))

    draw_button(screen, start_x + 10, 100, "Set Start", setting_start)
    draw_button(screen, start_x + 10, 150, "Set Goal", setting_goal)
    draw_button(screen, start_x + 10, 200, "Add Obstacle", setting_obstacle)
    draw_button(screen, start_x + 10, 250, "Clear Obstacle", clearing_obstacle)
    draw_button(screen, start_x + 10, 300, "Clear All", False)
    draw_button(screen, start_x + 10, 350, "A*", False,button_size=(50, 30), color= BLACK)
    draw_button(screen, start_x + 60, 350, "Dji", False,button_size=(50, 30), color= BLACK)
    draw_button(screen, start_x + 110, 350, "T*", False,button_size=(50, 30), color= BLACK)
    draw_button(screen, start_x + 160, 350, "RRT*", False,button_size=(50, 30), color= BLACK)


def draw_button(screen, x, y, text, active,button_size=(120, 30),color=RED):
    font = pygame.font.Font(None, 36)
    color = GREEN if active else color
    pygame.draw.rect(screen, color, (x, y, button_size[0],button_size[1]))
    text_surface = font.render(text, True, WHITE)
    screen.blit(text_surface, (x + 10, y + 5))
