import pygame
import numpy as np
from gui_utils import draw_grid, draw_start_goal, draw_path, draw_side_panel, draw_explored_points,draw_fronteriors_points
from astar import make_plan as a_star_search
from Djikstra import make_plan as dijkstra_search
from theta_star import make_plan as theta_star_search  # Import Theta* search function
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREY = (200, 200, 200)
ORANGE = (255, 165, 0)

class GridManager:
    def __init__(self, grid_size=(100, 100), cell_size=10):
        pygame.init()

        self.grid_size = grid_size
        self.cell_size = cell_size
        self.grid = np.zeros(grid_size, dtype=int)

        self.start = (0, 0)
        self.goal = (99, 99)
        self.path = []
        self.explored_points = set()
        self.fronteriors_points =set()

        self.screen_width = grid_size[1] * cell_size
        self.screen_height = grid_size[0] * cell_size
        self.side_panel_width = 200
        self.total_width = self.screen_width + self.side_panel_width

        self.screen = pygame.display.set_mode((self.total_width, self.screen_height))

        self.setting_start = False
        self.setting_goal = False
        self.setting_obstacle = False
        self.clearing_obstacle = False

        self.algorithm = "A*"
        self.mouse_pressed = False
        self.search_generator = None

    def run(self):
        running = True
        clock = pygame.time.Clock()

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.handle_mouse_button_down()
                if event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_pressed = False
                if event.type == pygame.MOUSEMOTION and self.mouse_pressed:
                    self.handle_mouse_motion()

            if self.search_generator:
                try:
                    self.path, self.explored_points, self.fronteriors_points = next(self.search_generator)
                    if self.path and self.path[-1] == self.goal:
                        self.explored_points=set()
                        self.fronteriors_points=set()
                        self.search_generator = None
                except StopIteration:
                    self.search_generator = None

            self.update_display()
            clock.tick(100)  # Control the speed of visualization

        pygame.quit()

    def handle_mouse_button_down(self):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        self.mouse_pressed = True

        if mouse_x < self.screen_width:
            col = mouse_x // self.cell_size
            row = mouse_y // self.cell_size

            if self.setting_obstacle:
                self.grid[row, col] = 1
            elif self.clearing_obstacle:
                self.grid[row, col] = 0
            elif self.setting_start:
                self.start = (row, col)
                self.setting_start = False
                self.start_search()
            elif self.setting_goal:
                self.goal = (row, col)
                self.setting_goal = False
                self.start_search()
        else:
            self.handle_side_panel_click(mouse_x, mouse_y)

    def handle_side_panel_click(self, mouse_x, mouse_y):
        if self.screen_width + 10 <= mouse_x <= self.screen_width + 130:
            if 100 <= mouse_y <= 130:
                self.setting_start = not self.setting_start
                self.setting_goal = False
                self.setting_obstacle = False
                self.clearing_obstacle = False
            elif 150 <= mouse_y <= 180:
                self.setting_goal = not self.setting_goal
                self.setting_start = False
                self.setting_obstacle = False
                self.clearing_obstacle = False
            elif 200 <= mouse_y <= 230:
                self.setting_obstacle = not self.setting_obstacle
                self.setting_start = False
                self.setting_goal = False
                self.clearing_obstacle = False
            elif 250 <= mouse_y <= 280:
                self.clearing_obstacle = not self.clearing_obstacle
                self.setting_start = False
                self.setting_goal = False
                self.setting_obstacle = False
            elif 300 <= mouse_y <= 330:
                self.grid.fill(0)
                self.setting_start = False
                self.setting_goal = False
                self.setting_obstacle = False
                self.clearing_obstacle = False
                self.start_search()
            elif 350 <= mouse_y <= 380:
                if(self.screen_width+10 <= mouse_x < self.screen_width+60):
                    self.algorithm = "A*"
                    self.start_search()
                elif self.screen_width+60 <= mouse_x < self.screen_width+110:
                    self.algorithm = "Dijkstra"
                    self.start_search()
                elif self.screen_width+110 <= mouse_x < self.screen_width+160:
                    self.algorithm = "Theta*"
                    self.start_search()

    def start_search(self):
        self.search_generator = None
        if self.algorithm == "A*":
            self.search_generator = a_star_search(self.grid, self.start, self.goal)
        elif self.algorithm == "Dijkstra":
            self.search_generator = dijkstra_search(self.grid, self.start, self.goal)
        elif self.algorithm == "Theta*":
            self.search_generator = theta_star_search(self.grid, self.start, self.goal)
        self.path = []
        self.explored_points = set()
        self.fronteriors_points=set()

    def handle_mouse_motion(self):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        if mouse_x < self.screen_width:
            col = mouse_x // self.cell_size
            row = mouse_y // self.cell_size
            if self.setting_obstacle:
                self.grid[row, col] = 1
            elif self.clearing_obstacle:
                self.grid[row, col] = 0

    def update_display(self):
        self.screen.fill(WHITE)
        draw_grid(self.screen, self.grid, self.cell_size, self.screen_width)
        draw_path(self.screen, self.path if self.path is not None else [], self.cell_size)
        draw_explored_points(self.screen, self.explored_points, self.cell_size)
        draw_fronteriors_points(self.screen, self.fronteriors_points, self.cell_size)
        draw_start_goal(self.screen, self.start, self.goal, self.cell_size)
        draw_side_panel(self.screen, self.screen_width, self.side_panel_width,
                        self.setting_start, self.setting_goal, self.setting_obstacle, self.clearing_obstacle,self.algorithm)

        pygame.display.flip()

if __name__ == "__main__":
    manager = GridManager()
    manager.run()