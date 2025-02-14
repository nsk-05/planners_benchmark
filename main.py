import pygame
import pygame_gui
import numpy as np
from gui_utils import draw_grid, draw_start_goal, draw_path, draw_side_panel, draw_explored_points, draw_fronteriors_points,update_algo
from Astar import make_plan as a_star_search
from Djikstra import make_plan as dijkstra_search
from Theta_star import make_plan as theta_star_search 
from HybridAStar.hybrid_a_star import make_plan as ha_star_search
from RRT import RRT
from RRT_star import RRT_star
rrt=RRT(max_iterations=10000,step_size=3,exploration_constant=0.8)
rrt_star=RRT_star(max_iterations=10000,step_size=5,search_radius=50,exploration_constant=0.98)
rrt_search=rrt.make_plan
rrt_star_search=rrt_star.make_plan

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREY = (200, 200, 200)
ORANGE = (255, 165, 0)

class GridManager:
    def __init__(self, grid_size=(200, 200), cell_size=5):
        pygame.init()
        pygame.display.set_caption("Planner visualizer")

        self.grid_size = grid_size
        self.cell_size = cell_size #int(1000 / grid_size[0]) #cell_size
        self.grid = np.zeros(grid_size, dtype=int)
        self.grid[0,:]=1
        self.grid[grid_size[1]-1,:]=1
        self.grid[:,0]=1
        self.grid[:,grid_size[1]-1]=1
        self.start = (1, 1)
        self.goal = (grid_size[0]-2,grid_size[1]-2)
        self.start_angle=0
        self.goal_angle=0
        self.path = []
        self.explored_points = set()
        self.fronteriors_points = set()
        self.inflation_radius=0
        self.robot_radius=1
        self.screen_width = grid_size[1] * cell_size
        self.screen_height = grid_size[0] * cell_size
        self.side_panel_width = 250
        self.total_width = self.screen_width + self.side_panel_width

        self.screen = pygame.display.set_mode((self.total_width, self.screen_height))
        

        self.setting_start = False
        self.setting_goal = False
        self.setting_obstacle = False
        self.clearing_obstacle = False
        self.visualize = True

        self.algorithm = "A*"
        self.is_node_graph=False
        self.mouse_pressed = False
        self.search_generator = None
        self.iteration=0

        # Initialize pygame GUI manager
        self.gui_manager = pygame_gui.UIManager((self.total_width, self.screen_height))

        # Initialize sliders
        self.inflation_slider = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((self.screen_width + 10, 435), (180, 20)),
                                                                       start_value=0,
                                                                       value_range=(0, 10),
                                                                       manager=self.gui_manager)

        self.robot_radius_slider = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((self.screen_width + 10, 475), (180, 20)),
                                                                       start_value=1,
                                                                       value_range=(1, 10),
                                                                       manager=self.gui_manager)
        
        self.robot_angle_slider = pygame_gui.elements.UIHorizontalSlider(relative_rect=pygame.Rect((self.screen_width + 10, 555), (180, 20)),
                                                                       start_value=0,
                                                                       value_range=(-180, 180),
                                                                       manager=self.gui_manager,visible=0)

        # Initialize cost map (0 for normal, higher values for higher costs)
        self.cost_map = [[0 for _ in range(grid_size[1])] for _ in range(grid_size[0])]

    def run(self):
        running = True
        clock = pygame.time.Clock()

        while running:
            time_delta = clock.tick(60)/1000.0  # Time in seconds since last loop
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.handle_mouse_button_down()
                if event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_pressed = False
                if event.type == pygame.MOUSEMOTION and self.mouse_pressed:
                    self.handle_mouse_motion()
                if event.type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                    if event.ui_element == self.inflation_slider:
                        self.inflation_radius = int(self.inflation_slider.get_current_value())
                    if event.ui_element == self.robot_radius_slider:
                        self.robot_radius = int(self.robot_radius_slider.get_current_value())
                    if event.ui_element == self.robot_angle_slider:
                        if(self.setting_start):
                            self.start_angle = int(self.robot_angle_slider.get_current_value())
                        if(self.setting_goal):
                            self.goal_angle = int(self.robot_angle_slider.get_current_value())
                
                self.gui_manager.process_events(event)

            if self.search_generator:
                try:
                    self.path, self.explored_points, self.fronteriors_points = next(self.search_generator)
                    if self.path :     
                        self.explored_points = set()
                        self.fronteriors_points = set()
                        self.search_generator = None
                except StopIteration:
                    self.search_generator = None

            self.gui_manager.update(time_delta)
            
            self.update_display()
            clock.tick(30)  # Control the speed of visualization

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
                self.setting_goal = False
                self.start_search()
            elif self.setting_goal:
                self.goal = (row, col)
                self.setting_start = False
                self.start_search()
        else:
            self.handle_side_panel_click(mouse_x, mouse_y)

    def handle_side_panel_click(self, mouse_x, mouse_y):
        if self.screen_width + 10 <= mouse_x <= self.screen_width + 180:
            if 100 <= mouse_y <= 130:
                self.setting_start = not self.setting_start
                self.robot_angle_slider.set_current_value(self.start_angle)
                self.setting_goal = False
                self.setting_obstacle = False
                self.clearing_obstacle = False
            elif 150 <= mouse_y <= 180:
                self.setting_goal = not self.setting_goal
                self.robot_angle_slider.set_current_value(self.goal_angle)
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
                self.path = []
                self.explored_points = set()
                self.fronteriors_points = set()
            elif 350 <= mouse_y <= 380:
                if self.screen_width + 10 <= mouse_x < self.screen_width + 60:
                    self.algorithm = "A*"
                    self.is_node_graph=False
                    self.start_search()
                elif self.screen_width + 60 <= mouse_x < self.screen_width + 110:
                    self.algorithm = "Dijkstra"
                    self.is_node_graph=False
                    self.start_search()
                elif self.screen_width + 110 <= mouse_x < self.screen_width + 160:
                    self.algorithm = "Theta*"
                    self.is_node_graph=False
                    self.start_search()
                elif self.screen_width + 160 <= mouse_x < self.screen_width + 230:
                    print("Selecting RRT")
                    self.algorithm = "RRT"
                    self.is_node_graph=True
                    self.start_search()
            elif 385 <= mouse_y <= 415:
                if self.screen_width + 10 <= mouse_x < self.screen_width + 90:
                    print("Selecting RRT*")
                    self.algorithm = "RRT*"
                    self.is_node_graph=True
                    self.start_search()
                elif self.screen_width + 90 <= mouse_x < self.screen_width + 150:
                    print("Selecting HA*")
                    self.algorithm = "HA*"
                    self.is_node_graph=False
                    self.start_search()

            elif 490 <= mouse_y <= 520:
                self.visualize = not self.visualize
                print("changing visualization")

    def start_search(self):
        self.search_generator = None
        is_generator=self.visualize
        if self.algorithm == "A*":
            self.search_generator = a_star_search(np.array(self.cost_map), self.start, self.goal,is_generator,self.robot_radius)
        elif self.algorithm == "Dijkstra":
            self.search_generator = dijkstra_search(np.array(self.cost_map), self.start, self.goal,is_generator,self.robot_radius)
        elif self.algorithm == "Theta*":
            self.search_generator = theta_star_search(np.array(self.cost_map), self.start, self.goal,is_generator,self.robot_radius)
        elif self.algorithm == "RRT":
            self.search_generator = rrt_search(self.grid, self.start, self.goal,is_generator,self.robot_radius)
        elif self.algorithm == "HA*":
            self.search_generator = ha_star_search(self.grid, [self.start[0],self.start[1],self.start_angle], [self.goal[0],self.goal[1],self.goal_angle],is_generator,self.robot_radius)
        elif self.algorithm == "RRT*":
            self.search_generator = rrt_star_search(self.grid, self.start, self.goal,is_generator,self.robot_radius)
           
        if(not is_generator):
            self.path, self.explored_points, self.fronteriors_points = next(self.search_generator)
            if self.path :
                print(self.path[-1],self.goal)
                print("found the path so closing")
                self.search_generator = None
        else:
            self.path = []
        self.explored_points = set()
        self.fronteriors_points = set()

    def handle_mouse_motion(self):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        if mouse_x < self.screen_width:
            col = mouse_x // self.cell_size
            row = mouse_y // self.cell_size
            if self.setting_obstacle:
                self.grid[row, col] = 1
            elif self.clearing_obstacle:
                self.grid[row, col] = 0

    def update_gui(self, events):
        print("update gui added")
        for event in events:
            if event.type == pygame.USEREVENT:
                if event.user_type == pygame_gui.UI_HORIZONTAL_SLIDER_MOVED:
                    if event.ui_element == self.inflation_slider:
                        inflation_radius = int(self.inflation_slider.get_current_value())
                        self.update_cost_map(inflation_radius)
            self.gui_manager.process_events(event)

    def update_cost_map(self, inflation_radius):
        # Reset cost map
        self.cost_map = [[0 for _ in range(self.grid.shape[1])] for _ in range(self.grid.shape[0])]

        # Apply inflation to obstacles
        for row in range(self.grid.shape[0]):
            for col in range(self.grid.shape[1]):
                if self.grid[row, col] == 1:
                    self.inflate_obstacle(row, col, inflation_radius+1)
                    self.cost_map[row][col]=1
        # print(self.cost_map)

    def inflate_obstacle(self, row, col, inflation_radius):
        for dr in range(-inflation_radius, inflation_radius + 1):
            for dc in range(-inflation_radius, inflation_radius + 1):
                r = row + dr
                c = col + dc
                if 0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]:
                    distance = max(abs(dr), abs(dc))
                    added_cost = max(0, inflation_radius - distance) * 0.1
                    if(self.cost_map[r][c]==1):
                        continue
                    else:
                        if (self.cost_map[r][c] < added_cost):
                            self.cost_map[r][c] = added_cost

    def draw_slider(self):
        # self.gui_manager.update(0)
        self.gui_manager.draw_ui(self.screen)
        font = pygame.font.Font(None, 24)
        slider_label = font.render(f'Inflation Radius: {self.inflation_slider.get_current_value()}', True, BLACK)
        self.screen.blit(slider_label, (self.screen_width + 10, 420))
        slider_label = font.render(f'Robot Radius: {self.robot_radius_slider.get_current_value()}', True, BLACK)
        self.screen.blit(slider_label, (self.screen_width + 10, 460))
        if(self.setting_goal or self.setting_start):
            self.robot_angle_slider.show()
            slider_label = font.render(f'Robot angle: {self.robot_angle_slider.get_current_value()}', True, BLACK)
            self.screen.blit(slider_label, (self.screen_width + 10, 540))
        else:
            self.robot_angle_slider.hide()

    def update_display(self):
        self.screen.fill(WHITE)
        self.update_cost_map(self.inflation_radius)
        update_algo(self.algorithm)
        draw_grid(self.screen,np.array(self.cost_map), self.cell_size, self.screen_width)
        draw_path(self.screen, self.path if self.path is not None else [], self.cell_size)
        draw_explored_points(self.is_node_graph,self.screen, self.explored_points, self.cell_size)
        draw_fronteriors_points(self.screen, self.fronteriors_points, self.cell_size)
        draw_start_goal(self.screen, self.start, self.goal, self.cell_size)
        draw_side_panel(self.screen, self.screen_width, self.side_panel_width,
                        self.setting_start, self.setting_goal, self.setting_obstacle, self.clearing_obstacle, self.visualize)
        self.draw_slider()
        pygame.display.flip()

if __name__ == "__main__":
    manager = GridManager()
    manager.run()
