import pygame
import sys

from cg_interfaces.srv import MoveCmd
from rclpy.node import Node

from .Maze import Maze
from .Robot import Robot
from .Utils.Finders import find


class Game(Node):
    def __init__(self, maze_initial_configuration, resolution=720):
        super().__init__("Culling_Games")
        self.srv = self.create_service(MoveCmd, 'move_command',
                                       self.handle_move_cmd)
        self.maze = Maze(maze_initial_configuration, resolution)
        self.running = False
        self.win = False
        self.robot = Robot(self.maze)

    def update(self):
        self.handle_input()
        if not self.win:
            self.maze.draw()

    def handle_move_cmd(self, request, response):
        direction = request.direction.lower()
        target_pos = find(self.maze.get_occupancy_grid(), 't')
        response.success, surroundings = self.robot.move(direction)
        if target_pos == self.robot.pos:
            self.win = True
            self.maze.win()
        response.left, response.down, response.up, response.right = surroundings
        response.robot_pos = self.robot.pos
        response.target_pos = target_pos
        return response

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_h:
                    direction = 'left'
                elif event.key == pygame.K_j:
                    direction = 'down'
                elif event.key == pygame.K_k:
                    direction = 'up'
                elif event.key == pygame.K_l:
                    direction = 'right'
                else:
                    return
                target_pos = find(self.maze.get_occupancy_grid(), 't')
                success, surroundings = self.robot.move(direction)
                if target_pos == self.robot.pos:
                    self.win = True
                    self.maze.win()
                if success:
                    print(f"Robo movido! Robo vê: {surroundings}")
                else:
                    print(f"O movimento falhou! Robo vê: {surroundings}")
        
    def run(self):
        pygame.init()
        self.running = True
        while self.running:
            self.update()
            pygame.display.flip()
        pygame.quit()
        sys.exit()
