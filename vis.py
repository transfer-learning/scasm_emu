import pygame
import numpy as np
import math


class Visualizer:
    def __init__(self):
        self.resolution = (800, 600)
        self.display = pygame.display.set_mode(self.resolution)
        pygame.display.set_caption("TL-DE2SIM")
        self.close = False
        self.keys = set([])
        self.ppm = 200
        # self.radius = 0.09
        self.radius = 0.13

    def update_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.close = True

    def draw(self, robot_coords, ping_coords, obs_coords):
        """
        Visualize things
        :param robot_coords:
        :param ping_coords:
        :param obs_coords:
        :return:
        """
        screen_coords = self.to_screen_coords(robot_coords)

        heading = robot_coords[2]
        line_end_coords = (
            screen_coords[0] + int(self.ppm * self.radius * 4 * np.cos(heading)),
            screen_coords[1] - int(self.ppm * self.radius * 4 * np.sin(heading))
        )
        radius = int(self.ppm * self.radius)

        self.display.fill((0, 0, 0))
        pygame.draw.circle(self.display, (255, 0, 0), screen_coords, radius)
        pygame.draw.aaline(self.display, (255, 0, 0),
                           screen_coords, line_end_coords, 10)

        for ping in ping_coords:
            ping_pos = self.to_screen_coords(ping)
            robot_screen_coords = self.to_screen_coords(robot_coords[:, 0])

            dx = ping[0] - robot_coords[0, 0]
            dy = ping[1] - robot_coords[1, 0]

            line_angle = math.atan2(dy, dx)

            dy = ping_pos[0] - robot_screen_coords[0]
            dx = ping_pos[1] - robot_screen_coords[1]

            line_length = math.hypot(dx, dy)

            start_angle = line_angle - 10 * np.pi / 180
            end_angle = line_angle + 10 * np.pi / 180

            rect = pygame.Rect(robot_screen_coords[0] - line_length, robot_screen_coords[1] - line_length,
                               line_length * 2, line_length * 2)
            # pygame.draw.rect(self.display, (255, 255, 255), rect, 1)
            pygame.draw.arc(self.display, (255, 255, 255), rect, start_angle, end_angle)
            pygame.draw.aaline(self.display, (255, 255, 255),
                               screen_coords, self.to_screen_coords(ping), 10)

        for obs in obs_coords:
            pygame.draw.circle(self.display, (0, 0, 255),
                               self.to_screen_coords(obs), 10)

        pygame.display.update()

    def to_screen_coords(self, pos):
        return (int(pos[0] * self.ppm + self.resolution[0] / 2),
                int(self.resolution[1] / 2 - pos[1] * self.ppm))
