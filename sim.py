import time
import math
from threading import Thread

import pygame
import numpy as np
import random
import vis
import sys
from queue import Queue

from de2bot import DE2Bot, DE2Config
from sensor_info import SensorInfo

kill_sim_thread = False


def meas_prob(x, o, meas_angle):
    ddir = o - x[:2, :]
    dist = np.linalg.norm(ddir)
    ddir = ddir / dist

    abs_angle = x[2, 0] + meas_angle
    adir = np.asmatrix([[np.cos(abs_angle), np.sin(abs_angle)]]).T

    cos_th = ddir.T.dot(adir)
    cos = np.arccos(cos_th)
    # return np.exp(-35 * dist * cos[0, 0] * cos[0, 0])
    deg = 15
    rad = np.radians(deg)
    return 1 if np.abs(cos) < rad else 0


def sensor_model(x, obstacles, meas_angle):
    by_prox = sorted(obstacles, key=lambda o: np.linalg.norm(x[:2, :] - o))

    for obs in by_prox:
        prob = meas_prob(x, obs, meas_angle)
        if prob == 1:
            return np.linalg.norm(obs - x[:2, 0])
    return None


def wait():
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_f:
                return


# def main():
#     clock = pygame.time.Clock()
#
#     visualizer = vis.Visualizer()
#
#     fps = 60
#     dt = 1.0 / fps
#     t = 0.0
#     i = 0
#
#     robot = DE2Bot()
#
#     obstacles = [
#         np.asmatrix([[-1.5, -0.5]]).T,
#         np.asmatrix([[1.5, -0.5]]).T,
#     ]
#
#     angles_deg = [-144, -90, -44, -12, 12, 44, 90, 144]
#     angles_rad = [np.deg2rad(a) for a in angles_deg]
#
#     controller = Controller(robot.state.pose, angles_rad, DE2Config(), obstacles)
#
#     next_sensor = None
#
#     framerate = 60.
#
#     # sensor_update_time = 1. / 16.
#     sensor_update_time = 1. / 30.
#     last_sensor_update = pygame.time.get_ticks()
#
#     hits = []
#     while not visualizer.close:
#         visualizer.update_events()
#
#         pos = robot.state.pose
#
#         sense = None
#         if next_sensor:
#             a = angles_rad[next_sensor]
#             sense = sensor_model(pos, obstacles, a)
#
#             for b in angles_rad:
#                 if b != a:
#                     angle = b + pos[2, 0]
#                     hits.append((pos[0, 0] + 0.1 * np.cos(angle), pos[1, 0] + 0.1 * np.sin(angle)))
#
#             if sense is not None:
#                 angle = a + pos[2, 0]
#                 hits.append((pos[0, 0] + sense * np.cos(angle), pos[1, 0] + sense * np.sin(angle)))
#
#         can_sense = pygame.time.get_ticks() > (last_sensor_update + sensor_update_time * 1000)
#         if can_sense:
#             hits = []
#
#         encoder_noise = 0.10 * np.asmatrix(np.random.normal(size=(2, 1)))
#
#         controls, next_sensor = controller.update(
#             robot.left_right() + encoder_noise,
#             sense,
#             can_sense,
#             1. / framerate
#         )
#         controls_noise = 0.5 * np.asmatrix(np.random.normal(size=(2, 1))) * np.linalg.norm(controls)
#         robot.apply(controls, dt)
#
#         if can_sense and next_sensor:
#             last_sensor_update = pygame.time.get_ticks()
#
#         visualizer.draw(
#             robot.state.pose,
#             hits,
#             [(mat[0, 0], mat[1, 0]) for mat in obstacles])
#
#         # wait()
#         clock.tick(framerate)

OUT_OF_RANGE_READING = 5

MAX_LINEAR = 1
MAX_ANGULAR = 1

def simulation_thread(controls_queue, sensor_queue):
    global kill_sim_thread

    print("Started simulation thread")
    clock = pygame.time.Clock()
    visualizer = vis.Visualizer()

    fps = 60
    dt = 1.0 / fps

    robot = DE2Bot()

    obstacles = [
        np.asmatrix([[1.0, 0]]).T,
        # np.asmatrix([[1.5, -0.5]]).T,
    ]

    # angles_deg = [-144, -90, -44, -12, 12, 44, 90, 144]
    angles_deg = [90, 44, 12, -12, -44, -90, -144, 144]
    # angles_deg = [0]
    angles_rad = [np.deg2rad(a) for a in angles_deg]

    next_sensor = 0
    framerate = 60.

    sensor_update_time = 1. / 30.
    last_sensor_update = pygame.time.get_ticks()

    hits = []
    actual_hits = [OUT_OF_RANGE_READING for i in range(len(angles_rad))]

    controls = np.asmatrix([[0, 0]]).T
    while not visualizer.close and not kill_sim_thread:
        visualizer.update_events()

        if not controls_queue.empty():
            new_controls = controls_queue.get()
            controls = new_controls / 1000

        pos = robot.state.pose

        a = angles_rad[next_sensor]
        sense = sensor_model(pos, obstacles, a)

        # for b in angles_rad:
        #     if b != a:
        #         angle = b + pos[2, 0]
        #         hits.append((pos[0, 0] + 0.1 * np.cos(angle), pos[1, 0] + 0.1 * np.sin(angle)))

        if sense is not None:
            angle = a + pos[2, 0]
            hits.append((pos[0, 0] + sense * np.cos(angle), pos[1, 0] + sense * np.sin(angle)))
            actual_hits[next_sensor] = sense
        else:
            actual_hits[next_sensor] = OUT_OF_RANGE_READING

        can_sense = pygame.time.get_ticks() > (last_sensor_update + sensor_update_time * 1000)
        if can_sense:
            # Send off to sensor queue
            linear, angular = robot.state.twist[0, 0], robot.state.twist[1, 0]
            theta = robot.state.pose[2, 0]
            sensor_info = SensorInfo(actual_hits, theta, linear)
            if sensor_queue.full():
                sensor_queue.get_nowait()
            sensor_queue.put_nowait(sensor_info)

            next_sensor = (next_sensor + 1) % len(angles_rad)
            hits = []

        encoder_noise = 0 * np.asmatrix(np.random.normal(size=(2, 1)))
        # controls = np.asarray([[0.1, -0.1]]).T
        robot.apply(controls, dt)

        if can_sense:
            last_sensor_update = pygame.time.get_ticks()

        visualizer.draw(
            robot.state.pose,
            hits[-1:],
            [(mat[0, 0], mat[1, 0]) for mat in obstacles])

        # wait()
        clock.tick(framerate)


def test_sim():
    global kill_sim_thread

    buf_size = 1
    controls_q = Queue(buf_size)
    sensor_q = Queue(buf_size)

    sim_thread = Thread(target=simulation_thread, args=(controls_q, sensor_q))
    sim_thread.start()

    try:
        i = 0
        while True:
            if controls_q.full():
                controls_q.get_nowait()
            if i == 0:
                control = np.asmatrix([[0.2, 0.2]]).T
                controls_q.put_nowait(control)
                i = 1
            else:
                control = np.asmatrix([[-0.2, 0.2]]).T
                controls_q.put_nowait(control)
                i = 0
            time.sleep(0.5)
            # if not sensor_q.empty():
            #     item = sensor_q.get()
            #     print(' '*120, end="\r")
            #     print(item, end="\r")
            # time.sleep(0.05)
    except KeyboardInterrupt:
        kill_sim_thread = True
        time.sleep(0.1)
        sim_thread.join()


if __name__ == '__main__':
    test_sim()
