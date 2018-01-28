from utilities.vector_pursuit import VectorPursuit
import numpy as np
import math
from magicbot import MagicRobot


def position_delta_x_y(theta, speed, robot_x, robot_y, orientation):
    vx = speed * math.cos(theta)
    vy = speed * math.sin(theta)
    oriented_vx = vx * math.cos(orientation) + vy * math.sin(orientation)
    oriented_vy = -vx * math.sin(orientation) + vy * math.cos(orientation)

    robot_x += MagicRobot.control_loop_wait_time*oriented_vx
    robot_y += MagicRobot.control_loop_wait_time*oriented_vy
    return robot_x, robot_y


def test_single_waypoint_converge():
    orientation = 0  # rad

    robot_x = 0.0
    robot_y = 1.0
    waypoints = np.array([[0, 0, 0, 0], [100, 0, 0, 1]])

    pursuit = VectorPursuit()
    pursuit.set_motion_params(3.0, 4.5, -4.5)
    pursuit.set_waypoints(waypoints)
    last_speed = 0

    for i in range(200):
        theta, speed, next_seg, over = pursuit.get_output(np.array([robot_x, robot_y]), last_speed)
        robot_x, robot_y = position_delta_x_y(theta, speed,
                                              robot_x, robot_y, orientation)
        if over:
            break
        last_speed = speed

    assert robot_x > 2
    assert abs(robot_y) < 0.1


def test_multi_waypoint_converge():
    speed = 1  # m/s
    orientation = 0  # rad

    robot_x = 0.0
    robot_y = 1.0
    waypoints = np.array([[0, 0, 0, 0], [1, 0, 0, 1], [1, 100, 0, 1]])

    pursuit = VectorPursuit()
    pursuit.set_motion_params(3.0, 4.5, -4.5)
    pursuit.set_waypoints(waypoints)

    last_speed = 0

    for i in range(500):
        theta, speed, next_seg, over = pursuit.get_output(np.array([robot_x, robot_y]), last_speed)
        robot_x, robot_y = position_delta_x_y(theta, speed,
                                              robot_x, robot_y, orientation)
        if over:
            break
        last_speed = speed

    assert abs(robot_x-1) < 0.1
    assert robot_y > 2


def test_speed_control():
    speed = 1  # m/s
    orientation = 0  # rad

    robot_x = 0.0
    robot_y = 1.0
    waypoints = np.array([[0, 0, 0, 0], [1, 0, 0, 1], [1, 2, 0, 2]])

    pursuit = VectorPursuit()
    pursuit.set_motion_params(3.0, 4.5, -4.5)
    pursuit.set_waypoints(waypoints)

    last_speed = 0

    for i in range(500):
        theta, speed, next_seg, over = pursuit.get_output(np.array([robot_x, robot_y]), last_speed)
        robot_x, robot_y = position_delta_x_y(theta, speed,
                                              robot_x, robot_y, orientation)
        if over:
            break
        last_speed = speed

    assert abs(speed - 2) < 0.2
