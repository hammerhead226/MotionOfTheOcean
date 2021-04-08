import numpy as np
from typing import Tuple
import math

COEFFICIENTS = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, .5, 0, 0, 0],
    [-10, -6, -1.5, 10, -4, .5],
    [15, 8, 1.5, -15, 7, -1],
    [-6, -3, -.5, 6, -3, .5]
])


class Spline:
    def __init__(self, waypoint_0: np.ndarray, waypoint_1: np.ndarray, radius: float = 0, module_offset: float = 0):
        """Create spline object"""

        self.waypoint_0 = waypoint_0
        self.waypoint_1 = waypoint_1
        self.r = radius
        self.module_offset = module_offset

        self.x_waypoint = np.array(
            [waypoint_0[0][0], waypoint_0[1][0], waypoint_0[2][0], waypoint_1[0][0], waypoint_1[1][0],
             waypoint_1[2][0]])
        self.y_waypoint = np.array(
            [waypoint_0[0][1], waypoint_0[1][1], waypoint_0[2][1], waypoint_1[0][1], waypoint_1[1][1],
             waypoint_1[2][1]])
        self.theta_waypoint = np.array(
            [waypoint_0[0][2] + module_offset, waypoint_0[1][2], waypoint_0[2][2], waypoint_1[0][2] + module_offset,
             waypoint_1[1][2],
             waypoint_1[2][2]])

    def position(self, t: float) -> Tuple:
        """Compute position given parameter"""

        # Trust the linear algebra
        temp = np.matmul(np.array([1, t, t ** 2, t ** 3, t ** 4, t ** 5]), COEFFICIENTS)

        theta = np.dot(self.theta_waypoint.T, temp)
        x = np.dot(self.x_waypoint.T, temp) + self.r * math.cos(theta)
        y = np.dot(self.y_waypoint.T, temp) + self.r * math.sin(theta)

        return x, y, theta

    def d_position(self, t: float) -> Tuple:
        """Compute first derivative of position given parameter"""

        # Trust the linear algebra
        temp = np.matmul(np.array([1, t, t ** 2, t ** 3, t ** 4, t ** 5]), COEFFICIENTS)
        d_temp = np.matmul(np.array([0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]), COEFFICIENTS)

        theta = np.dot(self.theta_waypoint.T, temp)
        d_theta = np.dot(self.theta_waypoint.T, d_temp)
        d_x = np.dot(self.x_waypoint.T, d_temp) - self.r * math.sin(theta) * d_theta
        d_y = np.dot(self.y_waypoint.T, d_temp) + self.r * math.cos(theta) * d_theta

        return d_x, d_y, d_theta

    def dd_position(self, t: float) -> Tuple:
        """Compute second derivative of position given parameter"""

        # Trust the linear algebra
        temp = np.matmul(np.array([1, t, t ** 2, t ** 3, t ** 4, t ** 5]), COEFFICIENTS)
        d_temp = np.matmul(np.array([0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]), COEFFICIENTS)
        dd_temp = np.matmul(np.array([0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3]), COEFFICIENTS)

        theta = np.dot(self.theta_waypoint.T, temp)
        d_theta = np.dot(self.theta_waypoint.T, d_temp)
        dd_theta = np.dot(self.theta_waypoint.T, dd_temp)
        dd_x = np.dot(self.x_waypoint.T, dd_temp) - self.r * (math.cos(theta) * d_theta ** 2 + math.sin(theta) * dd_theta)
        dd_y = np.dot(self.y_waypoint.T, dd_temp) + self.r * (-math.sin(theta) * d_theta ** 2 + math.cos(theta) * dd_theta)

        return dd_x, dd_y, dd_theta

    def curvature(self, t: float) -> float:
        """Compute curvature given parameter"""

        # Trust the calculus
        d_x, d_y, d_theta = self.d_position(t)
        dd_x, dd_y, dd_theta = self.dd_position(t)

        return (d_x * dd_y - d_y * dd_x) / (math.sqrt(d_x ** 2 + d_y ** 2) ** 3)