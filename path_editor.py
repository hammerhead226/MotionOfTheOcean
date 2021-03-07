import sys
import csv
import math
import matplotlib.pyplot as plt
from matplotlib import collections
import numpy as np

load_file = sys.argv[1]
write_file = sys.argv[2]

coefficients = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, .5, 0, 0, 0],
        [-10, -6, -1.5, 10, -4, .5],
        [15, 8, 1.5, -15, 7, -1],
        [-6, -3, -.5, 6, -3, .5]
])

waypoints = []

selected_type = -1
selected_index = -1

param_coords = [[], [], []]
x = []
y = []

def position_on_path(waypoint_0, waypoint_1, t):
    x_waypoint = np.array(
        [waypoint_0[0][0], waypoint_0[1][0], waypoint_0[2][0], waypoint_1[0][0],
         waypoint_1[1][0],
         waypoint_1[2][0]])
    y_waypoint = np.array(
        [waypoint_0[0][1], waypoint_0[1][1], waypoint_0[2][1], waypoint_1[0][1],
         waypoint_1[1][1],
         waypoint_1[2][1]])

    temp = np.matmul(np.array([1, t, t ** 2, t ** 3, t ** 4, t ** 5]), coefficients)
    x = np.dot(x_waypoint.T, temp)
    y = np.dot(y_waypoint.T, temp)

    return x, y


def mouse_press(event):

    global selected_index
    global selected_type

    lim = plt.gca().get_ylim()
    epsilon = (lim[1] - lim[0]) / 50
    print("click")
    print(param_coords)
    for i in range(len(param_coords)):
        for j in range(len(param_coords[i])):
            d = math.hypot(param_coords[i][j][0] - event.xdata, param_coords[i][j][1] - event.ydata)
            if d < epsilon:
                selected_index = j
                selected_type = i
                return
    selected_index = -1
    selected_type = -1


def mouse_move(event):
    global selected_index
    global selected_type

    if event.inaxes is None:
        return

    if selected_type == 0:
        waypoints[selected_index][0][0] = event.xdata
        waypoints[selected_index][0][1] = event.ydata

    if selected_type == 1:
        waypoints[selected_index][1] = [event.xdata - waypoints[selected_index][0][0],
                                                        event.ydata - waypoints[selected_index][0][1]]
    if selected_type == 2:
        waypoints[selected_index][2] = [event.xdata - waypoints[selected_index][0][0],
                                        event.ydata - waypoints[selected_index][0][1]]

    if selected_type != -1:
        update()

def mouse_release(event):

    global selected_type
    global selected_index

    selected_type = -1
    selected_index = -1


def key_press(event):

    global x
    global y

    if event.key == 'n':
        waypoints.append([[event.xdata, event.ydata], [1, 0], [0, 0]])
        update()

    if event.key == 'w':
        file = open(write_file, 'w')
        for i in range(len(x)):
            file.write(str(x[i]) + ', ' + str(y[i]) + ', ' + str(0) + '\n')

def update():

    global param_coords
    global x
    global y

    x = []
    y = []
    for i in range(len(waypoints) - 1):
        for j in np.arange(0, 1, 0.01):
            x.append(position_on_path(waypoints[i], waypoints[i + 1], j)[0])
            y.append(position_on_path(waypoints[i], waypoints[i + 1], j)[1])
    paths.set_xdata(x)
    paths.set_ydata(y)

    v = []
    param_coords = [[], [], []]
    for waypoint in waypoints:
        param_coords[0].append([waypoint[0][0], waypoint[0][1]])
        v.append([(waypoint[0][0], waypoint[0][1]), (waypoint[0][0] + waypoint[1][0], waypoint[0][1] + waypoint[1][1])])
        param_coords[1].append([waypoint[0][0] + waypoint[1][0], waypoint[0][1] + waypoint[1][1]])
    param_plot.set_segments(v)
    plt.gcf().canvas.draw_idle()


with open(load_file) as recording:
    recording = list(csv.reader(recording, delimiter=','))
    recording = [list(map(float, state[0:3])) for state in recording]

    state_x = [state[0] for state in recording]
    state_y = [state[1] for state in recording]

    plt.plot(state_x, state_y)
    plt.gcf().canvas.mpl_connect('key_press_event', key_press)
    plt.gcf().canvas.mpl_connect('button_press_event', mouse_press)
    plt.gcf().canvas.mpl_connect('motion_notify_event', mouse_move)
    plt.gcf().canvas.mpl_connect('button_release_event', mouse_release)

    paths, = plt.plot([], [], 'k')
    param_plot = plt.gca().add_collection(collections.LineCollection([], colors='m'))
    param_coords = [[], [], []]
    plt.axis('equal')
    plt.show()