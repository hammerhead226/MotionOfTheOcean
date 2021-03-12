import sys
import csv
import math
import ast
import matplotlib.pyplot as plt
from matplotlib import collections
import numpy as np
from os import path


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

try:
    file = open(path.splitext(write_file)[0] + "_splines" + path.splitext(write_file)[1])
    waypoints = ast.literal_eval(file.readline())
    slowpoints = ast.literal_eval(file.readline())
except:
    pass

selected_type = -1
selected_index = -1


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
    for i in range(len(param_coords)):
        for j in range(len(param_coords[i])):
            d = math.hypot(param_coords[i][j][0] - event.xdata, param_coords[i][j][1] - event.ydata)
            if d < epsilon:
                selected_index = j
                selected_type = i
                return
    for i in range(len(slowpoints)):
        d = math.hypot(slowpoints[i][0] - event.xdata, slowpoints[i][1] - event.ydata)
        if d < epsilon:
            selected_index = i
            selected_type = 'slow'
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

    if selected_type == 'slow':
        slowpoints[selected_index] = [event.xdata, event.ydata]

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

    if event.key == 'd':
        if selected_type not in [-1, 'slow']:
            del waypoints[selected_index]
            update()
        if selected_type == 'slow':
            del slowpoints[selected_index]
            update()

    if event.key == 'u':
        waypoints.append([[event.xdata, event.ydata], [0, 0], [0, 0]])
        update()

    if event.key == 'z':
        slowpoints.append([event.xdata, event.ydata])
        update()

    if event.key == 'w':
        file = open(write_file, 'w')

        command_positions = {}
        for command in commands:
            min_d = float('inf')
            index = None
            for i in range(len(x)):
                d = math.hypot(command[0] - x[i], command[1] - y[i])
                if d < min_d:
                    min_d = d
                    index = i
            if index in command_positions.keys():
                command_positions[index] = command_positions[index] + '#' + command[3]
            else:
                command_positions[index] = command[3]

        for i in range(len(x)):
            if i in command_positions.keys():
                file.write(str(x[i]) + ',' + str(y[i]) + ',' + str(0) + ',' + command_positions[i] + '\n')
            else:
                file.write(str(x[i]) + ',' + str(y[i]) + ',' + str(0) + ',\n')

        file.write("~\n")
        for slowpoint in slowpoints:
            file.write(str(slowpoint[0]) + ',' + str(slowpoint[1]) + '\n')

        file = open(path.splitext(write_file)[0] + "_splines" + path.splitext(write_file)[1], 'w')
        file.write(str(waypoints) + '\n')
        file.write(str(slowpoints) + '\n')
        file.close()


def update():
    global param_coords
    global x
    global y
    global angle_plot

    x = []
    y = []
    center_x = []
    center_y = []
    for i in range(len(waypoints) - 1):
        for j in np.arange(0, 1 + 0.005, 0.005):
            if j == 0:
                center_x.append(position_on_path(waypoints[i], waypoints[i + 1], j)[0])
                center_y.append(position_on_path(waypoints[i], waypoints[i + 1], j)[1])
            x.append(position_on_path(waypoints[i], waypoints[i + 1], j)[0])
            y.append(position_on_path(waypoints[i], waypoints[i + 1], j)[1])
    paths.set_xdata(x)
    paths.set_ydata(y)
    centers.set_xdata(center_x)
    centers.set_ydata(center_y)

    v = []
    param_coords = [[], [], []]
    for waypoint in waypoints:
        param_coords[0].append([waypoint[0][0], waypoint[0][1]])
        v.append([(waypoint[0][0], waypoint[0][1]), (waypoint[0][0] + waypoint[1][0], waypoint[0][1] + waypoint[1][1])])
        param_coords[1].append([waypoint[0][0] + waypoint[1][0], waypoint[0][1] + waypoint[1][1]])
    param_plot.set_segments(v)
    plt.gcf().canvas.draw_idle()

    slowx = []
    slowy = []
    for slow in slowpoints:
        slowx.append(slow[0])
        slowy.append(slow[1])
    slowplot.set_xdata(slowx)
    slowplot.set_ydata(slowy)


with open(load_file) as recording:
    recording = list(csv.reader(recording, delimiter=','))
    commands = []
    for state in recording:
        if state[3] != '':
            state[0] = float(state[0])
            state[1] = float(state[1])
            state[2] = float(state[2])
            commands.append(state)
    recording = [list(map(float, state[0:3])) for state in recording]

    state_x = [state[0] for state in recording]
    state_y = [state[1] for state in recording]

    plt.plot(state_x, state_y)
    plt.gcf().canvas.mpl_connect('key_press_event', key_press)
    plt.gcf().canvas.mpl_connect('button_press_event', mouse_press)
    plt.gcf().canvas.mpl_connect('motion_notify_event', mouse_move)
    plt.gcf().canvas.mpl_connect('button_release_event', mouse_release)

    paths, = plt.plot([], [], 'k')
    slowplot, = plt.plot([], [], 'ms')
    centers, = plt.plot([], [], 's')
    param_plot = plt.gca().add_collection(collections.LineCollection([], colors='m'))
    angle_plot = plt.gca().add_collection(collections.LineCollection([], colors='r'))

    plt.axis('equal')
    update()
    plt.show()
