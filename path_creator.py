import math
import numpy as np
import ast
import sys
import csv
from matplotlib import pyplot as plt
from matplotlib import collections
from splines import Spline


def mouse_press(event):
    """Handle mouse press events"""

    global mouse_selected_index
    global mouse_selected_type

    # Exit if mouse event occurs out of plot
    if event.inaxes is None:
        return

    # Compute maximum distance threshold based on plot size
    lim = plt.gca().get_ylim()
    epsilon = (lim[1] - lim[0]) / 50

    # Check if mouse event is within defined threshold to parameter points
    for i, param_type in enumerate(param_locations):
        for j, param_location in enumerate(param_type):
            d = math.hypot(param_location[0] - event.xdata, param_location[1] - event.ydata)
            if d < epsilon:
                mouse_selected_type = i
                mouse_selected_index = j
                # Set components to zero or delete waypoint/slowpoint
                if event.button == 3:
                    if mouse_selected_type == 0:
                        del waypoints[mouse_selected_index]
                    elif mouse_selected_type == 1:
                        waypoints[mouse_selected_index][1] = [0, 0, waypoints[mouse_selected_index][1][2]]
                    elif mouse_selected_type == 2:
                        waypoints[mouse_selected_index][1][2] = 0
                    elif mouse_selected_type == 3:
                        del slowpoints[mouse_selected_index]

                # Bring back deleted components
                if event.button == 2:
                    if mouse_selected_type in [0, 1, 2]:
                        if waypoints[mouse_selected_index][1][0:2] == [0, 0]:
                            waypoints[mouse_selected_index][1] = [0, 5, waypoints[mouse_selected_index][1][2]]
                        if waypoints[mouse_selected_index][1][2] == 0:
                            waypoints[mouse_selected_index][1][2] = 2
                update()
                return

    # Select no point
    mouse_selected_type = -1
    mouse_selected_index = -1


def mouse_move(event):
    """Handle mouse move events"""

    # exit if event occurs out of plot
    if event.inaxes is None:
        return

    # move position of the center
    if mouse_selected_type == 0:
        waypoints[mouse_selected_index][0] = [event.xdata, event.ydata, waypoints[mouse_selected_index][0][2]]

    # move the position of the velocity vector
    if mouse_selected_type == 1:
        waypoints[mouse_selected_index][1] = [event.xdata - waypoints[mouse_selected_index][0][0],
                                        event.ydata - waypoints[mouse_selected_index][0][1], waypoints[mouse_selected_index][1][2]]

    # move the position of the acceleration vector
    if mouse_selected_type == 2:
        waypoints[mouse_selected_index][2] = [event.xdata - waypoints[mouse_selected_index][0][0],
                                        event.ydata - waypoints[mouse_selected_index][0][1], waypoints[mouse_selected_index][2][2]]

    # move the position of the rotation vector
    if mouse_selected_type == 3:
        waypoints[mouse_selected_index][0][2] = math.atan2(event.ydata - waypoints[mouse_selected_index][0][1],
                                                     event.xdata - waypoints[mouse_selected_index][0][0])
        waypoints[mouse_selected_index][1][2] = 0

    # move the position of the slowpoint
    if mouse_selected_type == 4:
        slowpoints[mouse_selected_index] = [event.xdata, event.ydata]

    # update screen if a parameter was moved
    if mouse_selected_type != -1:
        update()


def mouse_release(event):
    """Handle mouse release events"""

    global mouse_selected_index
    global mouse_selected_type

    # deselect point on mouse release
    mouse_selected_index = -1
    mouse_selected_type = -1


def key_press(event):
    """Handle key press events"""

    global draw_modules

    # Exit if event occurs out of plot
    if event.inaxes is None:
        return

    # Create new waypoint
    if event.key == 'n':
        waypoints.append([[event.xdata, event.ydata, waypoints[-1][0][2] if len(waypoints) > 0 else 0], [0, 5, 0], [5, 0, 0]])

    # Create new slowpoint
    if event.key == 'z':
        slowpoints.append([event.xdata, event.ydata])

    # Write content to user-defined save file
    if event.key == 'w':
        with open(save_file_name, 'w') as save:
            save.write(str(waypoints) + '\n')
            save.write(str(slowpoints) + '\n')
            save.write(str(commands) + '\n')

    if event.key == 't':
        draw_modules = not draw_modules

    if event.key == 'up' and mouse_selected_type == 2:
        waypoints[mouse_selected_index][0][2] += 2 * math.pi

    if event.key == 'down' and mouse_selected_type == 2:
        waypoints[mouse_selected_index][0][2] -= 2 * math.pi

    update()


def update():
    """Method to redraw plot and update parameters"""

    global param_locations

    # Plot center plots
    center_x = []
    center_y = []
    for i in range(len(waypoints) - 1):
        spline = Spline(waypoints[i], waypoints[i + 1])
        for t in np.arange(0, 1, 0.01):
            x, y, theta = spline.position(t)
            center_x.append(x)
            center_y.append(y)
    center_plot.set_xdata(center_x)
    center_plot.set_ydata(center_y)

    for module_offset, module_plot in zip(module_offsets, module_plots):
        module_x = []
        module_y = []
        if draw_modules:
            for i in range(len(waypoints) - 1):
                spline = Spline(waypoints[i], waypoints[i + 1], radius, module_offset)
                for t in np.arange(0, 1, 0.01):
                    x, y, theta = spline.position(t)
                    module_x.append(x)
                    module_y.append(y)
        module_plot.set_xdata(module_x)
        module_plot.set_ydata(module_y)

    # Plot velocity, acceleration, and rotation plots
    velocity_segments = []
    acceleration_segments = []
    rotation_segments = []
    param_locations = [[], [], [], [], []]
    for i in waypoints:
        velocity_segments.append([(i[0][0], i[0][1]), (i[0][0] + i[1][0], i[0][1] + i[1][1])])
        acceleration_segments.append([(i[0][0], i[0][1]), (i[0][0] + i[2][0], i[0][1] + i[2][1])])
        rotation_segments.append([(i[0][0], i[0][1]), (i[0][0] + 2 * math.cos(i[0][2]), i[0][1] + 2 * math.sin(i[0][2]))])

        param_locations[0].append([i[0][0], i[0][1]])
        param_locations[1].append(velocity_segments[-1][1])
        param_locations[2].append(acceleration_segments[-1][1])
        param_locations[3].append(rotation_segments[-1][1])
    velocity_plot.set_segments(velocity_segments)
    acceleration_plot.set_segments(acceleration_segments)
    rotation_plot.set_segments(rotation_segments)

    # Plot waypoints
    waypoints_plot.set_xdata([waypoint[0][0] for waypoint in waypoints])
    waypoints_plot.set_ydata([waypoint[0][1] for waypoint in waypoints])

    # Plot slowpoints
    slowpoints_plot.set_xdata([slowpoint[0] for slowpoint in slowpoints])
    slowpoints_plot.set_ydata([slowpoint[1] for slowpoint in slowpoints])
    param_locations[4] = slowpoints

    # Redraw plot
    plt.gcf().canvas.draw_idle()


# Selection parameters
mouse_selected_index = -1
mouse_selected_type = -1

draw_modules = True

# Pull values from command line
radius = float(sys.argv[1])
module_offsets = ast.literal_eval(sys.argv[2])
template_path_file_name = sys.argv[3]
save_file_name = sys.argv[4]

# Plot template path and store commands
commands = []
try:
    with open(template_path_file_name, 'r') as template_path_file:

        # Read recording from template file
        recording = list(csv.reader(template_path_file))

        # Convert components to floats and store commands
        for state in recording:
            state[0:3] = [float(component) for component in state[0:3]]
            if state[3]:
                commands.append([state[0], state[1], state[3]])

        # Plot center of robot motion
        plt.plot([state[0] for state in recording], [state[1] for state in recording])

        # Plot modules motion
        # for module_offset in module_offsets:
        #     plt.plot([state[0] + radius * math.cos(state[2] + module_offset) for state in recording],
        #              [state[1] + radius * math.sin(state[2] + module_offset) for state in recording])
except FileNotFoundError:
    print("No template path file found at", template_path_file_name)

# Load previously saved waypoints and slowpoints
waypoints = []
slowpoints = []
try:
    with open(save_file_name, 'r') as save_file:
        # Check if waypoints exist in file
        raw_waypoints = save_file.readline()
        if raw_waypoints:
            waypoints = ast.literal_eval(raw_waypoints)
        else:
            print("No waypoints found in", save_file_name)

        # Check if slowpoints exist in file
        raw_slowpoints = save_file.readline()
        if raw_slowpoints:
            slowpoints = ast.literal_eval(raw_slowpoints)
        else:
            print("No slowpoints found in", save_file_name)
except FileNotFoundError:
    print("No previous save file found at", save_file_name)

# connect event handlers to plot
plt.gcf().canvas.mpl_connect('key_press_event', key_press)
plt.gcf().canvas.mpl_connect('button_press_event', mouse_press)
plt.gcf().canvas.mpl_connect('motion_notify_event', mouse_move)
plt.gcf().canvas.mpl_connect('button_release_event', mouse_release)

# Initialize plots to be updated later
center_plot = plt.plot([], [])[0]
module_plots = [plt.plot([], [])[0] for i in range(len(module_offsets))]
velocity_plot = plt.gca().add_collection(collections.LineCollection([], colors='y'))
rotation_plot = plt.gca().add_collection(collections.LineCollection([], colors='b'))
acceleration_plot = plt.gca().add_collection(collections.LineCollection([], colors='g'))
waypoints_plot = plt.plot([], [], 'ks')[0]
slowpoints_plot = plt.plot([], [], 'ms')[0]

plt.axis('equal')
update()
plt.show()
