from splines import Spline
import sys
import ast
import numpy as np
from matplotlib import pyplot as plt
import math
import statistics


def get_intermediate(index, list):
    fractional = index - int(index)
    return (1.0 - fractional) * list[int(index)] + fractional * list[int(index) + 1]


radius = float(sys.argv[1])
module_offsets = ast.literal_eval(sys.argv[2])
[max_velocity, min_velocity, max_linear_acceleration, max_linear_deceleration, max_centripetal_acceleration] = ast.literal_eval(sys.argv[3])
path_to_follow_file_name = sys.argv[4]
sped_path_to_follow_file_name = sys.argv[5]

# Load waypoints, slowpoints, and commands from the path to follow
waypoints = []
slowpoints = []
commands = []
try:
    with open(path_to_follow_file_name, 'r') as path_to_follow_file:

        # Check if waypoints exist in file
        raw_waypoints = path_to_follow_file.readline()
        if raw_waypoints:
            waypoints = ast.literal_eval(raw_waypoints)
        else:
            print("No waypoints found in", path_to_follow_file_name)

        # Check if slowpoints exist in file
        raw_slowpoints = path_to_follow_file.readline()
        if raw_slowpoints:
            slowpoints = ast.literal_eval(raw_slowpoints)
        else:
            print("No slowpoints found in", path_to_follow_file_name)

        # Check if commands exist in file
        raw_commands = path_to_follow_file.readline()
        if raw_commands:
            commands = ast.literal_eval(raw_commands)
        else:
            print("No commands found in", path_to_follow_file_name)
except:
    print("No path to follow found at", path_to_follow_file_name)

# Convert waypoints to list of coordinates
positions = []
for i in range(len(waypoints) - 1):
    spline = Spline(waypoints[i], waypoints[i + 1])
    for t in np.arange(0, 1, 0.001):
        positions.append(spline.position(t))
spline = Spline(waypoints[-2], waypoints[-1])
positions.append(spline.position(1))

# Compute curvatures from splines
curvatures = []
for i in range(len(waypoints) - 1):
    spline = Spline(waypoints[i], waypoints[i + 1])
    for t in np.arange(0.0005, 1, 0.001):
        curvatures.append(spline.curvature(t))

# Compute maximum velocities based on velocities
max_curvature_velocities = []
for curvature in curvatures:
    if curvature == 0:
        max_curvature_velocities.append(max_velocity)
    else:
        max_curvature_velocities.append(min(max_velocity, math.sqrt(abs(max_centripetal_acceleration / curvature))))

# Compute distance and angle change between points
d_distances = []
d_thetas = []
headings = []
for i in range(len(positions) - 1):
    d_distances.append(math.hypot(positions[i + 1][0] - positions[i][0], positions[i + 1][1] - positions[i][1]))
    d_thetas.append(positions[i + 1][2] - positions[i][2])
    headings.append(math.atan2(positions[i + 1][1] - positions[i][1], positions[i + 1][0] - positions[i][0]))

# Compute maximum velocity based on rotation
max_rotate_velocities = []
for d_distance, d_theta, heading in zip(d_distances, d_thetas, headings):
    current_max_velocity = float('inf')
    for module_offset in module_offsets:
        temp_max_velocity = max_velocity / (math.sqrt(1 + (d_theta / d_distance * radius) ** 2 + 2 * d_theta / d_distance * radius * math.cos(heading - module_offset - math.pi / 2)))
        current_max_velocity = temp_max_velocity if temp_max_velocity < current_max_velocity else current_max_velocity
    max_rotate_velocities.append(current_max_velocity)

# Compute maximum overall velocity
max_velocities = []
for max_rotate_velocity, max_curvature_velocity in zip(max_rotate_velocities, max_curvature_velocities):
    max_velocities.append(min(max_rotate_velocity, max_curvature_velocity))

# Add slowpoints to velocity maximum velocity profile
for slowpoint in slowpoints:
    min_d = float('inf')
    index = None
    for i in range(len(max_velocities)):
        d = math.hypot(positions[i][0] - slowpoint[0], positions[i][1] - slowpoint[1])
        if d < min_d:
            min_d = d
            index = i
    max_velocities[index] = min_velocity

# Shift max velocities by half a point
shifted_max_velocities = [max_velocities[0]]
for i in range(len(max_velocities) - 1):
    shifted_max_velocities.append((max_velocities[i] + max_velocities[i + 1]) / 2)
shifted_max_velocities.append(max_velocities[-1])

# Compute maximum forward velocities based on acceleration
true_velocities_forward = [0]
for d_distance, max_velocity in zip(d_distances, shifted_max_velocities):
    velocity = math.sqrt(true_velocities_forward[-1] ** 2 + 2 * max_linear_acceleration * d_distance)
    if velocity + true_velocities_forward[-1] > 2 * max_velocity:
        velocity = 2 * max_velocity - true_velocities_forward[-1]
    true_velocities_forward.append(velocity)

# Compute maximum backward velocities based on deceleration
true_velocities_backward = [0]
for d_distance, max_velocity in zip(d_distances[::-1], shifted_max_velocities[::-1]):
    velocity = math.sqrt(true_velocities_backward[-1] ** 2 + 2 * max_linear_deceleration * d_distance)
    if velocity + true_velocities_backward[-1] > 2 * max_velocity:
        velocity = 2 * max_velocity - true_velocities_backward[-1]
    true_velocities_backward.append(velocity)

# Compute true velocities from forward and backward velocities
true_velocities = []
for velocity_forward, velocity_backward in zip(true_velocities_forward, true_velocities_backward[::-1]):
    true_velocities.append(min(velocity_forward, velocity_backward))

# Split positions to components
positions_x = [position[0] for position in positions]
positions_y = [position[1] for position in positions]
positions_theta = [position[2] for position in positions]

# Initialize sped positions
sped_positions = [[positions_x[0], positions_y[0], positions_theta[0]]]

# Compute next sped positions
i = 0
while i < len(positions) - 2:
    broke = False
    for j in np.arange(i, len(positions) - 2, 0.01):
        x = get_intermediate(j, positions_x)
        y = get_intermediate(j, positions_y)
        distance = math.hypot(x - sped_positions[-1][0], y - sped_positions[-1][1])
        time = (2 * distance) / (get_intermediate(i, true_velocities) + get_intermediate(j, true_velocities))
        if abs(0.02 - time) < 0.001:
            i = j
            sped_positions.append([x, y, get_intermediate(j, positions_theta)])
            broke = True
            break
    if not broke:
        i = i + 1
        sped_positions.append([get_intermediate(i, positions_x), get_intermediate(i, positions_y), get_intermediate(i, positions_theta)])

# Add commands to output
command_positions = {}
for command in commands:
    min_d = float('inf')
    index = None
    for i in range(len(sped_positions)):
        d = math.hypot(command[0] - sped_positions[i][0], command[1] - sped_positions[i][1])
        if d < min_d:
            min_d = d
            index = i
    if index in command_positions.keys():
        command_positions[index] = command_positions[index] + '#' + command[2]
    else:
        command_positions[index] = command[2]
for i in range(len(sped_positions)):
    if i in command_positions.keys():
        sped_positions[i].append(command_positions[i])

sped_path_to_follow_file = open(sped_path_to_follow_file_name, 'w')
for sped_position in sped_positions:
    if len(sped_position) == 3:
        sped_path_to_follow_file.write(','.join(map(str, sped_position)) + ',\n')
    else:
        sped_path_to_follow_file.write(','.join(map(str, sped_position)) + '\n')

plt.plot(max_velocities)
plt.plot(true_velocities)
plt.plot(curvatures)
plt.show()
