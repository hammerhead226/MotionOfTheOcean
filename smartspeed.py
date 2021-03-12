import sys
import ast
import csv
import math
import statistics
import numpy as np
import matplotlib.pyplot as plt

load_file = sys.argv[1]
write_file = sys.argv[2]
[max_v, min_v, max_a_f, max_a_b, max_alpha, theta] = ast.literal_eval(sys.argv[3])
display = (sys.argv[4] == "true")


def mean_filter(ss, data):
    hss = int(ss / 2)
    maa_data = []
    for i in range(hss):
        maa_data.append(data[i])
    for i in range(ss, ss * int((len(data) + hss + 1) / ss), ss):
        for j in range(ss):
            maa_data.append(statistics.mean(data[i - hss:i + hss]))
    for i in range(ss * int((len(data) + hss + 1) / ss) - hss, len(data)):
        maa_data.append(data[i])
    return maa_data


def get_intermediate(index, list):
    fractional = index - int(index)
    return (1.0 - fractional) * list[int(index)] + fractional * list[int(index) + 1]


with open(load_file) as recording:
    temp_recording = list(csv.reader(recording, delimiter=','))
    recording = []
    commands = []
    slowpoints = []

    split = False
    for temp_state in temp_recording:
        if temp_state[0] == '~':
            split = True
        else:
            if split:
                slowpoints.append([float(temp_state[0]), float(temp_state[1])])
            else:
                recording.append(temp_state)

    for state in recording:
        state[0] = float(state[0])
        state[1] = float(state[1])
        state[2] = float(state[2])
        if state[3] != '':
            commands.append(state)

    states_x = [state[0] for state in recording]
    states_y = [state[1] for state in recording]
    states_theta = [state[2] for state in recording]

    for i in range(1, len(recording) - 1):
        j = i + 1
        while states_x[i] == states_x[j] and states_y[i] == states_y[j]:
            if j < len(recording) - 1:
                j += 1
            else:
                break

        if j != i + 1:
            start_x = states_x[i - 1]
            start_y = states_y[i - 1]
            end_x = states_x[j]
            end_y = states_y[j]

            for k in range(i, j):
                states_x[k] = start_x * ((j - k) / (j - i + 1)) + end_x * ((k - i + 1) / (j - i + 1))
                states_y[k] = start_y * ((j - k) / (j - i + 1)) + end_y * ((k - i + 1) / (j - i + 1))

    curvatures = [0]
    for i in range(0, len(recording) - 1):
        v_x_0 = (states_x[i] - states_x[i - 1]) * 50
        v_x_1 = (states_x[i + 1] - states_x[i]) * 50
        v_y_0 = (states_y[i] - states_y[i - 1]) * 50
        v_y_1 = (states_y[i + 1] - states_y[i]) * 50

        v_x = (v_x_0 + v_x_1) / 2
        v_y = (v_y_0 + v_y_1) / 2

        a_x = (v_x_1 - v_x_0) * 50
        a_y = (v_y_1 - v_y_0) * 50

        curvatures.append((v_x * a_y - v_y * a_x) / (math.sqrt(v_x ** 2 + v_y ** 2) ** 3))

    curvatures = mean_filter(16, curvatures)
    max_velocity = []
    for i in range(len(curvatures)):
        if curvatures[i] == 0:
            max_velocity.append(max_v)
        else:
            max_velocity.append(
                min(max_v, math.sqrt(abs(max_alpha / curvatures[i]))))

    print(len(max_velocity))

    for slowpoint in slowpoints:
        min_d = float('inf')
        index = None
        for i in range(len(max_velocity)):
            d = math.hypot(states_x[i] - slowpoint[0], states_y[i] - slowpoint[1])
            if d < min_d:
                min_d = d
                index = i
        max_velocity[index] = min_v

    true_velocity_f = [0]
    for i in range(1, len(max_velocity)):
        d = math.hypot(states_x[i] - states_x[i - 1], states_y[i] - states_y[i - 1])
        v_max = math.sqrt(true_velocity_f[-1] ** 2 + 2 * max_a_f * d)
        true_velocity_f.append(min(max_velocity[i], v_max))

    true_velocity_b = [0]
    for i in range(len(max_velocity) - 2, -1, -1):
        d = math.hypot(states_x[i] - states_x[i + 1], states_y[i] - states_y[i + 1])
        v_max = math.sqrt(true_velocity_b[-1] ** 2 + 2 * max_a_b * d)
        true_velocity_b.append(min(max_velocity[i], v_max))

    true_velocity = []
    for f, b in zip(true_velocity_f, true_velocity_b[::-1]):
        true_velocity.append(min(f, b))

    sped_states = [[states_x[0], states_y[0], theta]]

    i = 0
    while i < len(recording) - 2:
        broke = False
        for j in np.arange(i, len(recording) - 2, 0.01):
            x = get_intermediate(j, states_x)
            y = get_intermediate(j, states_y)
            v = get_intermediate(j, true_velocity)
            d = math.hypot(x - sped_states[-1][0], y - sped_states[-1][1])
            t = (2 * d) / (get_intermediate(i, true_velocity) + get_intermediate(j, true_velocity))
            if abs(0.02 - t) < 0.001:
                i = j
                sped_states.append([x, y, theta])
                broke = True
                break
        if not broke:
            i = i + 1
            x = get_intermediate(i, states_x)
            y = get_intermediate(i, states_y)
            sped_states.append([x, y, theta])

    plt.plot(max_velocity)
    plt.plot(true_velocity)

    command_positions = {}
    for command in commands:
        min_d = float('inf')
        index = None
        for i in range(len(sped_states)):
            d = math.hypot(command[0] - sped_states[i][0], command[1] - sped_states[i][1])
            if d < min_d:
                min_d = d
                index = i
        if index in command_positions.keys():
            command_positions[index] = command_positions[index] + '#' + command[3]
        else:
            command_positions[index] = command[3]

    file = open(write_file, 'w')

    for i in range(len(sped_states)):
        if i in command_positions.keys():
            sped_states[i].append(command_positions[i])

    for state in sped_states:
        if len(state) == 3:
            file.write(','.join(map(str, state)) + ',\n')
        else:
            file.write(','.join(map(str, state)) + '\n')

    if display:
        plt.show()
