import sys
import ast
import csv
import math
import statistics
import numpy as np
import matplotlib.pyplot as plt
import time

load_file = sys.argv[1]
write_file = sys.argv[2]
speed_factor = float(sys.argv[3])
[max_v, max_a_f, max_a_b, max_alpha] = ast.literal_eval(sys.argv[4])
display = (sys.argv[5] == "true")


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


class Recording:

    def __init__(self, states):
        self.states = states
        self.states_x = [state[0] for state in self.states]
        self.states_y = [state[1] for state in self.states]
        self.states_theta = [state[2] for state in self.states]

        for i in range(1, len(states) - 1):
            j = i + 1
            while self.states_x[i] == self.states_x[j] and self.states_y[i] == self.states_y[j]:
                if j < len(states) - 1:
                    j += 1
                else:
                    break

            if j != i + 1:
                start_x = self.states_x[i - 1]
                start_y = self.states_y[i - 1]
                end_x = self.states_x[j]
                end_y = self.states_y[j]

                for k in range(i, j):
                    self.states_x[k] = start_x * ((j - k) / (j - i + 1)) + end_x * ((k - i + 1) / (j - i + 1))
                    self.states_y[k] = start_y * ((j - k) / (j - i + 1)) + end_y * ((k - i + 1) / (j - i + 1))

        self.curvatures = [0]
        self.recorded_velocities = [0]
        for i in range(0, len(recording) - 1):
            v_x_0 = (self.states_x[i] - self.states_x[i - 1]) * 50
            v_x_1 = (self.states_x[i + 1] - self.states_x[i]) * 50
            v_y_0 = (self.states_y[i] - self.states_y[i - 1]) * 50
            v_y_1 = (self.states_y[i + 1] - self.states_y[i]) * 50

            v_x = (v_x_0 + v_x_1) / 2
            v_y = (v_y_0 + v_y_1) / 2

            a_x = (v_x_1 - v_x_0) * 50
            a_y = (v_y_1 - v_y_0) * 50

            self.curvatures.append((v_x * a_y - v_y * a_x) / (math.sqrt(v_x ** 2 + v_y ** 2) ** 3))
            self.recorded_velocities.append(math.hypot(v_x, v_y) * speed_factor)

        self.recorded_velocities = mean_filter(16, self.recorded_velocities)
        self.curvatures = mean_filter(16, self.curvatures)
        self.max_velocity = []
        for i in range(len(self.curvatures)):
            if self.curvatures[i] == 0:
                self.max_velocity.append(min(self.recorded_velocities[i], max_v))
            else:
                self.max_velocity.append(min(max_v, math.sqrt(abs(max_alpha / self.curvatures[i])), self.recorded_velocities[i]))

        print(len(self.max_velocity))

        self.true_velocity_f = [0]
        for i in range(1, len(self.max_velocity)):
            d = math.hypot(self.states_x[i] - self.states_x[i - 1], self.states_y[i] - self.states_y[i - 1])
            v_max = math.sqrt(self.true_velocity_f[-1] ** 2 + 2 * max_a_f * d)
            self.true_velocity_f.append(min(self.max_velocity[i], v_max))

        self.true_velocity_b = [0]
        for i in range(len(self.max_velocity) - 2, -1, -1):
            d = math.hypot(self.states_x[i] - self.states_x[i + 1], self.states_y[i] - self.states_y[i + 1])
            v_max = math.sqrt(self.true_velocity_b[-1] ** 2 + 2 * max_a_b * d)
            self.true_velocity_b.append(min(self.max_velocity[i], v_max))

        self.true_velocity = []
        for f, b in zip(self.true_velocity_f, self.true_velocity_b[::-1]):
            self.true_velocity.append(min(f, b))

        self.sped_states = [[self.states_x[0], self.states_y[0], 0]]
        i = 0
        while i < len(self.states) - 2:
            broke = False
            for j in np.arange(i, len(self.states) - 2, 0.001):
                x = get_intermediate(j, self.states_x)
                y = get_intermediate(j, self.states_y)
                v = get_intermediate(j, self.true_velocity)
                d = math.hypot(x - self.sped_states[-1][0], y - self.sped_states[-1][1])
                t = (2 * d) / (get_intermediate(i, self.true_velocity) + get_intermediate(j, self.true_velocity))
                # print(j, t, d, get_intermediate(i, self.true_velocity), get_intermediate(j, self.true_velocity))
                if abs(0.02 - t) < 0.005:
                    i = j
                    self.sped_states.append([x, y, 0])
                    broke = True
                    break
            if not broke:
                i = i + 1
                x = get_intermediate(i, self.states_x)
                y = get_intermediate(i, self.states_y)
                self.sped_states.append([x, y, 0])

        plt.plot(self.recorded_velocities)
        # plt.plot(self.states_x, self.states_y)
        # for i in range(len(self.states)):
        #     if i % 10 == 0:
        #         plt.annotate(i, (self.states_x[i], self.states_y[i]))
        plt.plot(self.max_velocity)
        plt.plot(self.true_velocity)

    def get_states(self):
        return [[self.states_x[i], self.states_y[i], self.states_theta[i]] for i in range(len(self.states_x))]

    def get_sped(self):
        return self.sped_states


with open(load_file) as recording:
    recording = list(csv.reader(recording, delimiter=','))
    recording = [list(map(float, state[0:3])) for state in recording]

    recording = Recording(recording)
    sped = recording.get_sped()

    file = open(write_file, 'w')
    for state in sped:
        file.write(','.join(map(str, state)) + '\n')

    if display:
        plt.show()