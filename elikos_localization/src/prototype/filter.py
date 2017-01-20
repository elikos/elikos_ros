import matplotlib.pyplot as plt
import numpy as np

f = open('altitude.txt', 'r')

time = []
altitude = []

for line in f:
    word = ''
    for char in line:
        if char != ' ':
            word += char
        else:
            time.append(float(word))
            word = ''
    altitude.append(float(word))

def Q_discrete_white_noise(dt, var):
    return np.array([[dt ** 4 / 4, dt ** 3 / 2],
                     [dt ** 3 / 2, dt ** 2]]) * var

dt = 0.1
var = 0.001
x = np.array([0.0, 0.0])
F = np.array([[1, dt],
              [0, 1 ]])

Q = Q_discrete_white_noise(dt, var)

H = np.array([1, 0])

R = np.array[0.01]

P = np.eye(4) * 0.01


plt.plot(time, altitude)
plt.show()