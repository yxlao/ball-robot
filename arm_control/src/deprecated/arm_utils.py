import numpy as np
import math

pi = math.pi

# size of manipulator
l0 = 6.5
l1 = 9
l2 = 11.5
tool_y = 1.5
tool_x = 9

# denavit-Hartenberg parameters
a1 = 0
a2 = 0
a3 = 0
a4 = l1
a5 = 0
a6 = l2
a7 = 0
a8 = 0
a9 = 0
a10 = tool_x
apha1 = 0
apha2 = pi / 2
apha3 = 0
apha4 = 0
apha5 = 0
apha6 = 0
apha7 = 0
apha8 = -pi / 2
apha9 = 0
apha10 = 0
d1 = l0
d2 = 0
d3 = 0
d4 = 0
d5 = 0
d6 = 0
d7 = 0
d8 = 0
d9 = tool_y
d10 = 0


def degree_to_position(theta1, theta4, theta6, theta7):
    # transfer theta
    theta1 = theta1 * pi / 180
    theta2 = 0
    theta3 = pi / 2
    theta4 = theta4 * pi / 180
    theta5 = -pi / 2
    theta7 = theta7 * pi / 180
    theta6 = theta6 * pi / 180
    theta8 = 0
    theta9 = 0
    theta10 = 0

    # Computing Transform Matrixes
    T1 = np.array([[np.cos(theta1), -np.sin(theta1) * np.cos(apha1), np.sin(theta1) * np.sin(apha1), a1 * np.cos(theta1)],
                   [np.sin(theta1), np.cos(theta1) * np.cos(apha1), -
                    np.cos(theta1) * np.sin(apha1), a1 * np.sin(theta1)],
                   [0, np.sin(apha1), np.cos(apha1), d1],
                   [0, 0, 0, 1]])
    T2 = np.array([[np.cos(theta2), -np.sin(theta2) * np.cos(apha2), np.sin(theta2) * np.sin(apha2), a2 * np.cos(theta2)],
                   [np.sin(theta2), np.cos(theta2) * np.cos(apha2), -
                    np.cos(theta2) * np.sin(apha2), a2 * np.sin(theta2)],
                   [0, np.sin(apha2), np.cos(apha2), d2],
                   [0, 0, 0, 1]])
    T3 = np.array([[np.cos(theta3), -np.sin(theta3) * np.cos(apha3), np.sin(theta3) * np.sin(apha3), a3 * np.cos(theta3)],
                   [np.sin(theta3), np.cos(theta3) * np.cos(apha3), -
                    np.cos(theta3) * np.sin(apha3), a3 * np.sin(theta3)],
                   [0, np.sin(apha3), np.cos(apha3), d3],
                   [0, 0, 0, 1]])
    T4 = np.array([[np.cos(theta4), -np.sin(theta4) * np.cos(apha4), np.sin(theta4) * np.sin(apha4), a4 * np.cos(theta4)],
                   [np.sin(theta4), np.cos(theta4) * np.cos(apha4), -
                    np.cos(theta4) * np.sin(apha4), a4 * np.sin(theta4)],
                   [0, np.sin(apha4), np.cos(apha4), d4],
                   [0, 0, 0, 1]])
    T5 = np.array([[np.cos(theta5), -np.sin(theta5) * np.cos(apha5), np.sin(theta5) * np.sin(apha5), a5 * np.cos(theta5)],
                   [np.sin(theta5), np.cos(theta5) * np.cos(apha5), -
                    np.cos(theta5) * np.sin(apha5), a5 * np.sin(theta5)],
                   [0, np.sin(apha5), np.cos(apha5), d5],
                   [0, 0, 0, 1]])
    T6 = np.array([[np.cos(theta6), -np.sin(theta6) * np.cos(apha6), np.sin(theta6) * np.sin(apha6), a6 * np.cos(theta6)],
                   [np.sin(theta6), np.cos(theta6) * np.cos(apha6), -
                    np.cos(theta6) * np.sin(apha6), a6 * np.sin(theta6)],
                   [0, np.sin(apha6), np.cos(apha6), d6],
                   [0, 0, 0, 1]])
    T7 = np.array([[np.cos(theta7), -np.sin(theta7) * np.cos(apha7), np.sin(theta7) * np.sin(apha7), a7 * np.cos(theta7)],
                   [np.sin(theta7), np.cos(theta7) * np.cos(apha7), -
                    np.cos(theta7) * np.sin(apha7), a7 * np.sin(theta7)],
                   [0, np.sin(apha7), np.cos(apha7), d7],
                   [0, 0, 0, 1]])
    T8 = np.array([[np.cos(theta8), -np.sin(theta8) * np.cos(apha8), np.sin(theta8) * np.sin(apha8), a8 * np.cos(theta8)],
                   [np.sin(theta8), np.cos(theta8) * np.cos(apha8), -
                    np.cos(theta8) * np.sin(apha8), a8 * np.sin(theta8)],
                   [0, np.sin(apha8), np.cos(apha8), d8],
                   [0, 0, 0, 1]])
    T9 = np.array([[np.cos(theta9), -np.sin(theta9) * np.cos(apha9), np.sin(theta9) * np.sin(apha9), a9 * np.cos(theta9)],
                   [np.sin(theta9), np.cos(theta9) * np.cos(apha9), -
                    np.cos(theta9) * np.sin(apha9), a9 * np.sin(theta9)],
                   [0, np.sin(apha9), np.cos(apha9), d9],
                   [0, 0, 0, 1]])
    T10 = np.array([[np.cos(theta10), -np.sin(theta10) * np.cos(apha10), np.sin(theta10) * np.sin(apha10), a10 * np.cos(theta10)],
                    [np.sin(theta10), np.cos(theta10) * np.cos(apha10), -
                     np.cos(theta10) * np.sin(apha10), a10 * np.sin(theta10)],
                    [0, np.sin(apha10), np.cos(apha10), d10],
                    [0, 0, 0, 1]])

    T = np.dot(T1, T2)
    T = np.dot(T, T3)
    T = np.dot(T, T4)
    T = np.dot(T, T5)
    T = np.dot(T, T6)
    T = np.dot(T, T7)
    T = np.dot(T, T8)
    T = np.dot(T, T9)
    T = np.dot(T, T10)

    return T1
