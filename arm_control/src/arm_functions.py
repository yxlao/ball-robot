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

# define global variables
joint1_x = 0
joint1_y = 0
joint1_z = 0
joint2_x = 0
joint2_y = 0
joint2_z = 0
joint3_x = 0
joint3_y = 0
joint3_z = 0
gripper_x = 0
gripper_y = 0
gripper_z = 0


def degree_to_xyz(theta1, theta4, theta6, theta7):
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
    # the first joint's position
    global joint1_x
    global joint1_y
    global joint1_z
    joint1_x = T[0][3]
    joint1_y = T[1][3]
    joint1_z = T[2][3]
    T = np.dot(T, T3)
    T = np.dot(T, T4)
    T = np.dot(T, T5)
    # the second joint's position
    global joint2_x
    global joint2_y
    global joint2_z
    joint2_x = T[0][3]
    joint2_y = T[1][3]
    joint2_z = T[2][3]
    T = np.dot(T, T6)
    T = np.dot(T, T7)
    # the third joint's position
    global joint3_x
    global joint3_y
    global joint3_z
    joint3_x = T[0][3]
    joint3_y = T[1][3]
    joint3_z = T[2][3]
    T = np.dot(T, T8)
    T = np.dot(T, T9)
    T = np.dot(T, T10)
    # the gripper's position
    global gripper_x
    global gripper_y
    global gripper_z
    gripper_x = T[0][3]
    gripper_y = T[1][3]
    gripper_z = T[2][3]

    return np.array([gripper_x, gripper_y, gripper_z])


def xyz_to_degree(xd, yd, zd, theta10, theta40, theta60, theta70):
    T11d = 1
    Td = np.array([[T11d],
                   [xd],
                   [yd],
                   [zd]])

    theta1 = theta10 * pi / 180
    theta4 = theta40 * pi / 180
    theta6 = theta60 * pi / 180
    theta7 = theta70 * pi / 180
    Q = np.array([[theta1],
                  [theta4],
                  [theta6],
                  [theta7]])
    x0 = 4.5 * np.sin(theta1 - theta4) + 5.75 * np.cos(theta1 + theta4 + theta6) + 4.5 * np.cos(theta1 + theta4 + theta6 + theta7) - 0.75 * np.sin(theta1 + theta4 + theta6 + theta7) + \
        4.5 * np.cos(theta1 - theta4 - theta6 - theta7) - 4.5 * np.sin(theta1 + theta4) + 0.75 * \
        np.sin(theta1 - theta4 - theta6 - theta7) + \
        5.75 * np.cos(theta1 - theta4 - theta6)
    y0 = -4.5 * np.cos(theta1 - theta4) + 5.75 * np.sin(theta1 + theta4 + theta6) + 0.75 * np.cos(theta1 + theta4 + theta6 + theta7) + 4.5 * np.sin(theta1 + theta4 + theta6 + theta7) + \
        4.5 * np.cos(theta1 + theta4) - 0.75 * np.cos(theta1 - theta4 - theta6 - theta7) + \
        4.5 * np.sin(theta1 - theta4 - theta6 - theta7) + \
        5.75 * np.sin(theta1 - theta4 - theta6)
    z0 = 1.5 * np.cos(theta4 + theta6 + theta7) + 9 * np.sin(theta4 + theta6 +
                                                             theta7) + 11.5 * np.sin(theta4 + theta6) + 9 * np.cos(theta4) + 6.5
    T110 = 0.5 * np.cos(theta1 + theta4 + theta6 + theta7) + 0.5 * \
        np.cos(theta1 - theta4 - theta6 - theta7)
    T = np.array([[T110],
                  [x0],
                  [y0],
                  [z0]])
    DT = np.subtract(T, Td)
    while math.sqrt((DT[1][0]) * (DT[1][0]) + (DT[2][0]) * (DT[2][0]) + (DT[3][0]) * (DT[3][0])) > 0.5:
        J = np.zeros(shape=(4, 4))
        J[0][0] = (-np.sin(theta1 + theta4 + theta6 + theta7) / 2 -
                   np.sin(theta1 - theta4 - theta6 - theta7) / 2)
        J[0][1] = (np.sin(theta1 - theta4 - theta6 - theta7) / 2 -
                   np.sin(theta1 + theta4 + theta6 + theta7) / 2)
        J[0][2] = (np.sin(theta1 - theta4 - theta6 - theta7) / 2 -
                   np.sin(theta1 + theta4 + theta6 + theta7) / 2)
        J[0][3] = (np.sin(theta1 - theta4 - theta6 - theta7) / 2 -
                   np.sin(theta1 + theta4 + theta6 + theta7) / 2)
        J[1][0] = (9 * np.cos(theta1 - theta4)) / 2 - (23 * np.sin(theta1 + theta4 + theta6)) / 4 - (3 * np.cos(theta1 + theta4 + theta6 + theta7)) / 4 - (9 * np.sin(theta1 + theta4 + theta6 + theta7)) / \
            2 - (9 * np.cos(theta1 + theta4)) / 2 + (3 * np.cos(theta1 - theta4 - theta6 - theta7)) / 4 - \
            (9 * np.sin(theta1 - theta4 - theta6 - theta7)) / \
            2 - (23 * np.sin(theta1 - theta4 - theta6)) / 4
        J[1][1] = ((9 * np.sin(theta1 - theta4 - theta6 - theta7)) / 2 - (23 * np.sin(theta1 + theta4 + theta6)) / 4 - (3 * np.cos(theta1 + theta4 + theta6 + theta7)) / 4 - (9 * np.sin(theta1 + theta4 +
                                                                                                                                                                                         theta6 + theta7)) / 2 - (9 * np.cos(theta1 + theta4)) / 2 - (3 * np.cos(theta1 - theta4 - theta6 - theta7)) / 4 - (9 * np.cos(theta1 - theta4)) / 2 + (23 * np.sin(theta1 - theta4 - theta6)) / 4)
        J[1][2] = ((9 * np.sin(theta1 - theta4 - theta6 - theta7)) / 2 - (3 * np.cos(theta1 + theta4 + theta6 + theta7)) / 4 - (9 * np.sin(theta1 + theta4 + theta6 +
                                                                                                                                           theta7)) / 2 - (3 * np.cos(theta1 - theta4 - theta6 - theta7)) / 4 - (23 * np.sin(theta1 + theta4 + theta6)) / 4 + (23 * np.sin(theta1 - theta4 - theta6)) / 4)
        J[1][3] = ((9 * np.sin(theta1 - theta4 - theta6 - theta7)) / 2 - (9 * np.sin(theta1 + theta4 + theta6 + theta7)) /
                   2 - (3 * np.cos(theta1 - theta4 - theta6 - theta7)) / 4 - (3 * np.cos(theta1 + theta4 + theta6 + theta7)) / 4)
        J[2][0] = ((9 * np.sin(theta1 - theta4)) / 2 + (23 * np.cos(theta1 + theta4 + theta6)) / 4 + (9 * np.cos(theta1 + theta4 + theta6 + theta7)) / 2 - (3 * np.sin(theta1 + theta4 + theta6 + theta7)) /
                   4 + (9 * np.cos(theta1 - theta4 - theta6 - theta7)) / 2 - (9 * np.sin(theta1 + theta4)) / 2 + (3 * np.sin(theta1 - theta4 - theta6 - theta7)) / 4 + (23 * np.cos(theta1 - theta4 - theta6)) / 4)
        J[2][1] = ((23 * np.cos(theta1 + theta4 + theta6)) / 4 - (9 * np.sin(theta1 - theta4)) / 2 + (9 * np.cos(theta1 + theta4 + theta6 + theta7)) / 2 - (3 * np.sin(theta1 + theta4 + theta6 + theta7)) /
                   4 - (9 * np.cos(theta1 - theta4 - theta6 - theta7)) / 2 - (9 * np.sin(theta1 + theta4)) / 2 - (3 * np.sin(theta1 - theta4 - theta6 - theta7)) / 4 - (23 * np.cos(theta1 - theta4 - theta6)) / 4)
        J[2][2] = ((23 * np.cos(theta1 + theta4 + theta6)) / 4 + (9 * np.cos(theta1 + theta4 + theta6 + theta7)) / 2 - (3 * np.sin(theta1 + theta4 + theta6 + theta7)) /
                   4 - (9 * np.cos(theta1 - theta4 - theta6 - theta7)) / 2 - (3 * np.sin(theta1 - theta4 - theta6 - theta7)) / 4 - (23 * np.cos(theta1 - theta4 - theta6)) / 4)
        J[2][3] = ((9 * np.cos(theta1 + theta4 + theta6 + theta7)) / 2 - (3 * np.sin(theta1 + theta4 + theta6 + theta7)) /
                   4 - (9 * np.cos(theta1 - theta4 - theta6 - theta7)) / 2 - (3 * np.sin(theta1 - theta4 - theta6 - theta7)) / 4)
        J[3][0] = 0
        J[3][1] = (9 * np.cos(theta4 + theta6 + theta7) - (3 * np.sin(theta4 +
                                                                      theta6 + theta7)) / 2 + (23 * np.cos(theta4 + theta6)) / 2 - 9 * np.sin(theta4))
        J[3][2] = (9 * np.cos(theta4 + theta6 + theta7) - (3 *
                                                           np.sin(theta4 + theta6 + theta7)) / 2 + (23 * np.cos(theta4 + theta6)) / 2)
        J[3][3] = (9 * np.cos(theta4 + theta6 + theta7) -
                   (3 * np.sin(theta4 + theta6 + theta7)) / 2)

        transposeJ = J.transpose()

        INVJ = np.dot(np.linalg.inv(np.dot(transposeJ, J)), transposeJ)
        Q = np.add(Q, np.dot(INVJ, DT))
        theta1 = Q[0][0]
        theta4 = Q[1][0]
        theta6 = Q[2][0]
        theta7 = Q[3][0]
        x = 4.5 * np.sin(theta1 - theta4) + 5.75 * np.cos(theta1 + theta4 + theta6) + 4.5 * np.cos(theta1 + theta4 + theta6 + theta7) - 0.75 * np.sin(theta1 + theta4 + theta6 + theta7) + \
            4.5 * np.cos(theta1 - theta4 - theta6 - theta7) - 4.5 * np.sin(theta1 + theta4) + 0.75 * \
            np.sin(theta1 - theta4 - theta6 - theta7) + \
            5.75 * np.cos(theta1 - theta4 - theta6)
        y = -4.5 * np.cos(theta1 - theta4) + 5.75 * np.sin(theta1 + theta4 + theta6) + 0.75 * np.cos(theta1 + theta4 + theta6 + theta7) + 4.5 * np.sin(theta1 + theta4 + theta6 + theta7) + \
            4.5 * np.cos(theta1 + theta4) - 0.75 * np.cos(theta1 - theta4 - theta6 - theta7) + \
            4.5 * np.sin(theta1 - theta4 - theta6 - theta7) + \
            5.75 * np.sin(theta1 - theta4 - theta6)
        z = 1.5 * np.cos(theta4 + theta6 + theta7) + 9 * np.sin(theta4 + theta6 +
                                                                theta7) + 11.5 * np.sin(theta4 + theta6) + 9 * np.cos(theta4) + 6.5
        T11 = 0.5 * np.cos(theta1 + theta4 + theta6 + theta7) + \
            0.5 * np.cos(theta1 - theta4 - theta6 - theta7)
        T = np.array([[T11],
                      [x],
                      [y],
                      [z]])
        DT = np.subtract(T, Td)
    theta1 = (Q[0][0] * 180 / pi) % 360
    if (theta1 < -180):
        theta1 = theta1 + 360
    theta4 = (Q[1][0] * 180 / pi) % 360
    if (theta4 < -180):
        theta4 = theta4 + 360
    theta6 = (Q[2][0] * 180 / pi) % 360
    if (theta6 < -180):
        theta6 = theta6 + 360
    theta7 = (Q[3][0] * 180 / pi) % 360
    if (theta7 < -180):
        theta7 = theta7 + 360
    return np.array([theta1, theta4, theta6, theta7])
