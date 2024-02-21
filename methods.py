

import models as mod
import math
import matplotlib.pyplot as plt
import numpy as np


def f_x(t, x):
    return -t * x


def f_y(t, y):
    return t * y


def create_body(t, num_points=10):
    radius = 3
    material_points = []
    theta = np.linspace(0, -np.pi/2, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    m = 0
    for i in range(len(x)):
        material_points.append(mod.MaterialPoint(m, x[i], y[i], f_x(t, x), f_y(t, y), x, y, t))
        m += 1
    material_body = mod.MaterialBody(material_points)
    return material_body

def runge_kutta(t, h, x_k, y_k):
    c2, c3, a21, a31, a32, b1, b2, b3 = 2 / 3, 2 / 3, 2 / 3, -1 / 3, 1, 1 / 4, 2 / 4, 1 / 4

    kx1 = f_x(t, x_k)
    kx2 = f_x((t + h * c2), (x_k + a21 * kx1 * h))
    kx3 = f_x((t + h * c3), (x_k + a31 * kx1 * h + a32 * kx2 * h))
    x_t = x_k + h * (kx1 * b1 + kx2 * b2 + kx3 * b3)

    ky1 = f_y(t, y_k)
    ky2 = f_y((t + h * c2), (y_k + a21 * ky1 * h))
    ky3 = f_y((t + h * c3), (y_k + a31 * ky1 * h + a32 * ky2 * h))
    y_t = y_k + h * (ky1 * b1 + ky2 * b2 + ky3 * b3)

    return x_t, y_t

def move_body(time, h, mb):
    point_trajectories = []
    for i in range(len(mb.material_points)):
        t = 0
        x_0 = mb.material_points[1].x_0[i]
        y_0 = mb.material_points[1].y_0[i]
        x_t = [x_0]
        y_t = [y_0]
        for n in range(int(time / h) + 1):
            x_k = x_t[n]
            y_k = y_t[n]

            x_t.append(runge_kutta(t, h, x_k, y_k)[0])
            y_t.append(runge_kutta(t, h, x_k, y_k)[1])

            t += h
        point_trajectories.append(mod.PointTrajectory(mb.material_points[i], x_t, y_t))
    body_trajectory = mod.BodyTrajectory(point_trajectories, mb)
    return body_trajectory

def plot_trajectory(mb, bt):
    for i in range(len(mb.material_points)):
        plt.plot(mb.material_points[i].coord_x, mb.material_points[i].coord_y, 'r.')
    for i in range(len(mb.material_points)):
        plt.plot(bt.point_trajectories[i].x, bt.point_trajectories[i].y, 'b', linewidth=0.1)
    for i in range(len(mb.material_points)):
        plt.plot(bt.point_trajectories[i].x[-1], bt.point_trajectories[i].y[-1], 'g.')
    plt.axis('equal')
    plt.grid()
    # plt.show()
    plt.savefig('assets/plot_trajectory.png', format='png')

def make_through_space(h=10):
    X = np.linspace(0, 3, h)
    Y = np.linspace(-5, 0, h)
    Vx = np.zeros((h, h))
    Vy = np.zeros((h, h))
    sp = mod.SpacePoint(X, Y, Vx, Vy)
    return sp
def plot_streamline(time, sp, h=10):

    X = sp.x
    Y = sp.y
    Vx = sp.Vx
    Vy = sp.Vy
    streamline = []

    for i in range(h):
        for j in range(h):
            Vx[i][j] = (f_x(time, X[i]))
            Vy[i][j] = (f_y(time, Y[i]))

    streamline.append(mod.StreamLine(Vx, Vy))
    space = mod.Space(streamline)

    fig, ax = plt.subplots()

    ax.quiver(X, Y, Vx, Vy)
    ax.streamplot(X, Y, Vx, Vy, color='b')

    plt.savefig(f'assets/vf{str(time)}.png', format='png', dpi=300)

    return space

















# def move_through_space(time, h):
#     t = h
#     m = 0
#     a = np.linspace(-4, 4, 9)
#     x_s, y_s = np.meshgrid(a, a)
#     velocity_fields = []
#     for n in range(int(time / h)):
#         space_points = []
#         for i in range(9):
#             for j in range(9):
#                 x = x_s[i, j]
#                 y = y_s[i, j]
#                 space_points.append(mod.SpacePoint(m, x, y, f_x(t, x), f_y(t, y), t))
#                 m += 1
#         velocity_fields.append(mod.Space(space_points))
#         t += h
#     return velocity_fields
#
#
# def plot_streamlines(vf):
#     h = vf[0].space_points[0].t
#     t = h
#     for n in range(len(vf)):
#         plt.figure(n)
#         plt.suptitle('t = ' + str(t))
#         m = 0
#         coord_x = []
#         coord_y = []
#         v_x = []
#         v_y = []
#         for i in range(9):
#             for j in range(9):
#                 coord_x.append(vf[n].space_points[m].coord_x)
#                 coord_y.append(vf[n].space_points[m].coord_y)
#                 v_x.append(vf[n].space_points[m].velocity_x)
#                 v_y.append(vf[n].space_points[m].velocity_y)
#                 m += 1
#         plt.subplot(1, 2, 1)
#         plt.quiver(coord_x, coord_y, v_x, v_y)
#         for p in range(1, 4):
#             for q in range(1, 4):
#                 x = np.linspace(-4, -0.1, 100)
#                 d = t / t + 1
#                 c = q * (p ** d)
#                 y = c * ((-x) ** (-d))
#                 plt.subplot(1, 2, 2)
#                 plt.axis([-4, 4, -4, 4])
#                 plt.plot(x, y)
#         t += h
#         # plt.show()
#         plt.savefig('assets/velocity_fields' + str(n) + '.png', format='png', dpi=300)