#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import pyrealsense2 as rs
import numpy as np
from pyquaternion import Quaternion
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D




W=1280
H=720

def init_camera():
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    conf.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    prof = p.start(conf)
    return p

def get_frame_data(p):
    f = p.wait_for_frames()
    color = color_data(f)
    accel = accel_data(f)
    gyro  = gyro_data(f)
    return (color, accel, gyro)

#def drawq(q):
#    pass
#
#def drawPos(pos):
#    pass

def gyro_data(frames):
    gyro = frames[2].as_motion_frame().get_motion_data()
    return Quaternion(0, gyro.x, gyro.y, gyro.z)


def accel_data(frames):
    accel = frames[1].as_motion_frame().get_motion_data() # how to decide index??
    return np.asarray([accel.x, accel.y, accel.z])


def color_data(frames):
    return np.asanyarray(frames[0].get_data())


def estimate_correction_quaternion(q, a, a0): # this doesn't correct yaw drift
    a0_hat = q.rotate(a)

    la0_hat = np.linalg.norm(a0_hat, ord=2)
    la0  = np.linalg.norm(a0, ord=2)

    b = np.cross(a0_hat/la0_hat,a0/la0)
    theta = math.acos(np.dot(a0_hat, a0)/(la0_hat * la0))

    gain = 0.5 * math.pow(2.2, -abs(la0_hat - la0))
    #print(gain, la0_hat, la0)

    return Quaternion(axis=b, radians=theta * gain)

def plot_frame(ax, R = np.eye(3), t = np.zeros(3).reshape(3,1)):
    ax.quiver(t[0], t[1], t[2], R[0,0], R[0,1], R[0,2], color=(1,0,0), arrow_length_ratio=0)
    ax.quiver(t[0], t[1], t[2], R[1,0], R[1,1], R[1,2], color=(0,1,0), arrow_length_ratio=0)
    ax.quiver(t[0], t[1], t[2], R[2,0], R[2,1], R[2,2], color=(0,0,1), arrow_length_ratio=0)

def main():
    np.set_printoptions(precision=2)

    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    fig = plt.figure()
    ax = Axes3D(fig)
    #ax = fig.gca(projection='3d')

    #ax.view_init(elev=-90., azim=-90)



    p = init_camera()

    q  = Quaternion()
    q2 = Quaternion()
    x = np.zeros(3)
    v = np.zeros(3)
    a0 = get_frame_data(p)[1] # 静止状態

    t = time.time()

    try:
        while True:
            (color, a, w) = get_frame_data(p)
            tn = time.time()
            dt = tn -t
            t = tn
            #print(dt)

            #cv2.imshow('RealSense', color)

            c = cv2.waitKey(1)

            if c == 27 or c == ord('q'):
                break

            qmodel = q + w * q / 2 * dt
            dq = estimate_correction_quaternion(qmodel, a, a0)
            q = dq * qmodel

            amodel = q.rotate(a) # world座標系での加速度
            b = amodel - a0
            v = v + b * dt
            x = x + v * dt

            ax.clear()
            ax.set_aspect('equal')
            ax.set_xlim(-3.0, 3.0)
            ax.set_ylim(-3.0, 3.0)
            ax.set_zlim(-3.0, 3.0)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plot_frame(ax, R=q.rotation_matrix, t=x)

            #ax.quiver(0, 0, 0, a0[0], a0[1], a0[2], color=(1,1,0), arrow_length_ratio=0)
            #ax.quiver(0, 0, 0, a[0], a[1], a[2], color=(0,1,0), arrow_length_ratio=0)
            #ax.quiver(0, 0, 0, a0_hat[0], a0_hat[1], a0_hat[2], color=(0,0,1), arrow_length_ratio=0)
            #b = q.rotate(a)
            #ax.quiver(0, 0, 0, b[0], b[1], b[2], color=(1,0,0), arrow_length_ratio=0)

            plt.pause(0.001)
            #print(q.rotation_matrix)
            #print( "acc: {:.2f} {:.2f} {:.2f} ".format(accel[0], accel[1], accel[2]), end="\r")
            #print( "omg: {:.2f} {:.2f} {:.2f} ".format(gyro[0], gyro[1], gyro[2]), end="\r")

    finally:
        p.stop()


if __name__ == '__main__':
    main()

