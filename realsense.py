#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyrealsense2 as rs
import numpy as np
from pyquaternion import Quaternion


def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p

def get_frame_data(p):
    f = p.wait_for_frames()
    accel = np.array(accel_data(f[0].as_motion_frame().get_motion_data()))
    w = gyro_data(f[1].as_motion_frame().get_motion_data())
    return (accel, Quaternion(0, w[0], w[1], w[2]))

#def drawq(q):
#    pass
#
#def drawPos(pos):
#    pass

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])

def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

def dq(w,q):
    return np.dot(rotmat(w), q)

def main():
    p = initialize_camera()
    q = Quaternion()
    a0 = get_frame_data(p)[0] # 静止を仮定

    np.set_printoptions(precision=2)
    try:
        while True:
            (a, w) = get_frame_data(p)
            #print(a,w, end="\r")
            q = q + w * q / 2
            print(q.rotation_matrix)
            #print( "acc: {:.2f} {:.2f} {:.2f} ".format(accel[0], accel[1], accel[2]), end="\r")
            #print( "omg: {:.2f} {:.2f} {:.2f} ".format(gyro[0], gyro[1], gyro[2]), end="\r")

    finally:
        p.stop()


if __name__ == '__main__':
    main()

