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

import os
import moderngl
from objloader import Obj
from pyrr import Matrix44
from PIL import Image

def initGL():

    ctx = moderngl.create_standalone_context()

    prog = ctx.program(
        vertex_shader='''
            #version 330

            uniform mat4 Mvp;

            in vec3 in_vert;
            in vec3 in_norm;
            in vec2 in_text;

            out vec3 v_vert;
            out vec3 v_norm;
            out vec2 v_text;

            void main() {
                gl_Position = Mvp * vec4(in_vert, 1.0);
                v_vert = in_vert;
                v_norm = in_norm;
                v_text = in_text;
            }
        ''',
        fragment_shader='''
            #version 330
            uniform vec3 Light;
            uniform vec3 Color;
            uniform bool UseTexture;
            uniform sampler2D Texture;
            in vec3 v_vert;
            in vec3 v_norm;
            in vec2 v_text;
            out vec4 f_color;
            void main() {
                float lum = clamp(dot(normalize(Light - v_vert), normalize(v_norm)), 0.0, 1.0) * 0.8 + 0.2;
                if (UseTexture) {
                    f_color = vec4(texture(Texture, v_text).rgb * lum, 1.0);
                } else {
                    f_color = vec4(Color * lum, 1.0);
                }
            }
        ''',
    )

    ctx.clear(1.0, 1.0, 1.0)
    ctx.enable(moderngl.DEPTH_TEST)

    light = prog['Light']
    light.value = (67.69, -8.14, 52.49)

    objects = {}

    for name in ['ground', 'grass']:
        obj = Obj.open('scene-1-%s.obj' % name)
        vbo = ctx.buffer(obj.pack('vx vy vz nx ny nz tx ty'))
        vao = ctx.simple_vertex_array(prog, vbo, 'in_vert', 'in_norm', 'in_text')
        objects[name] = vao

    fbo = ctx.simple_framebuffer((1280, 720))
    fbo.use()

    return (ctx, prog, objects, fbo)


def render(ctx, prog, objects, fbo, q, x, img):
    m = np.eye(4)
    m[0:3,0:3] = q.rotation_matrix
    #m[0:3, 3] = x
    m = Matrix44(m)

    mvp = prog['Mvp']
    proj = Matrix44.perspective_projection(58.0, float(W) / H, 0.1, 1000.0)
    lookat = Matrix44.look_at(
        (47.697, -8.147, 24.498), # eye
        (0.0, 0.0, 8.0),          # target
        (0.0, 0.0, 1.0),          # up(for side)
    )

    mvp.write((proj * m * lookat).astype('f4').tobytes())

    color = prog['Color']
    fbo.clear(0.0, 0.0, 0.0, 0.0)

    color.value = (0.67, 0.49, 0.29)
    objects['ground'].render()

    color.value = (0.46, 0.67, 0.29)
    objects['grass'].render()

    myimg = np.asarray(Image.frombytes('RGBA', fbo.size, fbo.read(components=4)))
    obj_mask = myimg[:,:, 3]
    obj_img  = myimg[:,:, 0:3]
    idx=(obj_mask!=0)
    img[idx]=obj_img[idx]
    cv2.imshow('RealSense', img)


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

def draw_frame(ax, q, x):
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xlim(-3.0, 3.0)
    ax.set_ylim(-3.0, 3.0)
    ax.set_zlim(-3.0, 3.0)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plot_frame(ax, R=q.rotation_matrix, t=x)
    plt.pause(0.001)

def main():
    np.set_printoptions(precision=2)

    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    fig = plt.figure()
    ax = Axes3D(fig)
    #ax = fig.gca(projection='3d')
    #ax.view_init(elev=-90., azim=-90)

    (ctx, prog, objects, fbo) = initGL()

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


            qmodel = q + w * q / 2 * dt
            dq = estimate_correction_quaternion(qmodel, a, a0)
            q = dq * qmodel

            amodel = q.rotate(a) # world座標系での加速度
            b = amodel - a0
            v = v + b * dt
            x = x + v * dt

            #draw_frame(ax, q, x)
            render(ctx, prog, objects, fbo, q, x, color)

            c = cv2.waitKey(1)
            if c == 27 or c == ord('q'):
                break

    finally:
        p.stop()


if __name__ == '__main__':
    main()

