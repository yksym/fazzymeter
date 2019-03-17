import os

import moderngl
import numpy as np
from objloader import Obj
from PIL import Image
from pyrr import Matrix44
from matplotlib import pyplot as plt



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

mvp = prog['Mvp']
light = prog['Light']
color = prog['Color']
use_texture = prog['UseTexture']

objects = {}

for name in ['ground', 'grass']:
    obj = Obj.open('scene-1-%s.obj' % name)
    vbo = ctx.buffer(obj.pack('vx vy vz nx ny nz tx ty'))
    vao = ctx.simple_vertex_array(prog, vbo, 'in_vert', 'in_norm', 'in_text')
    objects[name] = vao


ctx.clear(1.0, 1.0, 1.0)
ctx.enable(moderngl.DEPTH_TEST)

proj = Matrix44.perspective_projection(58.0, 1280.0 / 720.0, 0.1, 1000.0)

lookat = Matrix44.look_at(
    (47.697, -8.147, 24.498), # eye
    (0.0, 0.0, 8.0),          # target
    (0.0, 0.0, 1.0),          # up(for side)
)

light.value = (67.69, -8.14, 52.49)
mvp.write((proj * lookat).astype('f4').tobytes())

fbo = ctx.simple_framebuffer((1280, 720))
fbo.use()
fbo.clear(0.0, 0.0, 0.0, 0.0)

color.value = (0.67, 0.49, 0.29)
objects['ground'].render()

color.value = (0.46, 0.67, 0.29)
objects['grass'].render()

myimg = Image.frombytes('RGBA', fbo.size, fbo.read(components=4), 'raw', 'RGBA', 0, -1)
print(myimg)
png = Image.open("rose.png").convert('RGBA')
print(png)
alpha_composite = Image.alpha_composite(png, myimg)
plt.imshow(alpha_composite)
plt.show()
#alpha_composite.save('foo.jpg', 'JPEG', quality=80)

