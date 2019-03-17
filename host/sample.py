import moderngl
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt
from objloader import Obj


ctx = moderngl.create_standalone_context()

prog = ctx.program(
    vertex_shader='''
        #version 330

        in vec3 vert;
        out vec3 v_vert;


        uniform float z_near;
        uniform float z_far;
        uniform float fovy;
        uniform float ratio;

        uniform vec3 center;
        uniform vec3 eye;
        uniform vec3 up;


        mat4 perspective() {
            float zmul = (-2.0 * z_near * z_far) / (z_far - z_near);
            float ymul = 1.0 / tan(fovy * 3.14159265 / 360);
            float xmul = ymul / ratio;

            return mat4(
                xmul, 0.0, 0.0, 0.0,
                0.0, ymul, 0.0, 0.0,
                0.0, 0.0, -1.0, -1.0,
                0.0, 0.0, zmul, 0.0
            );
        }

        mat4 lookat() {
            vec3 forward = normalize(center - eye);
            vec3 side = normalize(cross(forward, up));
            vec3 upward = cross(side, forward);

            return mat4(
                side.x, upward.x, -forward.x, 0,
                side.y, upward.y, -forward.y, 0,
                side.z, upward.z, -forward.z, 0,
                -dot(eye, side), -dot(eye, upward), dot(eye, forward), 1
            );
        }

        void main() {
            v_vert = vert;
            gl_Position = perspective() * lookat() * vec4(vert, 1.0);
        }
    ''',
    fragment_shader='''
        #version 330

        uniform vec3 light;
        in vec3 v_vert;


        out vec4 color;

        void main() {
            //float lum = clamp(dot(normalize(light - v_vert), vec3(1,1,1)), 0.0, 1.0) * 0.8 + 0.2;
            float lum = 1.0f;
            color = vec4(0.0, 0.0, lum, 1.0);
            //color = vec4(v_color, 1.0);
        }
    ''',
)

prog['z_near'].value = 0.1

prog['z_far'].value = 1000.0
prog['ratio'].value = 1280.0 / 720.0
prog['fovy'].value = 58

prog['eye'].value = (10, 10, -10)
prog['center'].value = (0, 0, 0)
prog['up'].value = (0, 1, 0) # 実際にはsideを決める為のもの。そこから上を決める

#prog['light'].value = (0, 0, -1)

obj = Obj.open('./sample.obj')
vbo = ctx.buffer(obj.pack('vx vy vz'))
vao = ctx.simple_vertex_array(prog, vbo, 'vert')

fbo = ctx.simple_framebuffer((1280, 720))
fbo.use()
fbo.clear(0.0, 0.0, 0.0, 0.0)

ctx.clear(1.0, 1.0, 1.0)
ctx.enable(moderngl.DEPTH_TEST)
#vao.render(moderngl.LINES, 65 * 4)
vao.render()

myimg = Image.frombytes('RGBA', fbo.size, fbo.read(components=4), 'raw', 'RGBA', 0, -1)
plt.imshow(myimg)
plt.show()


