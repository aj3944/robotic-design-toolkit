from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys
import math
import numpy as np
import time




from sklearn.preprocessing import normalize

from jax import grad,jacobian
from pyassimp import load

from quaternion import Quaternion as qt
from quaternion import Haal


# from link_geometry import CoreCube

from bhram import Thing,Scene,bhram_vertex_shader,bhram_fragment_shader

from bot import Manipulator as mp

from OpenGL.GL.shaders import compileShader,compileProgram

import shaders as shader


thread_len1 = 1;
thread_len2 = 1;
position = [1000,1000,1000]
sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)

bhram_program = 0


# Shader global variables
shaders_programID         = None
shaders_frustumID         = None
shaders_viewID            = None
shaders_template_location = None
shaders_position_location = None
shaders_color_location    = None

# VBO global variables
template_buffer  = None
position_buffer  = None
color_buffer     = None
template_data    = None
position_data    = None
color_data       = None
VERTEX_SIZE      = 3 # 3 vertices per triangle
POSITION_SIZE    = 3 # xyz
COLOR_SIZE       = 3 # rgb



class Room(object):
    animate = 1

    def __init__(self,DRAW_SCENE):
        self.window_name = "Empty"
        self.camera_heading = [0,0,0]
        self.window_function()
        self.DRAW_SCENE = DRAW_SCENE
        self.motion_curr = (0,0)
        self.motion_prev = (0,0)
        self.m_count = 0

    def update(self):
        self.draw_display()
        # pass
    def look_at_scene(self):
        # glMatrixMode(GL_PROJECTION)
        d = math.sqrt(sum([x*x for x in position]))
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(40.,1.,d/100.,4*d)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(position[0],position[1],position[2],
          0,0,0,
          0,0,1)
    def draw_room(self):
        glPushMatrix()
        # glTranslatef(position[0],position[1],position[2])
        glutWireCube(1000.0)
        glPopMatrix()
    def keyboard_funtion(self,*args):
        return
        global position,thread_len1,thread_len2
        d = math.sqrt(sum([x*x for x in position]))
        key = args[0]
        print(key)
        if key == ' ':
            snap_zero = True
        if key == 'w':
            position = [position[0]*(1-0.1),position[1]*(1-0.1),position[2]*(1-0.1)]
        if key == 's':
            position = [position[0]*(1+0.1),position[1]*(1+0.1),position[2]*(1+0.1)]
        if key == 'd':
            position = np.add(position,np.cross(position,[0,0,1])*0.1)
        if key == 'a':
            position = np.subtract(position,np.cross(position,[0,0,1])*0.1)
        if key == 'r':
            position = [2,2,0]
        if key == '\'':
            thread_len1 += 0.1
        if key == '/':
            thread_len1 -= 0.1
        if key == ';':
            thread_len2 += 0.1
        if key == '.':
            thread_len2 -= 0.1
        # print(key)
        # print("\t\tThread len 1: ",thread_len1)
        # print("\t\tThread len 2: ",thread_len2)
        # print(position)
    def zoom_funtion(self,*args):
        global position
        delta = args[0]/10
        position = [ i*(1+delta) for i in position]
    def motion_funtion(self,*args):
        global position
        if not self.m_count:
            self.motion_curr = (args[0],args[1])
            self.motion_prev = self.motion_curr
            self.m_count += 1
        else:
            self.motion_curr = (args[0],args[1])
            del_x = self.motion_curr[0] - self.motion_prev[0]
            del_y = self.motion_curr[1] - self.motion_prev[1]
            self.motion_prev = self.motion_curr
            if (del_x*del_x > 1000) or del_y*del_y > 1000:
                return 
            fac = del_x*5
            print("del x,y ",del_x,del_y)
            if del_x:
                normalized_position = position / np.linalg.norm(position)
                position = np.add(position,np.cross(normalized_position,[0,0,1])*(fac))
            else:
                self.m_count = 0
            
            position = [position[0],position[1],position[2] + del_y*math.log(position[2]*position[2]) ]

            # position = np.add(position,np.cross(position,[0,0,1])*0.1)

    def resize(self,w,h):
        print("<-RESIZE->")
        # glMatrixMode(GL_PROJECTION);
        # glLoadIdentity()
        aspect_ratio = w / h;
        aspect_ratio = 1.;
        # glOrtho(-aspect, aspect, -1, 1, -1, 1);
        glViewport(0, 0, w, h)
        nearClip, farClip = 10,1000
        left, right, bottom, top = -200.,200.,-200.,200.
        glOrtho(left * aspect_ratio, right * aspect_ratio, bottom, top, nearClip,farClip)
    def draw_display(self):        
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        color = [1.0,0.,0.,1.]
        
        self.look_at_scene()
        
        glPushMatrix()

        self.draw_room()
        self.DRAW_SCENE.make_scene()
        
        glPopMatrix()
        return
    def window_function(self):

        glutInit()
        

        # self.initShaders()
        # self.initVBOs()
        # TODO FIX_SHADERS
        # bhram_program = compileProgram( 
        #     compileShader(bhram_vertex_shader, GL_VERTEX_SHADER),
        #     compileShader(bhram_fragment_shader, GL_FRAGMENT_SHADER))



        glClearColor(0.,0.,0.,1.)
        glLight(GL_LIGHT0, GL_POSITION,  (5, 5, 5, 1)) # point light from the left, top, front
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0, 0, 0, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1, 1, 1, 1))


        glShadeModel(GL_FLAT)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE )


        glUseProgram(bhram_program);



        gluPerspective(40.,1.,1.,40.)
        glMatrixMode(GL_MODELVIEW)
        gluLookAt(0,0,10,
                  0,0,0,
                  0,1,1)
        glPushMatrix()
        return
    def draw_function(self):
        glutPostRedisplay()
        glutMainLoopEvent()

    def initShaders(self):
        global shaders_programID, shaders_frustumID, shaders_viewID, shaders_template_location, shaders_position_location, shaders_color_location
        vertex_shader   = shader.compile_shader("VS")
        fragment_shader = shader.compile_shader("FS")
        shaders_programID = shader.link_shader_program(vertex_shader, fragment_shader)     

        shaders_template_location = glGetAttribLocation(shaders_programID, "templateVS")
        shaders_position_location = glGetAttribLocation(shaders_programID, "positionVS")
        shaders_color_location    = glGetAttribLocation(shaders_programID, "colorVS_in")

        shaders_frustumID = glGetUniformLocation(shaders_programID, "frustum")
        shaders_viewID = glGetUniformLocation(shaders_programID, "view" )
    def initVBOs(self):
        global template_buffer, position_buffer, color_buffer, template_data, position_data
          
        # Opengl VBO vertex template buffer created, bound, and filled with vertex template data
        template_buffer = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, template_buffer)
        glBufferData(GL_ARRAY_BUFFER, template_data, GL_STATIC_DRAW)

        # Opengl VBO cube position buffer created, bound, and filled with cube position data
        position_buffer = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, position_buffer)
        glBufferData(GL_ARRAY_BUFFER, position_data, GL_STATIC_DRAW)

        # Opengl VBO cube color buffer created, bound, and left empty
        color_buffer = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, color_buffer)
        glBufferData(GL_ARRAY_BUFFER, None, GL_STREAM_DRAW)
# _CELESTIAL_SPHERE_ = Thing(glutWireSphere,(1,20,20))



# while 21:
#     R.update()

#     for i in range(len(joint_values)):
#         joint_values[i] += thread_len1*10
#     # joint_values = [
#     #     thread_len1*5,
#     #     thread_len2*5,
#     #     0
#     # ]

#     my_manip.set_joint_angles(joint_values);
#     time.sleep(1/24)
    # for i in range(1000000):
    #     pass