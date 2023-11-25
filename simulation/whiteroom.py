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




from bhram import Thing,Scene
from bot import Manipulator as mp


thread_len1 = 1;
thread_len2 = 1;
position = [1000,1000,1000]

class Room(object):
    animate = 1
    def __init__(self,DRAW_SCENE):
        self.window_name = "Empty"
        self.camera_heading = [0,0,0]
        self.window_function()
        self.DRAW_SCENE = DRAW_SCENE
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

    def resize(self,w,h):
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity()
        aspect = w / h;
        glOrtho(-aspect, aspect, -1, 1, -1, 1);
    def draw_display(self):
        # if self.animate:
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        color = [1.0,0.,0.,1.]
        self.look_at_scene()
        glPushMatrix()
        self.draw_room()
        self.DRAW_SCENE.make_scene()
        glPopMatrix()
        # else:
            # pass
        # glutSwapBuffers()
        return
    def window_function(self):
        glutInit()
        # glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        # glutInitWindowSize(300,300)
        # glutCreateWindow(self.window_name)

        glClearColor(0.,0.,0.,1.)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        # glutKeyboardFunc(self.keyboard_funtion)
        # glutDisplayFunc(self.draw_display)
        glMatrixMode(GL_PROJECTION)
        gluPerspective(40.,1.,1.,40.)
        glMatrixMode(GL_MODELVIEW)
        gluLookAt(0,0,10,
                  0,0,0,
                  0,1,1)
        glPushMatrix()
        # glutMainLoop()
        return
    def draw_function(self):
        glutPostRedisplay()
        glutMainLoopEvent()



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