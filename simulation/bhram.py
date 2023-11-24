from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

from quaternion import Quaternion as qt
from quaternion import Haal

import math


class Thing(object):
    def __init__(self,draw_function,draw_args):
        self.draw_function = draw_function
        self.draw_args = draw_args
        self.haal = Haal()
    def make(self):
        # Q_filterred = self.Q_filterred
        w, v = self.haal.rotation_Q.get_axisangle()
        w_degrees = w*180/math.pi
        location = self.haal.position_P
        glPushMatrix()
        glTranslatef(location[0],location[1],location[2])
        print(location)
        glRotatef(w_degrees,v[0],v[1],v[2])
        self.draw_function(*self.draw_args)
        glPopMatrix()
    def locate(self,x,y,z):
        self.haal.locate(x,y,z)

class Scene(object):
    def __init__(self):
        self.objects = []
        self.scenes = []
        self.scales = []
        self.coordinates = [0.,0.,0.]
    def add_object(self,thing):
        self.objects.append(thing)
    def add_scene(self,scene,x,y,z):
        self.scenes.append(scene)
        self.scales.append([x,y,z])
    def fix_position(self,coords):
        self.coordinates = coords
    def make_scene(self,depth=0):
        if depth > 10:
            return
        glPushMatrix()
        glTranslatef(self.coordinates[0],self.coordinates[1],self.coordinates[2])
        for o in self.objects:
            print(o)
            o.make()
        for s in range(len(self.scenes)):
            scale = self.scales[s]
            glPushMatrix()
            glScalef(scale[0],scale[1],scale[2])
            self.scenes[s].make_scene(depth+1)
            glPopMatrix()
        glPopMatrix()
