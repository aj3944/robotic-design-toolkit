from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

import numpy as np
import math
from quaternion import Quaternion as qt
from quaternion import Haal

from bhram import Thing,Scene


rad_2_deg = 180/math.pi;


class Manipulator(object):
    def __init__(self):
        name = "manipulator"
        self.thing = Thing(self.draw_func, [])
        self.init()
        self.draw_axes_FLAG = False;
        self.draw_frames_FLAG = False;
        self.draw_link_FLAG = True;
    def init(self):
        self.joint_values = []
        self.links = []
        self.frames = []
        self.DH_PARAMETERS = []
    
    def make_manip(self,DH_TABLE,JOINT_TYPES):
        self.init()
        for i in range(len(DH_TABLE)):
            self.frames.append(Haal())
            # if i > 0 :
            self.joint_values.append(0)
            self.links.append(JOINT_TYPES[i - 1])
            self.DH_PARAMETERS.append(DH_TABLE[i])
        # self.frames.append(len(DH_TABLE)) #END EFFECTOR
    def set_joint_angles(self,joint_angles):
        if not len(joint_angles) == len(self.joint_values):
            return 
        else:
            self.joint_values = joint_angles

    def draw_func(self):

        # self.thing.locate(100,100,100)
        

        for i in range(len(self.DH_PARAMETERS)):
            link_rot = self.DH_PARAMETERS[i][0]*rad_2_deg + self.joint_values[i]
            link_off = self.DH_PARAMETERS[i][1]            
            link_len = self.DH_PARAMETERS[i][2]
            link_twi = self.DH_PARAMETERS[i][3]*rad_2_deg


            glRotatef(link_twi,1,0,0)
            glRotatef(link_rot,0,1.,0.)
            glTranslatef(link_len/2,0.,0.)
            glPushMatrix()

            self.draw_link(link_len,i);

            glPopMatrix()
            glTranslatef(link_len/2,0.,0.)

            # print(t_d_a_a)
            # glPushMatrix()
            # glRotatef(t_d_a_a[0],1.0,0.,0.)
            # glScalef(t_d_a_a[2],t_d_a_a[1],1)
            # glutSolidCube(link_size)
            # glTranslatef(link_size/2.,0.,0.)
            # glRotatef(t_d_a_a[3],0,0,1)
            # glPopMatrix()

        # for t_d_a_a in self.DH_PARAMETERS:
        #     glutWireSphere(1,20,20)
        #     glPopMatrix()
        # glutWireSphere(1,20,20)
        # pass
    def draw_link(self,link_len,i=0):

        glPushMatrix();
        glTranslatef(-link_len/2,0,0)
        # glTranslatef(0.,0.)
        # glTranslatef(0.,5.,0.)

        if self.draw_axes_FLAG:
            glPushMatrix()
            glTranslatef(-5.,0.,0.)
            glColor3f(1.0, 0., 0.);
            glScalef(10,1,1)
            glutSolidCube(1)
            glPopMatrix()

            glPushMatrix()        
            glTranslatef(0.,-5.,0.)
            glColor3f(0., 1.0,0.);
            glScalef(1,10,1)
            glutSolidCube(1)
            glPopMatrix()

            glPushMatrix()
            glTranslatef(0.,0.,-5.)
            glColor3f(0., 0., 1.0);
            glScalef(1,1,10)
            glutSolidCube(1)
            glPopMatrix()

        if self.draw_frames_FLAG:
            glPushMatrix()        
            glColor3f(1., 1., 1.0);
            glTranslatef(0.,0.,0.)
            text = b"{%d}"%(i)
            glRasterPos2i(1,1);        
            glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, text)
            glPopMatrix()

        glPopMatrix();

        if self.draw_link_FLAG:
            glPushMatrix()        
            glColor3f(1., 1.0,1.);
            glScalef(link_len,5.,5.)
            glutWireCube(1)
            glPopMatrix()


        # glutSolidCube(1)
    def do_fk(self):
        pass
    def do_ik(self,goal):
        pass
    def do_id(self,wrench_force):
        pass
    def compute_ws(self):
        pass
    def trajectory(goal1,goal2,optimization = 0):
        pass

        





mp1 = Manipulator();
