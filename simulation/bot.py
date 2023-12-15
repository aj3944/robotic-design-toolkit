from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

import numpy as np
import math
from quaternion import Quaternion as qt
from quaternion import Haal,Transformation

from bhram import Thing,Scene
from math import sin, cos, acos, sqrt

from scipy.spatial.transform import Rotation as R

rad_2_deg = 180./math.pi;
deg_2_rad = math.pi/180.;

def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm


class Manipulator(object):

    goal = Haal()

    def __init__(self):
        name = "manipulator"
        self.thing = Thing(self.draw_func, [])
        self.init()
        self.draw_axes_FLAG = False;
        self.draw_frames_FLAG = False;
        self.draw_link_FLAG = True;
        self.draw_FK = True;
    def init(self):
        self.joint_values = []
        self.links = []
        self.frames = []
        self.DH_PARAMETERS = []
        self.FORWARD = Transformation()
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
 

        if self.draw_FK:
            # glRotatef(link_twi,1,0,0)
            pos = self.FORWARD.translation
            # rot_mat = np.array(self.FORWARD.rotation.as_matrix())

            fk_matrix = np.array(self.FORWARD.T_matrix());

            rota = R.from_matrix(fk_matrix[:3,:3])
            rot_vec = rota.as_rotvec(degrees = True)

            theta = np.linalg.norm(rot_vec)
            v = normalize(rot_vec)

            glPushMatrix()
            glTranslatef(pos[0],pos[1],pos[2])
            glRotatef(theta,v[0],v[1],v[2])
            # glMatrixMode(GL_MODELVIEW)
            # glLoadIdentity()
            # modelViewMatrix = glGetDoublev(GL_MODELVIEW_MATRIX)
            # print(fk_matrix)
            # glMultMatrixf(fk_matrix)
            glColor3f(1.0, 1.0, 0.);
            glutSolidCube(1)
            glPopMatrix()       
        

        for i in range(len(self.DH_PARAMETERS)):
            link_rot = (self.DH_PARAMETERS[i][0] + self.joint_values[i])*rad_2_deg
            link_off = self.DH_PARAMETERS[i][1]            
            link_len = self.DH_PARAMETERS[i][2]
            link_twi = self.DH_PARAMETERS[i][3]*rad_2_deg


            glRotatef(link_rot,0,0.,1.)
            glRotatef(link_twi,1,0,0)
            glTranslatef(0.,0.,link_off)
            glTranslatef(link_len/2,0.,0.)
            # glTranslatef(0.,0.,link_off/2.)
            glPushMatrix()

            self.draw_link(link_len,i,link_off);

            glPopMatrix()
            glTranslatef(link_len/2,0.,0.)


    def draw_link(self,link_len,i=0,off= 2):

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
            glScalef(link_len,1.,2)
            glutWireCube(1)
            glPopMatrix()


        # glutSolidCube(1)
    def do_fk(self,joint_values):
        frames = []
        orig_matrix = Transformation()
        curr_matrix = Transformation()
        # print("DHTABLE")
        # print(self.joint_values)
        # print(self.DH_PARAMETERS)
        for i in range(len(self.DH_PARAMETERS)):

            print(self.joint_values)
            theta = self.DH_PARAMETERS[i][0] + self.joint_values[i]
            link_off = self.DH_PARAMETERS[i][1]
            link_len = self.DH_PARAMETERS[i][2]
            alpha = self.DH_PARAMETERS[i][3]

            # print("<-----------{0}------------------>\n".format(i))
            # print(link_rot,link_off,link_len,link_twi)
            # print("<------------------------------->\n")
            frame_matrix = Transformation()
            frame_transform = frame_matrix.T_matrix()

            frame_transform[0][0] = cos(theta);
            frame_transform[0][1] = -sin(theta)*cos(alpha);
            frame_transform[0][2] = sin(alpha)*sin(theta);
            frame_transform[0][3] = link_len*cos(theta);

            frame_transform[1][0] = sin(theta);
            frame_transform[1][1] = cos(alpha)*cos(theta);
            frame_transform[1][2] = -sin(alpha)*cos(theta);
            frame_transform[1][3] = link_len*sin(theta);

            frame_transform[2][0] = 0;
            frame_transform[2][1] = sin(alpha);
            frame_transform[2][2] = cos(alpha);
            frame_transform[2][3] = link_off;


            prev_transform = curr_matrix.T_matrix();

            new_transform = np.matmul(prev_transform,frame_transform);
    
            curr_matrix.rotation = R.from_matrix(new_transform[:3,:3]);
            curr_matrix.translation = list(np.ndarray.flatten(new_transform[:-1,-1:]));


            print("<-----------{0}------------------>\n".format(i));
            print(new_transform);
            print("<------------------------------->\n");

            # new_x = curr_haal.position_P[0] + cos(link_rot)*link_len
            # new_y = curr_haal.position_P[1] + sin(link_rot)*link_len
            # new_z = curr_haal.position_P[2] + link_off

            # rotate = qt.from_axisangle(link_rot,[1,0,0])
            # twist = qt.from_axisangle(link_twi,[0,0,1])

            # haal_frame.position_P = [new_x,new_y,new_z ]
            # haal_frame.rotation_Q  = curr_haal.rotation_Q*rotate*twist

        self.FORWARD = curr_matrix

    def do_ik(self,goal):
        
        trajectory = [self.joint_values]



        return trajectory


    def do_id(self,wrench_force):
        pass
    def compute_ws(self):
        pass
    def trajectory(goal1,goal2,optimization = 0):
        pass

        