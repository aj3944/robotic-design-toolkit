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
import random


from octree import OxTree as Otree
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
        self.draw_WS = True;
        self.c_quadric = gluNewQuadric()
        self.s_quadric = gluNewQuadric()
        self.WORKSPACE = Otree()
    def init(self):
        self.joint_values = []
        self.joint_limits = []
        self.links = []
        self.frames = []
        self.DH_PARAMETERS = []
        self.WORKSPACE = Otree()
        self.FORWARD = Transformation()
        self.GOAL_POSITION = []
        self.JACOBIAN = []
        self.TORQUES = []
        self.MASSES = []
        self.GOAL = Transformation()
        self.MOMENTS = []
        self.O_T_i = []
    def calculate_jacobian(self):
        self.do_fk(self.joint_values,True);
        print(self.O_T_i)
        init_pos = self.FORWARD;
        self.JACOBIAN = []
        DOF = len(self.DH_PARAMETERS)
        for i in range(DOF):
            J_i = []
            T_i = self.O_T_i[i].T_matrix()
            T_n = self.O_T_i[DOF - 1].T_matrix()
            Z_cap_i = np.ndarray.flatten(T_i[:-1,2:3]);
            P_n = np.ndarray.flatten(T_n[:-1,3:4]);
            P_i = np.ndarray.flatten(T_i[:-1,3:4]);
            print(Z_cap_i,P_n,P_i)
            # for p in range(3):
            J_i.extend(np.cross(Z_cap_i,P_n - P_i))
            # for r in range(3):
            J_i.extend(Z_cap_i)
            self.JACOBIAN.append(J_i)
        print(self.JACOBIAN)
    def calculate_langrangian(self):
        pass
    def make_manip(self,DH_TABLE,JOINT_TYPES):
        
        self.init()
        
        for i in range(len(DH_TABLE)):
            self.frames.append(Haal())
            # if i > 0 :
            self.joint_values.append(0)
            self.joint_limits.append([-math.pi/2,math.pi/2])
            self.links.append(JOINT_TYPES[i - 1])
            self.DH_PARAMETERS.append(DH_TABLE[i])
            self.TORQUES.append(0)
            self.MASSES.append(1)
            self.MOMENTS.append(1)
        
        self.calculate_jacobian()
    def set_joint_angles(self,joint_angles):
        if not len(joint_angles) == len(self.joint_values):
            return 
        else:
            self.joint_values = joint_angles
    def random_joint_values(self):
        def rajl(ll, ul):
            r = random.random()
            a = ll + r*(ul-ll)
            # print(a)
            return a
        return [ rajl(*self.joint_limits[i]) for i in range(len(self.joint_limits))]
    def draw_fk(self,transformation):
        # print(transformation.T_matrix())
        t_matrix = transformation.T_matrix()
        fk_matrix = np.array(t_matrix);
        pos = np.ndarray.flatten(fk_matrix[:-1,-1:])
        rota = R.from_matrix(fk_matrix[:3,:3])
        rot_vec = rota.as_rotvec(degrees = True)
        theta = np.linalg.norm(rot_vec)
        v = normalize(rot_vec)
        glPushMatrix()
        glTranslatef(pos[0],pos[1],pos[2])
        glRotatef(theta,v[0],v[1],v[2])
        glColor3f(1.0, 1.0, 0.);
        glRotatef(90,0,1.,0.)
        glTranslatef(-2.,0.,0)
        xc2 = gluNewQuadric()
        gluCylinder(xc2,1,0.1,3,16,1);
        glTranslatef(4.,0.,0)
        gluCylinder(xc2,1,0.1,3,16,1);
        glPopMatrix()
    def draw_fk_sim(self,transformation):
        # print(transformation.T_matrix())
        t_matrix = transformation.T_matrix()
        fk_matrix = np.array(t_matrix);
        pos = np.ndarray.flatten(fk_matrix[:-1,-1:])
        rota = R.from_matrix(fk_matrix[:3,:3])
        rot_vec = rota.as_rotvec(degrees = True)
        theta = np.linalg.norm(rot_vec)
        v = normalize(rot_vec)
        glPushMatrix()
        glTranslatef(pos[0],pos[1],pos[2])
        glRotatef(theta,v[0],v[1],v[2])
        glColor3f(1.0, 1.0, 0.);
        glRotatef(90,0,1.,0.)
        glTranslatef(-2.,0.,0)
        glutWireCube(1)
        glPopMatrix()
    def make_ws(self):
        for i in range(2000):
            transformation = self.do_fk(self.random_joint_values())
            fk_transform = transformation.T_matrix();
            point = list(np.ndarray.flatten(fk_transform[:-1,-1:]));
            self.WORKSPACE.add_point_flat(*point)
    def draw_ws(self):
        self.WORKSPACE.draw_flat()
    def draw_func(self):
        if self.draw_FK :
            self.draw_fk(self.FORWARD);
        
        if self.draw_WS:
            self.draw_ws();

        for i in range(len(self.DH_PARAMETERS)):
            link_rot = (self.DH_PARAMETERS[i][0] + self.joint_values[i])*rad_2_deg
            link_off = self.DH_PARAMETERS[i][1]            
            link_len = self.DH_PARAMETERS[i][2]
            link_twi = self.DH_PARAMETERS[i][3]*rad_2_deg


            glRotatef(link_rot,0,0.,1.)
            glTranslatef(0.,0.,link_off)
            glTranslatef(link_len/2,0.,0.)
            glRotatef(link_twi,1,0,0)
            # glTranslatef(0.,0.,link_off/2.)
            glPushMatrix()

            self.draw_link(link_len,i,link_off,link_twi);

            glPopMatrix()
            glTranslatef(link_len/2,0.,0.)


    def draw_link(self,link_len,i=0,link_off= 2,link_twi = 0):

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
            glutSolidCube(1)
            glPopMatrix()

            glPushMatrix()        
            glRotatef(-link_twi,1,0,0)
            glTranslatef(-link_len/2,0.,0.)
            glTranslatef(0.,0.,-link_off/2)
            glColor3f(1., 1.0,1.);
            glScalef(5.,5.,link_off)
            glutWireCube(1)
            glPopMatrix()


        # glutSolidCube(1)
    def do_fk(self,joint_values,update = False):
        frames = []
        orig_matrix = Transformation()
        curr_matrix = Transformation()
        # print("DHTABLE")
        # print(self.joint_values)
        # print(self.DH_PARAMETERS)
        self.O_T_i = [0 for i in range(len(self.DH_PARAMETERS))]
        for i in range(len(self.DH_PARAMETERS)):

            # print(self.joint_values)
            theta = self.DH_PARAMETERS[i][0] + joint_values[i]
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

            t_i = Transformation()
            t_i.rotation = R.from_matrix(prev_transform[:3,:3]);
            t_i.translation = list(np.ndarray.flatten(prev_transform[:-1,-1:]));

            self.O_T_i[i] = t_i;
            new_transform = np.matmul(prev_transform,frame_transform);
    
            curr_matrix.rotation = R.from_matrix(new_transform[:3,:3]);
            curr_matrix.translation = list(np.ndarray.flatten(new_transform[:-1,-1:]));


            # print("<-----------{0}------------------>\n".format(i));
            # print(new_transform);
            # print("<------------------------------->\n");

            # new_x = curr_haal.position_P[0] + cos(link_rot)*link_len
            # new_y = curr_haal.position_P[1] + sin(link_rot)*link_len
            # new_z = curr_haal.position_P[2] + link_off

            # rotate = qt.from_axisangle(link_rot,[1,0,0])
            # twist = qt.from_axisangle(link_twi,[0,0,1])

            # haal_frame.position_P = [new_x,new_y,new_z ]
            # haal_frame.rotation_Q  = curr_haal.rotation_Q*rotate*twist
        self.FORWARD = curr_matrix
        return curr_matrix
    def del_angs(self,goal):
        pass
    def do_ik(self,goal):
        for i in range(100):
            # loss = self.get_next_point( 1 - (i/2000))
            loss = self.get_next_point( 0.01)
        return loss
    def get_next_point(self,step = 0.01):
        # random_axis = int(random.random()*len(self.DH_PARAMETERS));

        # random_amount = random.random()*step;

        # print("RANDOM AXIS",random_axis);
        # print("RANDOM AMOUNT",random_amount);

        rands = [random.random()*step for i in self.joint_values]

        random_joint_value = [ i + j for i,j in zip(self.joint_values,rands)];
        random_joint_value_conj = [ i - j for i,j in zip(self.joint_values,rands)];

        # print("JOINTVALUES (CURR,RANDOM):",self.joint_values,random_joint_value)


        conj_fk = self.do_fk(random_joint_value_conj).T_matrix()
        conj_point =  np.ndarray.flatten(conj_fk[:-1,-1:]);
        conj_orientation =  conj_fk[:3,:3]


        random_fk = self.do_fk(random_joint_value).T_matrix()
        random_point =  np.ndarray.flatten(random_fk[:-1,-1:]);
        random_orientation =  random_fk[:3,:3]

        curr_matrix = self.do_fk(self.joint_values).T_matrix()       
        curr_point = np.ndarray.flatten(curr_matrix[:-1,-1:]);
        curr_orientation =  curr_matrix[:3,:3]

        curr_pot = self.get_potential(curr_point,curr_orientation);
        random_pot = self.get_potential(random_point,random_orientation);
        conj_pot = self.get_potential(conj_point,conj_orientation);

        print("POTENTIAL:",curr_pot)
        # print("POINT (CURR,RANDOM):",curr_point,random_point)

        # print("POTENTIAL (CURR,RANDOM):",curr_pot,random_pot)

        if curr_pot > random_pot:
            print("WENT FORWAD")
            self.set_joint_angles(random_joint_value)
            loss = random_pot
        else:
            print("WENT CONJUGATE")
            self.set_joint_angles(random_joint_value_conj)         
            loss = conj_pot
        return loss 
    def set_goal(self,goal):
        self.GOAL = goal;  
        print("<<<<<<<GOAL UPDATED>>>>>>",self.GOAL)
    def get_potential(self,point,rota_matx):
        goal_matrix = self.GOAL.T_matrix();
        # print(goal_matrix)
        goal_point = np.ndarray.flatten(goal_matrix[:-1,-1:]);
        dist = np.linalg.norm((goal_point - point));

        goal_orientation = np.ndarray.flatten(goal_matrix[:3,:3]);


        Q1 = qt.from_MATRIX(goal_orientation);

        Q2 = qt.from_MATRIX(rota_matx);
        rota_dist = Q1 - Q2;




        return -1/(dist + rota_dist + 0.00001);
    def do_id(self,wrench_force):
        pass
    def compute_ws(self):
        pass
    def trajectory(goal1,goal2,optimization = 0):
        pass

        