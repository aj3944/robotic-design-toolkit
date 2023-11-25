import time
import tkinter
from OpenGL import GL
from pyopengltk import OpenGLFrame

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



from whiteroom import Room,thread_len1



pi = math.pi;



dh_table = [
    [ -pi/2, 10, 0 , -pi/2],
    [ -pi/2, 0, 0, pi/2 ],
    [ pi/2, 10 + 0, 20, 0],
    [ -pi/2, 0, 30, pi/2 ],
    [ pi/2, 10 + 10, 20, -pi/3],
    [ -pi/2, 0, 30, pi/2 ],
    [ pi/2, 10 + 0, 20, 0],
    [ -pi/2, 0, 0, pi/2 ],
    [ pi/2, 10 + 10, 20, 0],
]

joint_types =[ 0 for i in dh_table ] 
joint_values = [  0 for i in dh_table ] 

my_manip = mp()

my_manip.make_manip(dh_table,joint_types);


SCENE_1 = Scene()



SCENE_1.add_object(my_manip.thing)


# R = Room(SCENE_1)




class AppOgl(OpenGLFrame):
    # def __init__(self,Scene):
    #   self.scene = Scene

    def initgl(self):
        """Initalize gl states when the frame is created"""
        # GL.glViewport(0, 0, self.width, self.height)
        # GL.glClearColor(0.0, 1.0, 0.0, 0.0) 
        self.room = Room(SCENE_1)   
        self.start = time.time()
        self.nframes = 0

    def redraw(self):
        """Render a single frame"""
        self.room.update()


        for i in range(len(joint_values)):
            joint_values[i] += thread_len1*10       

        my_manip.set_joint_angles(joint_values);

        tm = time.time() - self.start
        self.nframes += 1
        print("fps",self.nframes / tm, end="\r" )

    def resize(self,event):
        print(event)

if __name__ == '__main__':
    root = tkinter.Tk()
    app = AppOgl(root, width=320, height=200)
    app.pack(fill=tkinter.BOTH, expand=tkinter.YES)
    app.animate = 1
    app.after(100, app.printContext)


    def onKeyPress(event):
        print('\t\tYou pressed %s\n' % (event.char, ))
        app.room.keyboard_funtion(event.char)
    def onConfigure(event):
        print('\t\tresized to(%d,%d)\n'%(event.width,event.height))
        app.room.resize(event.width,event.height)

    root.bind('<KeyPress>', onKeyPress)
    root.bind('<Configure>', onConfigure)
    app.mainloop()