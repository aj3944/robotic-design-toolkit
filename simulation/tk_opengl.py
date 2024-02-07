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



class AppOgl(OpenGLFrame):
    # def __init__(self,Scene):
    #   # self.scene = Scene()
    #   self.room = Room()

    def initgl(self):
        """Initalize gl states when the frame is created"""
        # GL.glViewport(0, 0, self.width, self.height)
        # GL.glClearColor(0.0, 1.0, 0.0, 0.0) 
        self.start = time.time()
        self.nframes = 0
        # self.room = Room()   

    def set_scene(self,SCENE):
        self.room = Room(SCENE)   

    def redraw(self):
        """Render a single frame"""
        self.room.update()


        # for i in range(len(joint_values)):
        #     joint_values[i] += thread_len1*10       

        # my_manip.set_joint_angles(joint_values);

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
    SCENE_1 = Scene()
    app.set_scene(SCENE_1)

    def onKeyPress(event):
        print('\t\tYou pressed %s\n' % (event.char, ))
        app.room.keyboard_funtion(event.char)
    def onConfigure(event):
        print('\t\tresized to(%d,%d)\n'%(event.width,event.height))
        app.room.resize(event.width,event.height)

    root.bind('<KeyPress>', onKeyPress)
    root.bind('<Configure>', onConfigure)
    app.mainloop()