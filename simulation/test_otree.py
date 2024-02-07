from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

import numpy as np
import math
from quaternion import Quaternion as qt
from quaternion import Haal,Transformation

from bhram import Thing,Scene
from math import sin, cos, acos, sqrt,pi
import math 

from numpy import log as ln

from scipy.spatial.transform import Rotation as R
import random


from octree import OxTree as Otree


from bhram import Thing,Scene

import tkinter as tk


from whiteroom import Room,thread_len1
from tk_opengl import AppOgl

def randxyz(r = 100):
    x_f = (0.5 - random.random())*2*pi
    y_f = (0.5 - random.random())*2*pi
    # r = r * random.random();s


    exp_random = (0.5 - random.random())*5


    r_parabola = 0.1 + math.pow(r,exp_random)


    # z_f = (0.5 - random.random())*r
    # return x_f,y_f,z_f
    return r_parabola,r_parabola,r_parabola





small_o = Otree()

# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))
# small_o.add_point(*randxyz(1000))

tree_thing = Thing(small_o.draw,())




SCENE_1 = Scene()

SCENE_1.add_object(tree_thing)


def do_animation():
    # print("helloe")
    # print("helloe")
    # print("helloe")
    # print("helloe")
    # print("helloe")
    # print("helloe")
    # print("helloe")
    # print("helloe")
    small_o.add_point(*randxyz(200))
    pass



def onKeyPress(event):
    print('\t\tYou pressed %s\n' % (event.char, ))
    viz_robo.room.keyboard_funtion(event.char)



def onMotion(event):
    print('\t\tMotion %d %d\n' % (event.x, event.y))
    viz_robo.room.motion_funtion(event.x,event.y)


def onZoom(delta):
    # print('\tZoom %d %d\n' % (event.delta))
    # viz_robo.room.zoom_funtion(event.delta)
    IO = 1 if delta.num > 4.5  else -1;
    I2 = int(IO)
    print('\tZoom %f\n' % IO)
    if IO > 0:
        viz_robo.room.zoom_funtion(1)
    else:
        viz_robo.room.zoom_funtion(-1)



# <------------ APPLICATION LAYOUT ----------->
print("helloe")

# Application window
window = tk.Tk()
window.title("OCTREE TEST")
window.rowconfigure(0, minsize=400, weight=1)
window.rowconfigure(1, minsize=400, weight=1)
window.columnconfigure(1, minsize=800, weight=1)





def task():
    # if sviz_robo.room.animate:
    do_animation()
    window.after(20, task)  # reschedule event in 2 seconds
window.after(2000, task)



print("helloe")

viz_robo = AppOgl(window, width=320, height=200)
viz_robo.animate = 1

viz_robo.pack(fill=tk.BOTH, expand=tk.YES)
viz_robo.after(100, viz_robo.printContext)

viz_robo.set_scene(SCENE_1)
viz_robo.room.animate = 1



window.bind('<B1-Motion>', onMotion)
window.bind('<Button-5>', onZoom)
window.bind('<Button-4>', onZoom)


viz_robo.mainloop()

window.mainloop()



