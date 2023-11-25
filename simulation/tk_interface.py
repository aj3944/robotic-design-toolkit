
from OpenGL import GL
from pyopengltk import OpenGLFrame

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import sys
import math
import numpy as np
import time


import tkinter as tk
from tkinter.filedialog import askopenfilename, asksaveasfilename

from sklearn.preprocessing import normalize

from jax import grad,jacobian
from pyassimp import load

from quaternion import Quaternion as qt
from quaternion import Haal




from bhram import Thing,Scene
from bot import Manipulator as mp



from whiteroom import Room,thread_len1
from tk_opengl import AppOgl

# <================= FUNCTIONS =========================>

# VIZUALIZATIONS




# FILE I/O
def open_file():
    """Open a file for editing."""
    filepath = askopenfilename(
        filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
    )
    if not filepath:
        return
    txt_edit.delete(1.0, tk.END)
    with open(filepath, "r") as input_file:
        text = input_file.read()
        txt_edit.insert(tk.END, text)
    window.title(f"Text Editor Application - {filepath}")

def save_file():
    """Save the current file as a new file."""
    filepath = asksaveasfilename(
        defaultextension="txt",
        filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
    )
    if not filepath:
        return
    with open(filepath, "w") as output_file:
        text = txt_edit.get(1.0, tk.END)
        output_file.write(text)
    window.title(f"Text Editor Application - {filepath}")



# <------------ APPLICATION LAYOUT ----------->

# Application window
window = tk.Tk()
window.title("Robotics Design Toolkit")
window.rowconfigure(0, minsize=400, weight=1)
window.rowconfigure(1, minsize=400, weight=1)
window.columnconfigure(1, minsize=800, weight=1)
# window.columnconfigure(0, minsize=200, weight=1)

# VISUALIZATION AND BUTTONS

viz_robo = AppOgl(window, width=320, height=200)
# viz_robo.pack(fill=tk.BOTH, expand=tk.YES)
viz_robo.animate = 1
viz_robo.after(100, viz_robo.printContext)



viz_buttons = tk.Frame(window, relief=tk.RAISED, bd=2)
btn_fwd_kin = tk.Button(viz_buttons, text="FK", command=open_file)
btn_inv_kin = tk.Button(viz_buttons, text="IK", command=save_file)
btn_pause = tk.Button(viz_buttons, text="Pause", command=pause_sim)
btn_play = tk.Button(viz_buttons, text="Play", command=play_sim)
btn_reset = tk.Button(viz_buttons, text="Reset", command=reset_sim)

btn_fwd_kin.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
btn_inv_kin.grid(row=1, column=0, sticky="ew", padx=5)


btn_pause.grid(row=2, column=0, sticky="ew", padx=5)
btn_play.grid(row=3, column=0, sticky="ew", padx=5)
btn_reset.grid(row=4, column=0, sticky="ew", padx=5)



#  EDITOR AND BUTTONS
txt_edit = tk.Text(window)
fr_buttons = tk.Frame(window, relief=tk.RAISED, bd=2)
btn_open = tk.Button(fr_buttons, text="Open", command=open_file)
btn_save = tk.Button(fr_buttons, text="Save As...", command=save_file)

btn_open.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
btn_save.grid(row=1, column=0, sticky="ew", padx=5)



# ADD TO WINDOW
txt_edit.grid(row=0, column=1, sticky="nsew")
fr_buttons.grid(row=0, column=0, sticky="ns")


viz_robo.grid(row=1, column=1, sticky="nsew")
viz_buttons.grid(row=1, column=0, sticky="ns")





# APPLICATION INITIALIZATIONS & SETUPS


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




def onKeyPress(event):
    print('\t\tYou pressed %s\n' % (event.char, ))
    viz_robo.room.keyboard_funtion(event.char)


window.bind('<KeyPress>', onKeyPress)
viz_robo.mainloop()




# mainloop
window.mainloop()
