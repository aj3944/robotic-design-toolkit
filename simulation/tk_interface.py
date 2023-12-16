
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
from quaternion import Haal,Transformation

from scipy.spatial.transform import Rotation as R



from bhram import Thing,Scene
from bot import Manipulator as mp



from whiteroom import Room,thread_len1
from tk_opengl import AppOgl
from tkinter import *

# <================= FUNCTIONS =========================>

# VIZUALIZATIONS

# APPLICATION INITIALIZATIONS & SETUPS


pi = math.pi;


def deg2rad(a):
    return a*pi/180.


def rad2deg(a):
    return a*180/pi

# dh_string = """[
#     [ 0, 0, 20 , -pi/2],
#     [ 0, 0, 20 , 0],
# ]"""
# dh_string = """[
#     [ pi/2, 0, 0 , pi/2],
#     [ 0, 0, 40, 0 ],
#     [ 0, 0, 20, 0],
# ]"""
# dh_string = """[
#     [ 0, 7, 100 , -pi/2],
#     [ -pi/2, 7, 10, pi/2 ],
#     [ 0, -7, 200, 0],
#     [ -pi, -7, 30, pi/2 ],
#     [ pi/2, 7, 20, -pi],
#     [ -pi/2, 7, 10, pi/2 ],
# ]"""
dh_string = """[
    [ 0, 10, 0 , -pi/2],
    [ -pi/2, 0, 0, 0 ],
    [ pi/2, 10 + 0, 20, 0],
    [ 0, 0, 30, pi/2 ],
    [ 0, 10 + 10, 20, -pi/2],
    [ 0, 0, 30, pi/2 ],
    [ 0, 10 + 0, 20, 0],
]"""

dh_table = eval(dh_string)

joint_types =[ 0 for i in dh_table ] 
joint_values = [  0 for i in dh_table ] 

my_manip = mp()

my_manip.make_manip(dh_table,joint_types);


SCENE_1 = Scene()



SCENE_1.add_object(my_manip.thing)


# <ANIMATION LOOP>

def do_animation():
    global dh_table,joint_types,joint_values
    for i in range(len(joint_values)):
        joint_values[i] += thread_len1*10       

    my_manip.set_joint_angles(joint_values);
    my_manip.FORWARD = my_manip.do_fk(joint_values)

def make_ws():
    global dh_table,joint_types,joint_values
    my_manip.make_ws()
    my_manip.WORKSPACE.print()


def reset_sim():
    global dh_table,joint_types,joint_values
    joint_values =[ 0 for i in dh_table ] 
    my_manip.init();
    my_manip.make_manip(dh_table,joint_types);
    my_manip.set_joint_angles(joint_values);



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

viz_robo.set_scene(SCENE_1)


#  EDITOR AND BUTTONS

input_frame = tk.Frame(window)

btn_pause = tk.Button()
goal_fame = tk.Frame(input_frame)
#
dh_frame = tk.Frame(input_frame)

dh_frame.grid(row=0, column=0, sticky="nsew")
goal_fame.grid(row=1, column=0, sticky="nsew")

# dh_frame.grid_propagate(False)

label_0 = Label( dh_frame, text = "link_rot")
label_0.grid(row=0, column=1)
label_1 = Label( dh_frame, text = "link_off")
label_1.grid(row=0, column=2)
label_2 = Label( dh_frame, text = "link_len")
label_2.grid(row=0, column=3)
label_3 = Label( dh_frame, text = "link_twi")
label_3.grid(row=0, column=4)


text_edits_goals = []


label_A = Label( goal_fame, text = "GOAL POSITION")
label_A.grid(row=0, column=0)


label_B = Label( goal_fame, text = "Coordinate")
label_B.grid(row=0, column=1)


label_C = Label( goal_fame, text = "Euler XYZ")
label_C.grid(row=2, column=1)



GOAL_POSITION = Haal()
GOAL_POSITION.locate(40,40,40)

# GOAL_POSITION = Transformation()
        # pos = np.ndarray.flatten(fk_matrix[:-1,-1:])
        # rota = R.from_matrix(fk_matrix[:3,:3])
        # rot_vec = rota.as_rotvec(degrees = True)
        # theta = np.linalg.norm(rot_vec)
        # v = normalize(rot_vec)

for i in range(2): #POSITION + ORIENTATION 
    text_edits_goals.append([])
    for j in range(3): #3D Universe!
        text_edits_goals[i].append(0)


GOAL_LIST = GOAL_POSITION.return_list()


rot_goal = R.from_quat(GOAL_LIST[1])
rot_goal_euler = rot_goal.as_euler("XYZ",degrees=True)

GOAL_PARAMS = [
    GOAL_LIST[0],
    rot_goal_euler
]

def make_goal_axis():
    glPushMatrix()
    glColor3f(1.0, 0., 0.);
    glScalef(10,5,5)
    glutWireCube(1)
    glPopMatrix()

    glPushMatrix()        
    glColor3f(0., 1.0,0.);
    glScalef(1,5,6)
    glutWireCube(1)
    glPopMatrix()

    glPushMatrix()
    glColor3f(0., 0., 1.0);
    glScalef(1,6,5)
    glutWireCube(1)
    glPopMatrix()  
    

    glColor3f(1., 1., 1.);

_GOAL_SPHERE_ = Thing(make_goal_axis,())

_GOAL_SPHERE_.haal = GOAL_POSITION


SCENE_1.add_object(_GOAL_SPHERE_)


for i in range(2): #POSITION + ORIENTATION 
    for j in range(3): #3D Universe!
        text_edits_goals[i][j] = tk.Text(goal_fame, height = 1, width = 20)
        text_edits_goals[i][j].insert(tk.END, GOAL_PARAMS[i][j])
        text_edits_goals[i][j].grid(row=2*i+1, column=j+1)

text_edits_dh = []

for i in range(len(dh_table)):
    text_edits_dh.append([])
    for j in range(len(dh_table[0])):
        text_edits_dh[i].append(0)


for i in range(len(dh_table)):
    for j in range(len(dh_table[0])):
        text_edits_dh[i][j] = tk.Text(dh_frame, height = 1, width = 20)
        if j == 0 or j == 3:
            text_edits_dh[i][j].insert(tk.END, "{:.2f}".format(round(rad2deg(dh_table[i][j]), 2)) )
        else:
            text_edits_dh[i][j].insert(tk.END, dh_table[i][j])
        text_edits_dh[i][j].grid(row=i+1, column=j+1)


def clearFrame():
    # destroy all widgets from frame
    for widget in dh_frame.winfo_children():
       widget.destroy()



action_var = StringVar(viz_robo,"Play")


def pause_sim():
    viz_robo.room.animate = 0
    action_var = StringVar(viz_robo,"Play")

def play_sim():
    viz_robo.room.animate = 1
    action_var = StringVar(viz_robo,"Pause")

def action_toggle():
    if viz_robo.room.animate:
        pause_sim()
        btn_pause.config(text="Play")
    else:
        play_sim()
        btn_pause.config(text="Pause")

def load_dh():
    global dh_table,joint_types,joint_values
    txt_edit.delete('1.0', tk.END)
    # print(dir(txt_edit))
    txt_edit.insert(tk.END, dh_string)

    clearFrame()



def save_dh():
    global dh_table,joint_types,joint_values
    for i in range(len(dh_table)):
        for j in range(len(dh_table[0])):
            dh_param = float(text_edits_dh[i][j].get(1.0, tk.END))
            if j == 0 or j == 3:
                dh_table[i][j] = deg2rad(dh_param)
            else:
                dh_table[i][j] = dh_param
    # text = txt_edit.get(1.0, tk.END)
    # dh_string = text
    # dh_table = eval(dh_string)
    joint_types =[ 0 for i in dh_table ] 
    joint_values = [  0 for i in dh_table ] 
    my_manip.make_manip(dh_table,joint_types);




def save_goal():
    global _GOAL_SPHERE_
    for i in range(2): #POSITION + ORIENTATION 
        for j in range(3): #3D Universe!
            goal_param = float(text_edits_goals[i][j].get(1.0, tk.END))
            if i == 0:
                GOAL_PARAMS[i][j] = goal_param
            if i == 1:
                GOAL_PARAMS[i][j] = deg2rad(goal_param)

    GOAL_POSITION.locate(*GOAL_PARAMS[0])

    rota = R.from_euler("XYZ",GOAL_PARAMS[1])
    val = list(rota.as_quat())
    GOAL_POSITION.rotation_Q =  qt.from_value(val)
    _GOAL_SPHERE_.haal = GOAL_POSITION

    gol_r = rota.as_matrix();
    gol_p = GOAL_POSITION.position_P;

    t_mat = [
        [gol_r[0][0],gol_r[0][1],gol_r[0][2],gol_p[0]],
        [gol_r[1][0],gol_r[1][1],gol_r[1][2],gol_p[1]],
        [gol_r[2][0],gol_r[2][1],gol_r[2][2],gol_p[2]],
        [0          ,0          ,0          ,1       ]
    ]

    goal_tansformation = Transformation.from_value(t_mat)


    print(goal_tansformation)
    my_manip.set_goal(goal_tansformation);


def btn_draw_axes():
    my_manip.draw_axes_FLAG = not my_manip.draw_axes_FLAG

def btn_draw_frames():
    my_manip.draw_frames_FLAG = not my_manip.draw_frames_FLAG

def btn_draw_links():
    my_manip.draw_link_FLAG = not my_manip.draw_link_FLAG

def do_fk():
    my_manip.do_fk(joint_values,True)


def do_ik():
    trnx = Transformation()


    GOAL_QUAT = GOAL_POSITION.rotation_Q

    GOAL_AXISANGLE = GOAL_QUAT.get_axisangle() 

    ROTVEC = GOAL_AXISANGLE[0]*np.array(GOAL_AXISANGLE[1])
    rot_goal = R.from_rotvec(ROTVEC)

    r_mat = rot_goal.as_matrix()
    # GOAL_POSITION.position_P
    

    position = GOAL_POSITION.position_P
    
    trans_matrix = [
        [r_mat[0][0], r_mat[0][1], r_mat[0][2],  position[0]],
        [r_mat[1][0], r_mat[1][1], r_mat[1][2],  position[1]],
        [r_mat[2][0], r_mat[2][1], r_mat[2][2],  position[2]],
        [0 ,0 ,0 ,1]
    ]
    loss = my_manip.do_ik(Transformation.from_value(trans_matrix))
    print(loss)
def btn_draw_ws():
    my_manip.draw_WS = not my_manip.draw_WS



T_buttons = tk.Frame(window, relief=tk.RAISED, bd=2)
viz_buttons = tk.Frame(window, relief=tk.RAISED, bd=2)
btn_fwd_kin = tk.Button(T_buttons, text="FK", command=do_fk)
btn_inv_kin = tk.Button(T_buttons, text="IK", command=do_ik)
btn_workspace = tk.Button(T_buttons, text="WS", command=make_ws)
btn_pause = tk.Button(T_buttons, textvariable=action_var, command=action_toggle)
# btn_play = tk.Button(viz_buttons, text="Play", command=play_sim)
btn_reset = tk.Button(T_buttons, text="Reset", command=reset_sim)

btn_fwd_kin.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
btn_inv_kin.grid(row=1, column=0, sticky="ew", padx=5)
btn_workspace.grid(row=2, column=0, sticky="ew", padx=5)




# btn_play.grid(row=3, column=0, sticky="ew", padx=5)
btn_pause.grid(row=3, column=0, sticky="ew", padx=5)
btn_reset.grid(row=4, column=0, sticky="ew", padx=5)

btn_draw_axes = tk.Button(viz_buttons, text="Axes", command=btn_draw_axes)
btn_draw_frames = tk.Button(viz_buttons, text="Frames", command=btn_draw_frames)
btn_draw_links = tk.Button(viz_buttons, text="Links", command=btn_draw_links)
btn_draw_links = tk.Button(viz_buttons, text="WS", command=btn_draw_ws)



btn_draw_axes.grid(row=2, column=1, sticky="ew", padx=5)
btn_draw_frames.grid(row=3, column=1, sticky="ew", padx=5)
btn_draw_links.grid(row=4, column=1, sticky="ew", padx=5)




txt_edit = tk.Text(input_frame)
fr_buttons = tk.Frame(window, relief=tk.RAISED, bd=2)
btn_open = tk.Button(fr_buttons, text="Open", command=open_file)
btn_save = tk.Button(fr_buttons, text="Save As...", command=save_file)
# btn_DHV = tk.Button(fr_buttons, text="DH View", command=load_dh)
btn_DHS = tk.Button(dh_frame, text="DH Save", command=save_dh)
btn_GS = tk.Button(goal_fame, text="GOAL Save", command=save_goal)

btn_open.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
btn_save.grid(row=1, column=0, sticky="ew", padx=5)
# btn_DHV.grid(row=2, column=1, sticky="ew", padx=5)
btn_DHS.grid(row=0, column=5, sticky="ew", padx=5)
btn_GS.grid(row=0, column=5, sticky="ew", padx=5)


# txt_edit.pack(side = LEFT)

# ADD TO WINDOW
input_frame.grid(row=0, column=1, sticky="nsew")
fr_buttons.grid(row=0, column=0, sticky="ns")


viz_robo.grid(row=1, column=1, sticky="nsew")
viz_buttons.grid(row=1, column=2, sticky="ns")
T_buttons.grid(row=1, column=0, sticky="ns")




def task():
    if viz_robo.room.animate:
        do_animation()
    window.after(200, task)  # reschedule event in 2 seconds

window.after(2000, task)



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


def onResize(event):
    # print('\tZoom %d %d\n' % (event.delta))
    # viz_robo.room.zoom_funtion(event.delta)
    print('Resize to (%d,%d)\n' % (event.width,event.height))
    viz_robo.room.resize(event.width,event.height)




# window.bind('<Configure>', onResize)


window.bind('<KeyPress>', onKeyPress)
window.bind('<B1-Motion>', onMotion)
# window.bind('<>', onZoom)
window.bind('<Button-5>', onZoom)
window.bind('<Button-4>', onZoom)

# window.bind('<Button-4>', lambda event: onZoom(int(-1*(event.delta/120))))
# window.bind('<Button-5>', lambda event: onZoom(int(-1*(event.delta/120))))

# window.bind('<B2-Motion>', onMotion)
# window.bind('<B3-Motion>', onMotion)
viz_robo.mainloop()


save_goal()

# mainloop
window.mainloop()
