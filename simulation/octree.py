from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *


import random
import math
import numpy as np


def is_one(x):
	if not x == 1:
		return 0
	return 1;

def octane(v):
	a = "FRONT " if is_one(v[0]) else "BACK "
	b = "RIGHT " if is_one(v[1]) else "LEFT "
	c = "TOP " if is_one(v[2]) else "BOTTOM "
	return a + b + c





indexed = lambda vec: sum(is_one(c) * (2 ** i) for i, c in enumerate(vec[::-1]));
def deindexed(x):
	vex = [-1,-1,-1];
	po2 = int(math.log2(0.001 + x));
	# print("po2",po2)
	for i in range(po2,-1,-1):
		# print("x:",x)
		if 2**i <= x:
			x -= 2**i
			vex[i] = 1
		else:
			vex[i] = -1
	# print(vex)

	return [vex[x] for x in range(len(vex)-1,-1,-1)]


text_v = [1,-1,1]

print(indexed(text_v))
# print((indexed(text_v)))
print(deindexed(indexed(text_v)))

assert deindexed(indexed(text_v)) == text_v
print(octane(deindexed(indexed(text_v))))


class o_tree(object):
	depth = 0
	point = [0,0,0]
	root = []
	OCTREE = []
	leaf = True
	branch = False
	num_pts = 0
	def __str__(self):
		return "<<D:{:} N:{:} S:{:}>>  ".format(
			self.num_pts,self.depth,self.size)

	def __repr__(self):
		return "<<D:{:} N:{:} S:{:}>> ".format(
			self.depth,self.num_pts,self.size)
	def __init__(self,g_depth = 0,g_coords = [0,0,0],size = 1000,parent = None):
		self.depth = g_depth
		self.root = g_coords;
		self.num_pts = 0;
		self.size = size;
		self.OCTREE = [0 for i in range(8)]
		self.PARENT = parent
	def split(self):
		sz = self.size/4;
		for i in range(8):
			vx = deindexed(i)
			c_r_d_vector = vx
			self.OCTREE[i] = o_tree(self.depth + 1, [self.root[0]+c_r_d_vector[0]*sz,self.root[1]+c_r_d_vector[1]*sz,self.root[2]+c_r_d_vector[2]*sz], sz*2,self)
	def supersplit(self,c_r_d_self):
		sz = self.size;

		print("<------------SUPERSPLIT-------------->")

		print(c_r_d_self)

		disp_from_parent = np.array([self.root[0]+c_r_d_self[0]*sz,self.root[1]+c_r_d_self[1]*sz,self.root[2]+c_r_d_self[2]*sz]);
		new_root = np.array(self.root) + disp_from_parent;

		self.PARENT = o_tree(self.depth - 1, [new_root[0]-c_r_d_self[0]*sz,new_root[1]-c_r_d_self[1]*sz,new_root[2]-c_r_d_self[2]*sz], sz*2)

		# self.PARENT
		self.PARENT.OCTREE[indexed((-c_r_d_self[0],-c_r_d_self[1],-c_r_d_self[2]))] = self

		return self.PARENT.depth

		# for i in range(8):
			# vx = deindexed(i)
			# c_r_d_vector = vx
			# self.OCTREE[i] = o_tree(self.depth - 1, [new_root[0]+c_r_d_vector[0]*sz,new_root[1]+c_r_d_vector[1]*sz,new_root[2]+c_r_d_vector[2]*sz], sz*2)
	def add_point(self,point):
		self.num_pts = self.num_pts + 1;
		# bounds_vector =  [ 1 if point[i] > self.root[i] else -1  for i in range(len(point))]
		c_r_d_vector =  [ 1 if point[i] > self.root[i] else -1  for i in range(len(point))]
		# print("added:({:07.3f},{:07.3f},{:07.3f}) , num_pts {:} from depth {:} of size {:07.3f} at coordinates:({:07.3f},{:07.3f},{:07.3f}) ".format(
			# *point,self.num_pts,self.depth,self.size,*self.root))

		bounded = 0 if np.linalg.norm(np.array(point) - np.array(self.root) ) > self.size else 1

		if not bounded:
			self.supersplit(c_r_d_vector)
			parent_depth =  self.PARENT.add_point(point)
			return parent_depth
		# all_bounded = [1 for i in range(6)]
		# for i in range(8):
		# 	vx = deindexed(i)
		# 	for j in range(3):


		if self.leaf:
			self.point = point
			self.leaf = False;
			return self.depth


		if self.OCTREE[indexed(c_r_d_vector)] == 0:
			self.split()
		

		child_depth = self.OCTREE[indexed(c_r_d_vector)].add_point(point)

		return child_depth



			# sz = self.size/2;
			# # self.OCTREE[indexed(c_r_d_vector)] = o_tree(self.depth + 1, [self.root[0]+c_r_d_vector[0]*sz,self.root[1]+c_r_d_vector[1]*sz,self.root[2]+c_r_d_vector[2]*sz], sz)
			# # print("LEAF_ADD")
			# self.leaf = False
		# 	# self.point = point
		# 	# return self.depth
		# else:
		# 	# print("BRANCH_ADD")
		# 	# self.leaf = False;
		# 	# self.branch = True;
		# 	# if self.root == []:
		# 	# self.root = self.point
		# 	# else:
		# 	# 	self.root
		# 		# for i in range(len(self.com)):
		# 		# 	self.com[i] =  self.com[i]*(self.num_pts - 1)/self.num_pts + point[i]/self.num_pts
		# 	# self.OCTREE[indexed(c_r_d_vector)].split()
		# 	# self.OCTREE[indexed(c_r_d_vector)].add_point(self.point)
		# 	return self.OCTREE[indexed(c_r_d_vector)].add_point(point)
	def print(self):
		for i in self.OCTREE:
			if i and (i.leaf or i.branch):
				i.print()
		print("ROOT: ",self.depth, self.root,self.OCTREE)
	def draw(self):
		for i in self.OCTREE:
			if i:
				i.draw()
		# if self.leaf:	
		glPushMatrix()
		glColor3f(0.0, 1.0, 0.);
		glTranslatef(*self.root)
		glutWireCube(self.size)
		glPopMatrix()
		# if self.leaf:	
		glPushMatrix()
		glColor3f(1.0, 0., 0.);
		glTranslatef(*self.point)
		glutSolidCube(0.1)
		glPopMatrix()
class OxTree(object):
	def __init__(self):
		self.depth = 1;
		self.span = 1000;
		self.nodes = [];
		self.num_nodes = [];
		self.OTREE = o_tree(0,[0,0,0],self.span)
		self.root_depth = 0
	def init(self):
		self.nodes = [];
		self.num_nodes = [];
		self.OTREE = o_tree(0,[0,0,0])
		self.root_depth = 0
	def add_point(self,x,y,z):
		self.nodes.append((x,y,z));
		depth_added = self.OTREE.add_point([x,y,z]);
		self.num_nodes.append(1)

		print(depth_added)
		if depth_added < self.root_depth:
			self.root_depth = depth_added
			self.OTREE = self.OTREE.PARENT
		# if depth_added - len(self.num_nodes) == 1:
		# 	self.num_nodes.append(1)
		# if depth_added - len(self.num_nodes) > 1:
		# 	print("OVERFLOW")
		# else:
		# 	# print(depth_added)
		# 	self.num_nodes[depth_added-1] = self.num_nodes[depth_added-1] + 1;
	def add_point_flat(self,x,y,z):
		# self.nodes.append((x,y,z));
		pass
	def draw(self):
		# print("DRAW OXTRE")
		self.print();
		self.OTREE.draw();
	def draw_flat(self):
		for p in self.nodes:
			glPushMatrix()
			glColor3f(0.3,0.9,0.6);
			glTranslatef(*p)
			glutWireCube(1)
			glPopMatrix()			
	def print(self):
		print("--------OXTREE-------")
		# print("NODES:",self.nodes)
		print("NUM:",self.num_nodes)
		self.OTREE.print()




