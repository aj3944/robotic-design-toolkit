from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *


import random


class o_tree(object):
	depth = 0
	com = [0,0,0]
	root = []
	OCTREE = []
	leaf = False
	branch = False
	num_pts = 0
	def __init__(self,g_depth = 0,g_coords = [0,0,0],size = 1000):
		self.depth = g_depth
		self.root = g_coords;
		self.num_pts = 0;
		self.size = size;
		self.OCTREE = [0 for i in range(8)]
	def add_point(self,point):
		self.num_pts = self.num_pts + 1;
		indexed = lambda vec: sum(int(c) * (2 ** i) for i, c in enumerate(vec[::-1]));
		c_r_d_vector =  [ 1 if point[i] > self.root[i] else -1  for i in range(len(point))]
		# print("CRD VECTOR",c_r_d_vector)
		if self.OCTREE[indexed(c_r_d_vector)] == 0:
			sz = self.size/2;
			self.OCTREE[indexed(c_r_d_vector)] = o_tree(self.depth + 1, [self.root[0]+c_r_d_vector[0]*sz,self.root[1]+c_r_d_vector[1]*sz,self.root[2]+c_r_d_vector[2]*sz], sz)
			# print("LEAF_ADD")
			self.leaf = True
			self.com = point
			return self.depth
		else:
			# print("BRANCH_ADD")
			self.leaf = False;
			self.branch = True;
			if self.root == []:
				self.root = point
			else:
				for i in range(len(self.com)):
					self.com[i] =  self.com[i]*(self.num_pts - 1)/self.num_pts + point[i]/self.num_pts
			return self.OCTREE[indexed(c_r_d_vector)].add_point(point)
	def print(self):
		for i in self.OCTREE:
			if i and (i.leaf or i.branch):
				i.print()
		# print("ROOT: ",self.depth, self.root,self.OCTREE)
	def draw(self):
		for i in self.OCTREE:
			if i and (i.leaf or i.branch):
				i.draw()
		# print(self.com)
		if self.branch:	
			glPushMatrix()
			glColor3f(0.0, 1.0, 0.);
			glTranslatef(*self.root)
			glutWireCube(self.size)
			glPopMatrix()
		# if self.leaf:	
		# 	glPushMatrix()
		# 	glColor3f(0, 1.0, 0.);
		# 	glTranslatef(*self.com)
		# 	glutWireCube(1)
		# 	glPopMatrix()
class OxTree(object):
	def __init__(self):
		self.depth = 1;
		self.span = 100;
		self.nodes = [];
		self.num_nodes = [];
		self.OTREE = o_tree(1,[0,0,0],self.span)
	def init(self):
		self.nodes = [];
		self.num_nodes = [];
		self.OTREE = o_tree(1,[0,0,0])
	def add_point(self,x,y,z):
		self.nodes.append((x,y,z));
		depth_added = self.OTREE.add_point([x,y,z]);
		if depth_added - len(self.num_nodes) == 1:
			self.num_nodes.append(1)
		if depth_added - len(self.num_nodes) > 1:
			print("OVERFLOW")
		else:
			# print(depth_added)
			self.num_nodes[depth_added-1] = self.num_nodes[depth_added-1] + 1;
	def add_point_flat(self,x,y,z):
		self.nodes.append((x,y,z));
	def draw(self):
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
		# print("NUM:",self.num_nodes)
		# self.OTREE.print()
