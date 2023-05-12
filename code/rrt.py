import numpy as np
from rtree import index
from math import ceil
from visualize_rrt import *
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D

class RRT:
    def __init__(self, start, goal, Map,
                    max_length = (20.0*np.sin(np.pi/6)),
                    goal_sample_rate = 0.05,
                    max_iter = 100 ):
        self.start = Node(start)
        self.goal = Node(goal)
        self.max_length = max_length
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.tree = Tree()
        self.map = Map
    
    def plan(self):
        self.tree.add(self.start)
        vec = np.array([-1,1,1])
        vec_z_axis = vec / np.linalg.norm(vec)

        self.start.unit_vec = vec_z_axis
        nearest_list = []
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()  
            nearest_node = self.tree.nearest(rnd_node)
            nearest_list.append(nearest_node.p)
            nearest_node.isnearest = True
            new_node = self.steer(nearest_node,rnd_node) 
            if not self.map.collision(nearest_node.p,new_node.p):
              self.tree.add(new_node)
              if self.dist(new_node,self.goal) <= self.max_length:
                  if not self.map.collision(new_node.p,self.goal.p):
                      self.goal.parent = new_node
                      return self.final_path()
        return nearest_list

    @staticmethod
    def dist(from_node, to_node):
        return np.linalg.norm(from_node.p - to_node.p)

    def steer(self,from_node, to_node):
        dist = self.dist(from_node, to_node)
        if dist > self.max_length:
            diff = from_node.p - to_node.p
            to_node.p  = from_node.p - diff/dist * self.max_length
        to_node.parent = from_node
        return to_node

    def get_random_node(self):
        if np.random.rand() > self.goal_sample_rate:
            rnd = np.random.rand(3)*100
        else:
            rnd = self.goal.p
        random_pt = Node(rnd)
        random_pt.pt = True
        return random_pt

    def final_path(self):
        path = []
        node = self.goal
        if (node.p == node.parent.p).all(): node = node.parent
        while node.parent is not None:
          path.append(node.p)
          node = node.parent
        path.append(self.start.p)
        return np.array(path[::-1])
