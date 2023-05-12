from rtree import index
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
def cuboid_data(box):
    l = box[3] - box[0]
    w = box[4] - box[1]
    h = box[5] - box[2]
    x = [[0, l, l, 0, 0],
         [0, l, l, 0, 0],
         [0, l, l, 0, 0],
         [0, l, l, 0, 0]]
    y = [[0, 0, w, w, 0],
         [0, 0, w, w, 0],
         [0, 0, 0, 0, 0],
         [w, w, w, w, w]]
    z = [[0, 0, 0, 0, 0],
         [h, h, h, h, h],
         [0, 0, h, h, 0],
         [0, 0, h, h, 0]]
    return box[0] + np.array(x), box[1] + np.array(y), box[2] + np.array(z)

def sphere_data(sphere):
    cx, cy, cz = sphere[:3]
    r = sphere[3]
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = cx + r*np.cos(u)*np.sin(v)
    y = cy + r*np.sin(u)*np.sin(v)
    z = cz + r*np.cos(v)
    return x, y, z

class Node:
    node_list = []
    def __init__(self,coords):
        self.p = np.array(coords)
        self.parent = None
        self.unit_vec = None
        self.angle = 0
        self.cost = np.inf
        self.pt = False
        Node.node_list.append(self)

    def __len__(self):
        return len(self.p)

    def __getitem__(self, i):
        return self.p[i]

    def __repr__(self):
        return 'Node({}, {})'.format(self.p,self.cost)
    
class Tree:
  def __init__(self):
    self.node_list = []
    self.idx = self.get_tree()
    self.len = 0

  @staticmethod
  def get_tree():
    p = index.Property()
    p.dimension = 3
    p.dat_extension = 'data'
    p.idx_extension = 'index'
    return index.Index(properties=p)

  def add(self,new_node):
    self.node_list.append(new_node)
    self.idx.insert(self.len,(*new_node.p,))
    self.len += 1

  def nearest(self,node):
    near_ids = self.idx.nearest((*node.p,),1)
    id = list(near_ids)[0]
    return self.node_list[id]
  
  def k_nearest(self,node,k):
    near_ids = self.idx.nearest((*node.p,),k)
    for i in near_ids:
      yield self.node_list[i]

  def all(self):
    return self.node_list
    
class Map:
  def __init__(self,obstacle_list,bounds, path_resolution = 0.5):
    self.obstacles = obstacle_list
    self.idx = self.get_tree(obstacle_list)
    self.len = len(obstacle_list)
    self.path_res = path_resolution
    self.bounds = bounds
  @staticmethod
  def get_tree(obstacle_list):
    p = index.Property()
    p.dimension = 3
    ls = [(i, (*obj, ), None) for i, obj in enumerate(obstacle_list)]
    return index.Index(ls, properties = p)

  def add(self, obstacle):
    self.idx.insert(self.len,obstacle)
    self.obstacles.append(obstacle)
    self.len += 1

  def collision(self ,start, end):
    dist = np.linalg.norm(start-end)
    n = int(dist/self.path_res)
    points = np.linspace(start,end,n)
    for p in points:
      if self.idx.count((*p,)) != 0 :
          return True
    return False

  def plotobs(self,ax,scale = 1):
    obstacles = scale*np.array(self.obstacles)
    for obstacle in obstacles:
        if len(obstacle) == 6:
          X, Y, Z = cuboid_data(obstacle)
          ax.plot_surface(X, Y, Z, rstride=1, cstride=1,alpha = 0.2,zorder = 1)
        elif len(obstacle) == 2:
          x, y, z = sphere_data(obstacle)
          ax.plot_surface(x, y, z, rstride=1, cstride=1, alpha=0.2, zorder=1)

  def inbounds(self,p):

      lower,upper = self.bounds
      return (lower <= p).all() and (p <= upper).all()

  def draw_graph(self,ax, t_obj):
        for node in Node.node_list:
           if node.pt == True:
              ax.scatter(node[0], node[1],node[2], "-", s=10, color = (0.3, 0.3, 0.2, 0.9), zorder = 5)
        
        for node in t_obj.tree.all():
          if node.parent:
              xy = np.c_[node.p,node.parent.p]
              ax.plot(*xy, "-",color = (0.5, 0.5, 0.5, 0.9),zorder = 5)

  def plot_map(self, start, end,t_obj, path = None, ax = None, graph = True):
    
    if ax is None:
        fig = plt.figure(figsize=(30,20))
        ax = Axes3D.Axes3D(fig)
        ax.axes.set_xlim3d(left=0, right=100) 
        ax.axes.set_ylim3d(bottom=0, top=100) 
        ax.axes.set_zlim3d(bottom=0, top=100) 
    ax.scatter(float(start[0]), float(start[1]), float(start[2]), "-", s=80, color = (0.2, 0.9, 0.2, 0.9), zorder = 5)
    ax.scatter(float(end[0]), float(end[1]), float(end[2]), "-", s=60, color = (0.9, 0.2, 0.9, 0.9), zorder = 5)
    self.plotobs(ax)
    if graph: 
      self.draw_graph(ax, t_obj)
    if path is not None:

        path_x = (path.T)[0]
        path_y = (path.T)[1]
        path_z = (path.T)[2]

        prev_point_x = path_x[0]
        prev_point_y = path_y[0]
        prev_point_z = path_z[0]
        
        for i in range(len(path_x)):
            ax.plot([path_x[i], prev_point_x],[path_y[i], prev_point_y],[path_z[i], prev_point_z], '-', color = (0.9, 0.2, 0.5, 0.8), zorder = 5)
            ax.scatter(path_x[i], path_y[i], path_z[i], "-", color = (0.2, 0.2, 0.9, 0.9), zorder = 5)
            prev_point_x = path_x[i]
            prev_point_y = path_y[i]
            prev_point_z = path_z[i]

            # plt.pause(0.1)
    plt.show()