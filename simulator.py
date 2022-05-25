import tkinter as tk
import numpy as np
from math import radians, cos, sin, asin, pi, exp, sqrt, inf
from copy import deepcopy
PI2 = pi / 2

width=1000
height=700

MAX_THROTTLE = 100
MIN_THROTTLE = 0

old = -1

ortho = (1.0 /sqrt(6)) * np.matrix([[sqrt(3),0,-sqrt(3)],
                                   [1,2,1],
                                   [sqrt(2),-sqrt(2),sqrt(2)]])

def CLAMP(x,lower,upper):
    return min(upper,max(x,lower))

def to_ss(vec3):
    ss = (ortho * np.reshape(vec3,(-1,1)))[:2]
    ss[0] *= (width >> 1)
    ss[1] *= (height >> 1)
    return [int(ss[0]) + (width >> 1), height - (int(ss[1]) + (height >> 3))]

def rotate_about(vec3_pt,vec3_relative,pitch,yaw,roll):
    vec3_pt = np.reshape(vec3_pt,(-1,1))
    vec3_relative = np.reshape(vec3_relative,(-1,1))
    
    #move to origin
    vec3_pt = vec3_pt - vec3_relative

    #generate rotation matrix
    

    y = radians(pitch)
    b = radians(roll)
    a = radians(yaw)
    
    rot = np.matrix([
        [cos(b)*cos(y), sin(a)*sin(b)*cos(y)-cos(a)*sin(y), cos(a)*sin(b)*cos(y)+sin(a)*sin(y)],
        [cos(b)*sin(y), sin(a)*sin(b)*sin(y)+cos(a)*cos(y), cos(a)*sin(b)*sin(y)-sin(a)*cos(y)],
        [-sin(b),       sin(a)*cos(b),                      cos(a)*cos(b)]
    ])
    
    vec3_pt = rot * vec3_pt

    #move back
    res = np.reshape(vec3_pt + vec3_relative, (1,-1))
    return res

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def reset_globals():
    global centroid, offs
    centroid = np.array([[0.5,1.0,0.5]])
    jet.pitch=0
    jet.roll=0
    jet.yaw=0
    jet.throttle=50
    offs = deepcopy(offs_b)
    
    
    for i in range(len(offs)): #rotate about initial configuration
        offs[i] = scale(offs[i],list(centroid[0]),0.5)
        offs[i] = rotate_about(offs[i],centroid,jet.pitch,jet.yaw,jet.roll)
    
def scale(a, b, s):
    diff = [(aa - bb) * s for aa, bb in zip(a,b)] 
    return [bb + ddiff for bb, ddiff in zip(b, diff)]

def draw_loop():
    global jet
    
    jet.pitch = (jet.pitch + jet.dpitch) % 360
    jet.yaw   = (jet.yaw   + jet.dyaw)   % 360
    jet.roll  = (jet.roll  + jet.droll)  % 360
    jet.throttle = CLAMP(jet.throttle + jet.dthrottle, 0, 100)
    
    #print(pitch,yaw,roll,throttle)

    #move in worldspace
    global centroid
    direc = normalize(np.array(offs[3]) - centroid)
    centroid += direc * 0.00004 * CLAMP(jet.throttle - 50,0,inf)
    #print(type(centroid))
    
    #bounds
    #hit a wall?
    if centroid[0][0] < -0.5:
        print("Hit -X")
        reset_globals()
    elif centroid[0][0] > 1.0:
        print("Hit +X")
        reset_globals()
    elif centroid[0][1] < 0.0:
        print("Hit -Y")
        reset_globals()
    elif centroid[0][1] > sqrt(2):
        print("Hit +Y")
        reset_globals()
    elif centroid[0][2] < -0.5:
        print("Hit -Z")
        reset_globals()
    elif centroid[0][2] > 1.0:
        print("Hit +Z")
        reset_globals()

    for i in range(len(offs)):
        offs[i] += direc * 0.00004 * CLAMP(jet.throttle - 50,0,inf)

    #maybe use this?
    depth_factor = sqrt((centroid[0][0] * centroid[0][0]) + 
                        (centroid[0][2] * centroid[0][2]) )
    print(centroid, jet.throttle, depth_factor)
       
    #plane center, don't rotate
    ss2 = to_ss(centroid)
    canv.coords(centroid_dot,ss2[0]-5,ss2[1]-5,ss2[0]+5,ss2[1]+5)
       
    for idx, (off_dot, off) in enumerate(zip(off_dots, offs)):
        off2 = (np.array(off - centroid))
        
        ss = rotate_about(off,centroid,jet.dpitch,jet.dyaw,jet.droll)
        offs[idx] = ss
        ss = to_ss(ss)
        
        #line to plane center
        canv.coords(plane_lines[idx], ss2+ss)
        
        canv.coords(off_dot,ss[0]-5,ss[1]-5,
                            ss[0]+5,ss[1]+5)
    
    canv.coords(plane_lines[5],to_ss(offs[0])+to_ss(offs[4]))
    canv.after(10,draw_loop)

class Jet:
    
    def keydown_gen(self,keys):
        def keydown(e):
            nonlocal self
        
            if e.char == keys[0]:
                self.droll = -1
            elif e.char == keys[1]: 
                self.droll = 1
            elif e.char == keys[2]:
                self.dpitch = -1
            elif e.char == keys[3]:
                self.dpitch = 1
            elif e.char == keys[4]:
                self.dyaw = -1
            elif e.char == keys[5]:
                self.dyaw = 1
            elif e.char == keys[6]:
                self.dthrottle = -1
            elif e.char == keys[7]:
                self.dthrottle = 1
        return keydown

    def keyup_gen(self,keys):
        def keyup(e):
            nonlocal self 
            
            if e.char == keys[0] and self.droll == -1:
                self.droll = 0
            elif e.char == keys[1] and self.droll == 1: 
                self.droll = 0
            elif e.char == keys[2] and self.dpitch == -1:
                self.dpitch = 0
            elif e.char == keys[3] and self.dpitch == 1:
                self.dpitch = 0
            elif e.char == keys[4] and self.dyaw == -1:
                self.dyaw = 0
            elif e.char == keys[5] and self.dyaw == 1:
                self.dyaw = 0
            elif e.char == keys[6] and self.dthrottle == -1:
                self.dthrottle = 0
            elif e.char == keys[7] and self.dthrottle == 1:
                self.dthrottle = 0
        return keyup 

    

    def __init__(self, root, canv, keys):
        root.bind("<KeyPress>",   self.keydown_gen(keys))
        root.bind("<KeyRelease>", self.keyup_gen(keys))
        self.pitch=0
        self.roll=0
        self.yaw=0
        self.throttle=50

        self.dpitch=0
        self.dyaw=0
        self.droll=0
        self.dthrottle=0

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry('%dx%d' % (width,height) )
    root.title("Airshow")
    root.resizable(False,False)
    
    canv = tk.Canvas(root, height=height, width=width, bg='black')
    keys = ['a','d','s','w','j','l','k','i']
    jet = Jet(root,canv,keys)
    canv.grid()

    grid_points = [[0,0,0],[1,0,0],[1,0,1],[0,0,1]]
    grid_points = [to_ss(point) for point in grid_points]
    floor = canv.create_polygon(
        grid_points[0]+grid_points[1]+grid_points[2]+grid_points[3],
    fill='green')

    centroid = [0.5,1.0,0.5]
    centroid_dot = canv.create_oval(0,0,0,0,fill='red')
    
    offs_b = [
        [0.5,1.0,0.7], #main points
        [0.6,1.0,0.5],
        [0.4,1.0,0.5],
        [0.5,1.0,0.2],
        
        [0.5,1.1,0.7]
    ]
    
    offs = deepcopy(offs_b)
    
    for i in range(len(offs)): #rotate about initial configuration
        offs[i] = scale(offs[i],centroid,0.5)
        offs[i] = rotate_about(offs[i],centroid,jet.pitch,jet.yaw,jet.roll)
    
    off_dots = [
        canv.create_oval(0,0,0,0,fill='blue'),
        canv.create_oval(0,0,0,0,fill='yellow'),
        canv.create_oval(0,0,0,0,fill='orange'),
        canv.create_oval(0,0,0,0,fill='pink'),
        canv.create_oval(0,0,0,0,fill='purple'),
    ]
    plane_lines = [
        canv.create_line(0,0,0,0,fill='white'),
        canv.create_line(0,0,0,0,fill='white'),
        canv.create_line(0,0,0,0,fill='white'),
        canv.create_line(0,0,0,0,fill='white'),
        canv.create_line(0,0,0,0,fill='white'),
        canv.create_line(0,0,0,0,fill='white')
    ]
    

    draw_loop()
    root.mainloop()
    
    
    
    
    
    
    