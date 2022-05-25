import tkinter as tk
import numpy as np
from math import radians, cos, sin, asin, pi, exp, sqrt, inf
from copy import deepcopy
PI2 = pi / 2

width=1000
height=700

pitch=0
roll=0
yaw=0
throttle=50

dpitch=0
dyaw=0
droll=0
dthrottle=0

MAX_THROTTLE = 100
MIN_THROTTLE = 0

old = -1

ortho = (1.0 /sqrt(6)) * np.matrix([[sqrt(3),0,-sqrt(3)],
                                   [1,2,1],
                                   [sqrt(2),-sqrt(2),sqrt(2)]])

def CLAMP(x,lower,upper):
    return min(upper,max(x,lower))

def mouse_orient(e):
   x = e.x
   y = e.y
   #print("Pointer is currently at %d, %d" % (x,y))

'''
    keybinds:
    
    a-d: roll  aft-starboard
    w-s: pitch up-down

    j-l: yaw   left-right
    i-k: throttle up-down
'''

def keydown(e):
    global dpitch, dyaw, droll, dthrottle
    
    if e.char == 'a':
        droll = -1
    elif e.char == 'd': 
        droll = 1
    elif e.char == 's':
        dpitch = -1
    elif e.char == 'w':
        dpitch = 1
    elif e.char == 'j':
        dyaw = -1
    elif e.char == 'l':
        dyaw = 1
    elif e.char == 'k':
        dthrottle = -1
    elif e.char == 'i':
        dthrottle = 1

def keyup(e):
    global dpitch, dyaw, droll, dthrottle
    
    if e.char == 'a' and droll == -1:
        droll = 0
    elif e.char == 'd' and droll == 1: 
        droll = 0
    elif e.char == 's' and dpitch == -1:
        dpitch = 0
    elif e.char == 'w' and dpitch == 1:
        dpitch = 0
    elif e.char == 'j' and dyaw == -1:
        dyaw = 0
    elif e.char == 'l' and dyaw == 1:
        dyaw = 0
    elif e.char == 'k' and dthrottle == -1:
        dthrottle = 0
    elif e.char == 'i' and dthrottle == 1:
        dthrottle = 0

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
    global centroid, offs, pitch, roll, yaw, throttle
    centroid = np.array([[0.5,1.0,0.5]])
    pitch=0
    roll=0
    yaw=0
    throttle=50
    offs = deepcopy(offs_b)
    
    
    for i in range(len(offs)): #rotate about initial configuration
        offs[i] = scale(offs[i],list(centroid[0]),0.5)
        offs[i] = rotate_about(offs[i],centroid,pitch,yaw,roll)
    
def scale(a, b, s):
    diff = [(aa - bb) * s for aa, bb in zip(a,b)] 
    return [bb + ddiff for bb, ddiff in zip(b, diff)]

def draw_loop():
    global pitch, yaw, roll, throttle
    
    pitch = (pitch + dpitch) % 360
    yaw   = (yaw   + dyaw)   % 360
    roll  = (roll  + droll)  % 360
    throttle = CLAMP(throttle + dthrottle, 0, 100)
    
    #print(pitch,yaw,roll,throttle)
    
    #let us draw the 'grid' guide:
    grid_points = [[0,0,0],[1,0,0],[0,1,0],[1,1,0],
                   [0,0,1],[1,0,1],[0,1,1],[1,1,1]]
    grid_points = [to_ss(point) for point in grid_points]

    canv.coords(floor,grid_points[0]+grid_points[1]+grid_points[5]+grid_points[4])

    
    global old
    cur = (old + 1) % 8
    canv.itemconfig(text,text=str(cur))
    canv.itemconfig(grid_dots[old],fill='white')
    canv.itemconfig(grid_dots[cur],fill='red')
    old = cur
    

    #bounds
    for dot, point in zip(grid_dots, grid_points):
        canv.coords(dot, point[0], point[1], point[0] + 5, point[1] + 5)
       
       
    #move in worldspace
    global centroid
    direc = normalize(np.array(offs[3]) - centroid)
    centroid += direc * 0.0001 * CLAMP(throttle - 50,0,inf)
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
        offs[i] += direc * 0.0001 * CLAMP(throttle - 50,0,inf)

    #maybe use this?
    depth_factor = sqrt((centroid[0][0] * centroid[0][0]) + 
                        (centroid[0][2] * centroid[0][2]) )
    print(centroid, throttle, depth_factor)
       
    #plane center, don't rotate
    ss2 = to_ss(centroid)
    canv.coords(centroid_dot,ss2[0]-5,ss2[1]-5,ss2[0]+5,ss2[1]+5)
       
    for idx, (off_dot, off) in enumerate(zip(off_dots, offs)):
        off2 = (np.array(off - centroid))
        
        ss = rotate_about(off,centroid,dpitch,dyaw,droll)
        offs[idx] = ss
        ss = to_ss(ss)
        
        #line to plane center
        canv.coords(plane_lines[idx], ss2+ss)
        
        canv.coords(off_dot,ss[0]-5,ss[1]-5,
                            ss[0]+5,ss[1]+5)
    
    canv.coords(plane_lines[5],to_ss(offs[0])+to_ss(offs[4]))
    canv.after(10,draw_loop)

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry('%dx%d' % (width,height) )
    root.title("Airshow")
    root.resizable(False,False)
    
    canv = tk.Canvas(root, height=height, width=width, bg='black')
    root.bind('<Motion>',mouse_orient)
    root.bind("<KeyPress>", keydown)
    root.bind("<KeyRelease>", keyup)
    canv.grid()

    grid_dots = [canv.create_oval(0,0,0,0,fill='white') for j in range(8)]

    floor = canv.create_polygon(0,0,0,0,0,0,0,0,fill='green')
    text = canv.create_text(200,200,text='h',fill='white')

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
        offs[i] = rotate_about(offs[i],centroid,pitch,yaw,roll)
    
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
    
    
    
    
    
    
    