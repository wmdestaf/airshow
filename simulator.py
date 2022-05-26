import tkinter as tk
import numpy as np
from math import degrees, radians, cos, acos, sin, asin, pi, exp, sqrt, inf
from copy import deepcopy
from time import time

width=1000
height=700

MAX_THROTTLE = 100
MIN_THROTTLE = 0

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

def scale(a, b, s):
    diff = [(aa - bb) * s for aa, bb in zip(a,b)] 
    return [bb + ddiff for bb, ddiff in zip(b, diff)]

def draw_loop():
    jet.position_step_integration()
    jet.check_bounds()
    jet.graphical_step_iteration()
    canv.after(10,draw_loop)

def sign(x):
    return -1 if x < 0 else 1

TIME_STEP = 0.001
SCALE_CS  = 0.1

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

    def check_bounds(self):
        #hit a wall?
        return
        
        if self.centroid[0][0] < -0.5:
            print("Hit -X")
            self.reset()
        elif self.centroid[0][0] > 1.0:
            print("Hit +X")
            self.reset()
        elif self.centroid[0][1] < 0.0:
            print("Hit -Y")
            self.reset()
        elif self.centroid[0][1] > sqrt(2):
            print("Hit +Y")
            self.reset()
        elif self.centroid[0][2] < -0.5:
            print("Hit -Z")
            self.reset()
        elif self.centroid[0][2] > 1.0:
            print("Hit +Z")
            self.reset()

    def reset(self):
        self.centroid = np.array([deepcopy(self.centroid_b)])
        self.pitch=0
        self.roll=0
        self.yaw=0
        self.throttle=100
        self.velocity=np.array([0.,0.,0.])
        self.force=np.array([0.,0.,0.])
        
        self.offs = deepcopy(offs_b)

        for i in range(len(self.offs)): #rotate about initial configuration
            self.offs[i] = scale(self.offs[i],list(self.centroid[0]),self.scale_factor)
            self.offs[i] = rotate_about(self.offs[i],self.centroid,self.pitch,self.yaw,self.roll)

    def position_step_integration(self):
        #adjust orientation
        self.pitch = (self.pitch + self.dpitch) % 360
        self.yaw   = (self.yaw   + self.dyaw)   % 360
        self.roll  = (self.roll  + self.droll)  % 360
        self.throttle = CLAMP(self.throttle + self.dthrottle, 0, 100)
        
        #FLIGHT DYNAMICS BEGIN HERE
        
        '''
        Constants we KNOW:
            w = 39000lb ~ 17690 kg
            max_thrust = 32000 ft/lbs ~ 140,000 N with afterburner
            g = -9.81 m/s/s
            wing area = 49.2 m^2
            air density = 1.2754 kg/m^3
            
            Cd0=0.025
            dCd=0.3 1/radians
            Cl0=0.1
            dCl=3.75 1/radians
            
            Mach 1 = 373 m/s
            kinematic viscocity of air = 1.460×10−5 m^2/s
            mean chord length ~ chord length = 4.8768m
        '''
        
        #orientation direction, lift direction
        direc = normalize(np.array(self.offs[3]) - self.centroid)
        up    = normalize(np.array(self.offs[4]) - self.offs[0])
        
        #weight = mass * gravity
        Fw = 17960 * 9.818 * 0.001 #w h y
        #lerp thrust from our 0-100 slider
        Ft = 700000 * (self.throttle / (MAX_THROTTLE - MIN_THROTTLE))
        
        #lift + drag force(s) are more complicated...
        v_scalar = np.linalg.norm(self.velocity)
        Mach = v_scalar / 373
        Reynolds = (v_scalar * 4.8768) / 0.0000146
        
        #angle of attack = velocity vs direction
        up = np.asarray(up) #why is this so
        angle_attack = acos(normalize(self.velocity).dot(direc[0])) * sign(direc[0][1])
        
        #lift coefficient = Cl0 + dCl * aa
        Cl = 0.1 + angle_attack * 3.75 #linear approx
        if degrees(angle_attack) > 22.5: #stall
            Cl = 0 #!!!!!
        
        #lift force = lift coefficient * air density * v^2 * A * 0.5
        Fl = Cl * 0.5 * (v_scalar ** 2) * 49.2 * 0.5
        
        
        #derag coefficient = Cd0 + dCd * aa
        Cd = 0.025 + angle_attack * 0.3
        
        #drag force, same as lift force, only with drag coefficient
        Fd = Cd * 0.5 * (np.linalg.norm(self.velocity) ** 2) * 49.2 * 0.5
        
        #FLIGHT DYNAMICS END HERE
        
        #sum of the forces equals zero, yadda yadda yadda....
        Fx= (up[0][0] * Fl) + (direc[0][0] * Ft) - (direc[0][0] * Fd)
        Fy= (up[0][1] * Fl) + (direc[0][1] * Ft) - (direc[0][1] * Fd) - (Fw)
        Fz= (up[0][2] * Fl) + (direc[0][2] * Ft) - (direc[0][2] * Fd)
        
        self.force = np.array([Fx,Fy,Fz])
        self.velocity += TIME_STEP * self.force * (1. / 17690.) #leapfrog 1: f/m = a
        self.centroid += TIME_STEP * self.velocity * SCALE_CS #leapfrog 2
        self.centroid = np.asarray(np.asmatrix(self.centroid))
        #print(self.centroid)
        
        #print(up,direc)#Fl, Fw, self.force)#, self.force, self.velocity)
        #print(self.force, Fw, Fl, self.velocity, self.centroid)
        
        #exit(1)

        #move center in worldspace
        
        #self.centroid += direc * 0.00004 * CLAMP(self.throttle - 50,0,inf)
        
        #move surrounding jet 'points' in worldspace
        for i in range(len(self.offs)):
            #self.offs[i] += direc * 0.00004 * CLAMP(self.throttle - 50,0,inf)
            self.offs[i] += TIME_STEP * self.velocity * SCALE_CS
            self.offs[i] = rotate_about(self.offs[i],self.centroid,self.dpitch,self.dyaw,self.droll)

    def graphical_step_iteration(self):
        ss2 = to_ss(jet.centroid)
        #print(ss2)
        
        if self.display_dots:
            self.canv.coords(self.centroid_dot,ss2[0]-5,ss2[1]-5,ss2[0]+5,ss2[1]+5)
        else:
            self.canv.coords(self.centroid_dot,0,0,0,0)
       
        for idx, (off_dot, off) in enumerate(zip(self.off_dots, self.offs)):
            ss = to_ss(off)
            self.canv.coords(self.plane_lines[idx], ss2+ss)
            
            if self.display_dots:
                canv.coords(off_dot,ss[0]-5,ss[1]-5,ss[0]+5,ss[1]+5)
            else:
                self.canv.coords(off_dot,0,0,0,0)
    
        self.canv.coords(self.plane_lines[5],to_ss(self.offs[0])+to_ss(self.offs[4])) #tailfin to centroid

    def __init__(self, root, canv, keys, centroid_b, offs_b,scale_factor=0.5,display_dots=True):
        
        self.root=root
        self.canv=canv
        
        self.root.bind("<KeyPress>",   self.keydown_gen(keys))
        self.root.bind("<KeyRelease>", self.keyup_gen(keys))
        self.pitch=0
        self.roll=0
        self.yaw=0
        self.throttle=100

        self.dpitch=0
        self.dyaw=0
        self.droll=0
        self.dthrottle=0
        
        self.velocity=np.array([0.0,0.0,0.0])
        self.force=np.array([0.0,0.0,0.0])
        
        self.centroid_b = centroid_b
        self.centroid = deepcopy(self.centroid_b)
        self.offs_b = offs_b
        self.offs = deepcopy(self.offs_b)
        
        self.scale_factor=scale_factor

        for i in range(len(self.offs)): #rotate about initial configuration
            self.offs[i] = scale(self.offs[i],self.centroid,self.scale_factor)
            self.offs[i] = rotate_about(self.offs[i],self.centroid,self.pitch,self.yaw,self.roll)
            
        self.centroid_dot = canv.create_oval(0,0,0,0,fill='red') #what we 'really' care about
        
        self.display_dots=display_dots
        self.off_dots = [ #minimum for unique orientation in 3doF
            canv.create_oval(0,0,0,0,fill='blue'),
            canv.create_oval(0,0,0,0,fill='yellow'),
            canv.create_oval(0,0,0,0,fill='orange'),
            canv.create_oval(0,0,0,0,fill='pink'),
            canv.create_oval(0,0,0,0,fill='purple'),
        ]
        self.plane_lines = [ #connect the dots together to look pretty
            canv.create_line(0,0,0,0,fill='white'),
            canv.create_line(0,0,0,0,fill='white'),
            canv.create_line(0,0,0,0,fill='white'),
            canv.create_line(0,0,0,0,fill='white'),
            canv.create_line(0,0,0,0,fill='white'),
            canv.create_line(0,0,0,0,fill='white')
        ]
    
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry('%dx%d' % (width,height) )
    root.title("Airshow")
    root.resizable(False,False)
    
    canv = tk.Canvas(root, height=height, width=width, bg='black')
    
    #make floor before anything, so don't have to retag it lower
    grid_points = [[0,0,0],[1,0,0],[1,0,1],[0,0,1]]
    grid_points = [to_ss(point) for point in grid_points]
    floor = canv.create_polygon(
        grid_points[0]+grid_points[1]+grid_points[2]+grid_points[3],
    fill='green')
    
    keys = ['a','d','s','w','j','l','k','i']
    centroid_b = [0.5,1.0,0.5]
    offs_b = [
        [0.5,1.0,0.7], #main points
        [0.6,1.0,0.5],
        [0.4,1.0,0.5],
        [0.5,1.0,0.2],
        [0.5,1.1,0.7]
    ]
    jet = Jet(root,canv,keys,centroid_b, offs_b, scale_factor=0.4, display_dots=True)
    canv.grid()

    


    draw_loop()
    root.mainloop()
    
    
    
    
    
    
    