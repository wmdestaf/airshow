import tkinter as tk
import numpy as np
from math import degrees, radians, cos, acos, sin, asin, pi, exp, sqrt, inf
from copy import deepcopy
from time import time
import torch
from random import randint, random

width=1000
height=700

MAX_THROTTLE = 100
MIN_THROTTLE = 0

GAMMA=0.999 #we are invested in the future

ortho = (1.0 /sqrt(6)) * np.matrix([[sqrt(3),0,-sqrt(3)],
                                   [1,2,1],
                                   [sqrt(2),-sqrt(2),sqrt(2)]])

def CLAMP(x,lower,upper):
    return min(upper,max(x,lower))

def to_ss(vec3):
    ss =  (ortho * SCALE_CS * np.reshape(vec3,(-1,1)))[:2]
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

MAX_STEPS=6000 #60 seconds with delay

def draw_loop():
    global all_jets, all_models, cur_step, halt_sema
    
    if not halt_sema:
        print("TRAINING - PLEASE WAIT")
        canv.after(10,draw_loop)
        return
        
    
    #movememt
    for jet, memory, model in zip(all_jets, all_memory, all_models):
        #TODO: fire model and move jet with output
        
        #decide what the hell we're doing
        with torch.no_grad():
            raw = state_to_future_array_from_jet(jet)
            data = torch.FloatTensor(raw)
            memory.append(data)
            
            #make decision
            out = model(data)
            
            choice = -1
            if random() > 0.05:  #likely, make best one (max)
                max_ = -inf
                maxi = -1
                
                for idx, i in enumerate(out):
                    if max_ < i:
                        max_ = i
                        maxi = idx
                
                choice = maxi
            else:
                choice = randint(0,31)

            #HAHAHAHAHAHA
            decisions = [activate(not not 0x1 & (choice >> (i))) for i in range(5)]
            jet.auto_input(decisions)
            #print(decisions)
        
     
    #time step
    empty = True
    for i, jet in enumerate(all_jets):
        jet.position_step_integration()
        jet.check_bounds()
        jet.graphical_step_iteration()
        im_empty = True
        for missile in jet.missiles:
            if missile and missile.detonated != 1:
                empty = False
            if missile and missile.flying != 1 and missile.detonated != 1:
                im_empty = False
        
       # print(im_empty)
        
        global held        
        if im_empty and held[i] == MAX_STEPS: 
           # print("gortcha")
            held[i] = cur_step
        
  #  print(held)
        
    if empty or cur_step == MAX_STEPS:
        reset_simulator(None,None)
        
        
    cur_step += 1
    canv.after(10,draw_loop)

def activate(_Bool):
    return -1 if not _Bool else 1

def sign(x):
    return -1 if x < 0 else 1

class Missile:
    def __init__(self, jet, other_jets,speed=100):
        self.flying    = 0
        self.detonated = 0
        self.jet = jet
        self.other_jets = other_jets
        self.radius = 0.15
        self.dot = canv.create_oval(0,0,0,0,outline='yellow')
        self.speed = speed
        self.centroid=deepcopy(self.jet.centroid)
    
    def launch(self):
        self.flying = 1
        self.direc = normalize(np.array(self.jet.offs[3]) - self.jet.centroid)
        self.centroid = deepcopy(self.jet.centroid)
    
    def time_step(self):
        if not self.flying and not self.detonated: #attached to hardpoint
            self.centroid = deepcopy(self.jet.centroid)
        elif self.flying: #trying to find target
            self.centroid += self.direc * self.speed * TIME_STEP
            
            #pretty colors....woww
            new = 'blue' if self.jet.canv.itemcget(self.dot,'outline') == 'red' else 'red'
            self.jet.canv.itemconfig(self.dot,outline=new)
            
            #should we go boom?
            #first, have we hit boundary?
            if self.centroid[0][0] < Jet.BXYZ[0]:
                self.explode()
            elif self.centroid[0][0] > Jet.BXYZ[1]:
                self.explode()
            elif self.centroid[0][1] < Jet.BXYZ[2]:
                self.explode()
            elif self.centroid[0][1] > Jet.BXYZ[3]:
                self.explode()
            elif self.centroid[0][2] < Jet.BXYZ[4]:
                self.explode()
            elif self.centroid[0][2] > Jet.BXYZ[5]:
                self.explode()

            #otherwise, have we hit a jet?
            for jet in self.other_jets:
                diff = np.linalg.norm(self.centroid - jet.centroid)
                if diff < self.radius:
                    self.explode()
                #print(diff)
            

        ss = to_ss(self.centroid)
        self.jet.canv.coords(self.dot, ss[0]-5, ss[1]-5,
                                       ss[0]+5, ss[1]+5)


    def reset(self):
        self.flying    = 0
        self.detonated = 0
        self.jet.canv.itemconfig(self.dot,outline='yellow')

    def explode(self):
        self.flying = 0
        self.detonated = 1
        self.jet.canv.itemconfig(self.dot,outline='white')
        
        for jet in self.other_jets:
            if np.linalg.norm(self.centroid - jet.centroid) < self.radius:
                reset_simulator(self.jet,jet)

def reset_simulator(winner, failer):
    global all_jets, cur_step
    global halt_sema #gil makes this trivially simple
    
    halt_sema = 0
    
    cur_step = 0
    
    if winner or failer:
        res_net = [0.0001 for _ in range(len(all_jets))]
    else:
        res_net = [-0.5 for _ in range(len(all_jets))]
    
    for idx, jet in enumerate(all_jets):
        jet.reset()
        if jet == winner:
            print("Jet %d won!" % idx)
            res_net[idx] = 2
        if jet == failer and winner == None:
            print("Jet %d crashed (unforced error)" % idx)
            res_net[idx] = -1
        elif jet == failer:
            print("Jet %d shot down! (by %d)!" % (idx, 
            all_jets.index(winner))) #expand to N jets, refactor this
            res_net[idx] = -2

    for i in range(len(res_net)): #stayed alive, gain some points
        global held
        len_held = held[i]
        if res_net[i] < 0:
            res_net[i] /= len_held
        else:
            res_net[i] *= len_held
    print(res_net)

    #return

    global all_models
    for jet, memory, model, rw, optimizer in zip(all_jets, all_memory, all_models, res_net, all_opts):
        
        inputs = memory
        outputs = [rw * pow(GAMMA,i) for i in range(len(inputs))]
        outputs.reverse()
        
        for epoch in range(3):
            for input_, output_ in zip(inputs, outputs):
                #training passes
                optimizer.zero_grad()
                
                #at such a point, what does the model say?
                model_res = max(model(input_))
                #at such a point, what is the discounted reward we got?
                disc__res = torch.FloatTensor([output_])
                
                #print(model_res, disc__res)
                
                loss = criterion(model_res, disc__res)
                loss.backward()
                optimizer.step()
        
    for memory in all_memory:
        memory.clear()
        
    held = [MAX_STEPS for _ in range(len(all_jets))]
        
    halt_sema = 1

TIME_STEP = 0.001
SCALE_CS  = 0.5
N_MISSILES = 4

#input layer
#for every jet: 4 orientation, (p,y,r,t), 3 centroid (x,y,z), N_MISSILES missiles 
#for every missile: 1 flying, 1 detonated 3 centroid (x,y,z)
#
#total inputs: N_JETS * ( (4 + 3) + ((1 + 1 + 3) * N_MISSILES))
#              N_JETS * ( 7 + 5 * N_MISSILES)
#              7*N_JETS + 5 * N_JETS * N_MISSILES
#
#output layer
#
#5 outputs (dPitch, dYaw, dRoll, dThrottle, fire) @ 2 options each = 10 outputs
#2^5 = 32 outputs. LSB=fire / not fire, 2LSB=incease / decrease thrust, etc...
#
#
#in our case, N_JETS=2, N_MISSILES=4. So, go from 54 inputs to 32 outputs. How should we do so?
#
#Let's make 9 layers.
#54->54->54->43->43->43->32->32->32
#
import torch.nn as nn
import torch.nn.functional as F

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc1 = nn.Linear(54,54)
        self.fc2 = nn.Linear(54,54)
        self.fc3 = nn.Linear(54,43)
        self.fc4 = nn.Linear(43,43)
        self.fc5 = nn.Linear(43,43)
        self.fc6 = nn.Linear(43,32)
        self.fc7 = nn.Linear(32,32)
        self.fc8 = nn.Linear(32,32)
        self.fc9 = nn.Linear(32,32)

    def forward(self, x):
        x = F.gelu(self.fc1(x))
        x = F.gelu(self.fc2(x))
        x = F.gelu(self.fc3(x))
        x = F.gelu(self.fc4(x))
        x = F.relu(self.fc5(x)) #just for funsies
        x = F.gelu(self.fc6(x))
        x = F.gelu(self.fc7(x))
        x = F.gelu(self.fc8(x))
        x = self.fc9(x)
        return x


def state_to_future_array_from_jet(jet):
    global all_jets
    #first, spit out our orientation, and throttle
    
    ajc = jet.centroid
    if len(ajc) == 1:
        ajc = ajc[0]

    feature = [
        ajc[0],
        ajc[1],
        ajc[2],
        jet.pitch, jet.yaw, jet.roll, jet.throttle
    ]

    #now, our missiles
    missile_info = []
    for missile in jet.missiles:
        if missile:
            ajmc = missile.centroid
            if len(ajmc) == 1: #WTF
                ajmc = ajmc[0]
                ajmc = ajmc.tolist()
            missile_info += ajmc
            missile_info += [missile.flying,missile.detonated]
    feature += missile_info
    
    #now, everyone else and 'their' missiles
    for jet2 in all_jets:
        if jet2 == jet:
            continue

        aj2c = jet2.centroid
        if len(aj2c) == 1:
            aj2c = aj2c[0]
        
        feature += [
            aj2c[0],
            aj2c[1],
            aj2c[2],
            jet2.pitch, jet2.yaw, jet2.roll,jet2.throttle
        ]

        for missile in jet2.missiles:
            missile_info = deepcopy([]) #losing my mind
            if missile:
                ajmc = missile.centroid
                if len(ajmc) == 1: #WTF
                    ajmc = ajmc[0]
                    ajmc = ajmc.tolist()

                missile_info += ajmc
                missile_info += [missile.flying,missile.detonated]
            feature += missile_info
    
    #print(feature)
    return feature

class Jet:
    
    BXYZ=[-5.5,4.5,0,7,-5.5,4.5]
    
    def auto_input(self,in_):
        dpitch, dyaw, droll, dthrottle, fire = in_
        self.dpitch    = sign(dpitch)
        self.dyaw      = sign(dyaw)
        self.droll     = sign(droll)
        self.dthrottle = sign(dthrottle)
        if(sign(fire) > 0):
            self.fire_missile()
    
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
            elif e.char == keys[8]:
                self.fire_missile()
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
        #return
        
        if self.centroid[0][0] < Jet.BXYZ[0]:
            print("Hit -X")
            reset_simulator(None,self)
        elif self.centroid[0][0] > Jet.BXYZ[1]:
            print("Hit +X")
            reset_simulator(None,self)
        elif self.centroid[0][1] < Jet.BXYZ[2]:
            print("Hit -Y")
            reset_simulator(None,self)
        elif self.centroid[0][1] > Jet.BXYZ[3]:
            print("Hit +Y")
            reset_simulator(None,self)
        elif self.centroid[0][2] < Jet.BXYZ[4]:
            print("Hit -Z")
            reset_simulator(None,self)
        elif self.centroid[0][2] > Jet.BXYZ[5]:
            print("Hit +Z")
            reset_simulator(None,self)

    def fire_missile(self):
        if self.missiles[self.cur_missile_ptr]:
            self.missiles[self.cur_missile_ptr].launch()
            self.cur_missile_ptr+=1

    def reset(self):
        self.centroid = np.array([deepcopy(self.centroid_b)])
        self.pitch=self.cond[0]
        self.roll=self.cond[1]
        self.yaw=self.cond[2]
        self.throttle=100
        self.velocity=deepcopy(self.start_velocity)
        self.force=np.array([0.,0.,0.])
        
        self.offs = deepcopy(offs_b)

        for i in range(len(self.offs)): #rotate about initial configuration
            self.offs[i] = scale(self.offs[i],list(self.centroid[0]),self.scale_factor)
            self.offs[i] = rotate_about(self.offs[i],self.centroid,self.pitch,self.yaw,self.roll)
    
        for missile in self.missiles:
            if missile:
                missile.reset()
        self.cur_missile_ptr = 0

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
        Fw = 17960 * 9.818 #* 0.001 #w h y
        #lerp thrust from our 0-100 slider
        Ft = 1400000 * (self.throttle / (MAX_THROTTLE - MIN_THROTTLE))
        
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
        self.centroid += TIME_STEP * self.velocity #leapfrog 2
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
            self.offs[i] += TIME_STEP * self.velocity
            self.offs[i] = rotate_about(self.offs[i],self.centroid,self.dpitch,self.dyaw,self.droll)
        
        #move missiles
        for missile in self.missiles:
            if missile:
                missile.time_step()

    def graphical_step_iteration(self):
        ss2 = to_ss(self.centroid)
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

        #move the missile display text
        canv.coords(self.missile_text,ss2[0],ss2[1] - 25)
        canv.itemconfig(self.missile_text,text=str(N_MISSILES - self.cur_missile_ptr))
        if self.cur_missile_ptr == N_MISSILES:
            canv.itemconfig(self.missile_text,fill='red')
        else:
            canv.itemconfig(self.missile_text,fill='cyan')

    def __init__(self, root, canv, keys, centroid_b, orient_b,
                 offs_b,start_velocity,kd_dispatcher,kr_dispatcher,
                 scale_factor=0.5,display_dots=True):
        
        self.root=root
        self.canv=canv
        
        kd_dispatcher.append(self.keydown_gen(keys))
        kr_dispatcher.append(self.keyup_gen(keys))
        
        self.cond = deepcopy(orient_b)
        
        self.pitch=self.cond[0]
        self.roll=self.cond[1]
        self.yaw=self.cond[2]
        self.throttle=100

        self.dpitch=0
        self.dyaw=0
        self.droll=0
        self.dthrottle=0
        
        self.start_velocity=np.array(start_velocity)
        self.velocity=deepcopy(start_velocity)
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
        
        self.missile_text = canv.create_text(0,0,fill='pink',text=str(N_MISSILES))
        
    def bind_opponents(self, others):
        #missiles!
        self.missiles = [Missile(self,others) for _ in range(N_MISSILES)]
        self.missiles.append(None) #terminator
        self.cur_missile_ptr = 0
    
def dispatch(e, dispatcher):
    for f in dispatcher: #there HAS to be a better way to do this...
        f(e)
    
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
    
    keys1 = ['a','d','s','w','q','e','1','3','2']
    keys2 = ['j','l','k','i','u','o','7','9','8']
    
    centroid_b = [0.5,2.5,0.4]
    orient_b = [0,90,0]
    centroid_c = [0.5,2.5,0.6]
    orient_c = [0,-90,0]
    offs_b = [
        [0.5,2.5,0.7], #main points
        [0.6,2.5,0.5],
        [0.4,2.5,0.5],
        [0.5,2.5,0.2],
        [0.5,2.6,0.7]
    ]
    
    kr_dispatcher = []
    kd_dispatcher = []
    root.bind("<KeyPress>",   lambda e: dispatch(e,kd_dispatcher))
    root.bind("<KeyRelease>", lambda e: dispatch(e,kr_dispatcher))
    
    jet1 = Jet(root,canv,keys1, centroid_b, orient_b, offs_b, 
    [0.01,0.0,0.0],
    kd_dispatcher,kr_dispatcher,scale_factor=0.4, display_dots=False)
    
    
    jet2 = Jet(root,canv,keys2,centroid_c, orient_c, offs_b, 
    [0.01,0.0,0.0],
    kd_dispatcher,kr_dispatcher,scale_factor=0.4, display_dots=False)
    
    jet1.bind_opponents([jet2])
    jet2.bind_opponents([jet1])
    
    all_jets = [jet1,jet2]
    
    model1 = Net()
    m1_mem = []
    model2 = Net()
    m2_mem = []
    all_models = [model1, model2]
    all_memory = [m1_mem, m2_mem]
    
    import torch.optim as optim
    criterion = nn.MSELoss()
    optimizer1 = optim.SGD(model1.parameters(), lr=0.001, momentum=0.9)
    optimizer2 = optim.SGD(model2.parameters(), lr=0.001, momentum=0.9)
    all_opts=[optimizer1,optimizer2]
    
    
    cur_step = 0
    
    canv.grid()

    halt_sema = 1
    held = [MAX_STEPS for _ in range(len(all_jets))]


    draw_loop()
    root.mainloop()
    
    
    
    
    
    
    