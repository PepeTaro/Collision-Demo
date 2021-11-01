from graphics import *
from vector2 import Vector2
from matrix3 import Matrix3
from color import Color
import clip

from force import *
from rigid import *

import random
import math

def sign(p1,p2,p3):
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)

def point_in_triangle(pt,t):
    d1 = sign(pt,t.vecs[0],t.vecs[1])
    d2 = sign(pt,t.vecs[1],t.vecs[2])
    d3 = sign(pt,t.vecs[2],t.vecs[0])
    
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0);
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0);
    
    return not(has_neg and has_pos);

class MyGraphics(Graphics):
    def __init__(self,width,height):
        super().__init__(width,height)
        self.triangles = []
        self.forces    = []
        self.elapsed_time = 0.01
        self.prev_pos = None
        self.delta = None
        self.debug = False
        
    def event_handler(self,event):
        if event.type == QUIT:
            self._quit_handler()
            
        elif event.type == KEYUP:
            self._keyboard_up_handler()
                
        elif event.type == KEYDOWN:
            self._keyboard_down_handler(event.key)
                
        elif event.type == MOUSEBUTTONDOWN:
            self._mouse_button_down_handler()
            
        elif event.type == MOUSEBUTTONUP:
            self._mouse_button_up_handler()

    def keyboard_down_handler(self,key):        
        if key == K_q:
            self.running = False

    def holding_key_handler(self):
        if(not self.is_holding_key): return
        delta_angle = 2.0
        # 左右アローキーで、マウスで動かすオブジェクトを変更
        if self.holding_which_key == K_LEFT:
            self.triangles[0].add_torque(delta_angle)
            pass            
        elif self.holding_which_key == K_RIGHT:
            self.triangles[0].add_torque(-delta_angle)
            pass            
        elif self.holding_which_key == K_UP:
            self.debug = True
            pass            
        elif self.holding_which_key == K_DOWN:
            self.debug = False
            pass
        
    def holding_mouse_handler(self):
        if(not self.is_holding_mouse_button): return
        # マウスの現在の位置を取得し、以前の状態がないなら上書き
        pos = self.mouse_get_pos()        
        if(self.prev_pos == None): self.prev_pos = pos
        # 以前のマウスの位置と現在のマウスの位置の"差"を求め、そのベクトル分オブジェクトを動かく
        self.delta = (pos - self.prev_pos)        
        self.prev_pos = pos # 以前の位置を更新
                
        self.triangles[0].add_vel_impulse(self.delta*100)
        
        if(self.debug and self.prev_pos != None):
            self.draw_circle(Color.RED,self.prev_pos,4)
            self.draw_line(Color.RED,self.prev_pos,self.prev_pos + self.delta*10,4)

    def mouse_button_up_handler(self):
        self.prev_pos = None # マウスボタンを離したら、以前の位置をリセット

    def collision_test(self,t1,t2):        
        (depth,normal,is_t2_incident) = clip.sat(t1,t2)
        
        cp = None
        data = []        
        if(normal != None):
            cp = clip.get_contact_points(t1,t2,normal,data) 
            if(cp != None and is_t2_incident):
                
                if(self.debug):
                    self.draw_line(Color.GREEN,data[0].begin,data[0].end,5)
                    self.draw_line(Color.BLUE,data[1].begin,data[1].end,5)
                
                weight = weight1 = weight2 = 0.0
                if(t1.has_finite_mass()): weight += t1.inv_mass
                if(t2.has_finite_mass()): weight += t2.inv_mass
                if(weight == 0.0): return
                
                if(t1.has_finite_mass()): weight1 = t1.inv_mass/weight
                if(t2.has_finite_mass()): weight2 = t2.inv_mass/weight
                
                #e = 0.01*max(depth,0.5)
                e = 0.01
                restituon = 0
                separation_vel = Vector2.dot((t1.vel - t2.vel),normal)

                #if(t1.mass != -1): print(Vector2.norm(t1.vel),t1.mass,abs((self.elapsed_time)*t2.mass*100))
                #if(t2.mass != -1): print(Vector2.norm(t2.vel),t2.mass,abs((self.elapsed_time)*t1.mass*100))
                                
                if(separation_vel > 0.0):
                    new_separation_vel = -separation_vel*restituon
                    delta_vel = new_separation_vel - separation_vel
                    
                    if(t1.has_finite_mass()): t1.vel += delta_vel*weight1*normal
                    if(t2.has_finite_mass()): t2.vel -= delta_vel*weight2*normal
                
                if(cp.begin != None):                                        
                    if(t1.has_finite_moment_of_inertia()): t1.add_ang_impulse((cp.begin - t1.center),-e*normal)
                    if(t2.has_finite_moment_of_inertia()):t2.add_ang_impulse((cp.begin - t2.center),e*normal)

                    if(self.debug):
                        self.draw_circle(Color.RED,cp.begin,5)
                        self.draw_line(Color.YELLOW,t1.center,cp.begin,3)
                        self.draw_line(Color.CYAN,t1.center,t1.center - normal*30,3)
                        self.draw_line(Color.CYAN,cp.begin,cp.begin - normal*30,3)
                        self.draw_circle(Color.MAGENTA,t1.center,5)
                    
                if(cp.end != None):                                        
                    if(t1.has_finite_moment_of_inertia()): t1.add_ang_impulse((cp.end - t1.center),-e*normal)
                    if(t2.has_finite_moment_of_inertia()): t2.add_ang_impulse((cp.end - t2.center),e*normal)

                    if(self.debug):
                        self.draw_circle(Color.RED,cp.end,5)
                        self.draw_line(Color.YELLOW,t1.center,cp.end,3)
                        self.draw_line(Color.CYAN,t1.center,t1.center - normal*30,3)
                        self.draw_line(Color.CYAN,cp.end,cp.end - normal*30,3)                    
                        self.draw_circle(Color.MAGENTA,t1.center,5)

                """
                if(t1.has_finite_mass()):
                    mat1 = Matrix3.Translate(-depth*weight1*normal.x,-depth*weight1*normal.y)
                    t1.transform(mat1,True)

                if(t2.has_finite_mass()):
                    mat2 = Matrix3.Translate(depth*weight2*normal.x,depth*weight2*normal.y)                                    
                    t2.transform(mat2,True)                                
                """

                resolve_restitution = 10
                if(t1.has_finite_mass()): t1.vel -= depth*weight1*normal*resolve_restitution
                if(t2.has_finite_mass()): t2.vel += depth*weight2*normal*resolve_restitution

                
    def update(self):                
        delta = self.elapsed_time
        for force in self.forces:
            for obj in force.objs:
                force.update_force(obj,delta)
                
        for obj in self.triangles:
            obj.update(delta)

        for t1 in self.triangles:
            for t2 in self.triangles:                
                if(t1 == t2): continue
                self.collision_test(t1,t2)
        
    def draw_debug(self,t):
        self.draw_circle(Color.MAGENTA,t.center,2)
        self.draw_rect(Color.RED,t.aabb.lower,t.aabb.upper)

        for i in range(3):
            half = (t.vecs[i%3] + t.vecs[(i+1)%3])/2
            self.draw_line(Color.BLACK,half,half + t.normals[i]*10)

    def draw(self):                            
        for t in self.triangles:
            self.draw_triangle(Color.BLACK,t)
            if(self.debug): self.draw_debug(t)
            
    def draw_triangle(self,color,triangle,width=1):
        super().draw_triangle(color,triangle.vecs[0],triangle.vecs[1],triangle.vecs[2],width)

    def append_triangle(self,t):
        self.triangles.append(t)
        return len(self.triangles) - 1

    def append_force(self,force):
        self.forces.append(force)
        return len(self.forces) - 1

    def timer(self):
        self.clock.tick()
        fps = self.clock.get_fps()
        if(fps != 0): self.elapsed_time = 1.0/self.clock.get_fps()
        else: self.elapsed_time = 0.01

def _generate_random_triangles(xmin,xmax,ymin,ymax,size,num,mass,moment_of_inertia):
    triangles = []
    for i in range(num):
        x = random.randint(xmin,xmax)
        y = random.randint(ymin,ymax)        
        t = Triangle(Vector2(x,y),Vector2(x+size,y),Vector2(x+size*0.5,y+size),mass,moment_of_inertia)        
        triangles.append(t)

    return triangles

def main():

    size = 30    
    triangles = _generate_random_triangles(50,400,150,400,size,20,10,1)
    sweep = Triangle(Vector2(100,200),Vector2(150,100),Vector2(200,200),100,10)
    #floor = Triangle(Vector2(-100,100),Vector2(300,10),Vector2(700,100),-1,-1)
    #left_wall = Triangle(Vector2(50,100),Vector2(50,400),Vector2(0,200),-1,-1)
    #right_wall = Triangle(Vector2(500,100),Vector2(550,200),Vector2(500,400),-1,-1)
    floor = Triangle(Vector2(-200,100),Vector2(300,-200),Vector2(800,100),-1,-1)
    left_wall = Triangle(Vector2(-200,100),Vector2(50,100),Vector2(50,600),-1,-1)
    right_wall = Triangle(Vector2(600,100),Vector2(750,100),Vector2(600,600),-1,-1)
    
    gravity = RigidGravity()
    gravity.set_gravity(Vector2(0,-500))

    drag = RigidDrag()
    drag.set_drag(0,0.1)

    rot_drag = RigidRotationDrag()
    rot_drag.set_drag(1,1)
            
    graphics = MyGraphics(640,480)
    graphics.set_window_title("DYNAMIC AABB DEMO")

    graphics.append_triangle(sweep)    
    graphics.append_triangle(floor)
    graphics.append_triangle(right_wall)
    graphics.append_triangle(left_wall)

    gravity.append_obj(sweep)
    rot_drag.append_obj(sweep)
    
    for t in triangles:
        gravity.append_obj(t)
        drag.append_obj(t)
        rot_drag.append_obj(t)
        graphics.append_triangle(t)
    
    gravity_id = graphics.append_force(gravity)
    drag_id =graphics.append_force(drag)
    rot_drag_id =graphics.append_force(rot_drag)    
    
    graphics.init()    
    graphics.run()
    
if __name__=='__main__':
    main()
