from aabb import *
from aabb_tree import *
from graphics import *
from object import *
from vector2 import Vector2
from matrix3 import Matrix3
from color import Color

#import sat
import sat4
import clip 

import random
import math

class MyGraphics(Graphics):
    def __init__(self,width,height):
        super().__init__(width,height)
        
        self.tree = None # AABB木
        self.nodes = None # AABB木のノードを格納したリスト

        self.prev_pos = None # マウスでオブジェクトを動かすため使用
        self.moving_obj_idx = 0 # マウスで動かすオブジェクトを指定するために使用

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

        # 左右アローキーで、マウスで動かすオブジェクトを変更
        if self.holding_which_key == K_LEFT:            
            self.moving_obj_idx = (self.moving_obj_idx - 1)%len(self.nodes)

        elif self.holding_which_key == K_RIGHT:
            self.moving_obj_idx = (self.moving_obj_idx + 1)%len(self.nodes)
        
    def holding_mouse_handler(self):
        if(not self.is_holding_mouse_button): return

        # マウスの現在の位置を取得し、以前の状態がないなら上書き
        pos = self.mouse_get_pos()        
        if(self.prev_pos == None): self.prev_pos = pos

        # 以前のマウスの位置と現在のマウスの位置の"差"を求め、そのベクトル分オブジェクトを動かく
        delta = (pos - self.prev_pos)
        mat = Matrix3.Translate(delta.x,delta.y)        
        self.nodes[self.moving_obj_idx].body.update(mat)
        
        self.prev_pos = pos # 以前の位置を更新

    def mouse_button_up_handler(self):
        self.prev_pos = None # マウスボタンを離したら、以前の位置をリセット
                        
    def test_aabb(self,test_node,node,debug=False):
        """
        Broad Phase段階の衝突判定
        """
        
        if(node == test_node): return # 自身とは衝突判定をしない
        
        if(node.is_leaf()): # AABB木の葉に達した場合(AABB木の定義により、葉にのみオブジェクトが保存されている)

             # SAT(Separating Axis Theorem)テスト                                
             #(delta,move_opposite_direction) = sat.test_sat_objs(test_node.body,node.body) # Narrow Phase段階の衝突判定
            #sat.collision_response(test_node.body,delta,move_opposite_direction)    # Collision Response(めり込み解消)

            #collision_data = sat4.sat_objs(test_node.body,node.body) # Narrow Phase段階の衝突判定                            
            #sat4.collision_response(test_node.body,collision_data)    # Collision Response(めり込み解消)
            
            (depth,normal,is_test_normal) = clip.sat(test_node.body,node.body)
            
            cp = None
            data = []

            if(normal != None):                                
                mass_weight = node.body.mass/(test_node.body.mass + node.body.mass)
                    
                if(is_test_normal):
                    moving_vector = -normal*depth*mass_weight
                else:
                    moving_vector = normal*depth*mass_weight
                    
                mat = Matrix3.Translate(moving_vector.x,moving_vector.y)      
                test_node.body.update(mat)                
                
            if(normal != None and is_test_normal):
                cp = clip.get_contact_points(test_node.body,node.body,normal,data)
                
                """
                theta1 = theta2 = 0.0                
                if(cp.begin != None):
                    r1 =  cp.begin - test_node.body.center
                    if(is_test_normal):
                        theta1 = Vector2.cross(r1,normal)*1e-5
                    else:
                        theta1 = Vector2.cross(r1,-normal)*1e-5
                        
                if(cp.end != None):
                    r2 =  cp.end - test_node.body.center
                    
                    if(is_test_normal):
                        theta2 = Vector2.cross(r2,normal)*1e-5
                    else:
                        theta2 = Vector2.cross(r2,-normal)*1e-5

                theta = theta1 + theta2
                """
                
                if(normal != None and cp != None):
                    if(depth < 1e-1): return

                    if(self.debug):
                        if(cp.begin != None):
                            self.draw_circle(Color.RED,cp.begin,6)
                        if(cp.end != None):
                            self.draw_circle(Color.RED,cp.end,6)
                    
                    angle = 1e-3*depth

                    if(cp.begin == None and cp.end == None):
                        return
                    
                    elif(cp.begin != None and cp.end != None):
                        r =  ((cp.begin - test_node.body.center) + (cp.end - test_node.body.center))/2
                    elif(cp.begin != None):
                        r =  cp.begin - test_node.body.center                        
                    elif(cp.end != None):
                        r =  cp.end - test_node.body.center
                                        
                    if(is_test_normal):
                        theta = Vector2.cross(r,-normal)*angle*1.0/test_node.body.mass
                        print("test_normal:",theta,depth)
                        trans_mat = Matrix3.Translate(-test_node.body.center.x,-test_node.body.center.y)
                        rot_mat = Matrix3.Rotate(theta)
                        inv_trans_mat = Matrix3.Translate(test_node.body.center.x,test_node.body.center.y)                        
                        mat = inv_trans_mat*rot_mat*trans_mat                                        
                        test_node.body.update(mat)
                        
                    else:                        
                        theta = Vector2.cross(r,normal)*angle*1.0/test_node.body.mass
                        print("not test_normal:",theta)
                        trans_mat = Matrix3.Translate(-test_node.body.center.x,-test_node.body.center.y)
                        rot_mat = Matrix3.Rotate(theta)
                        inv_trans_mat = Matrix3.Translate(test_node.body.center.x,test_node.body.center.y)                        
                        mat = inv_trans_mat*rot_mat*trans_mat                                        
                        test_node.body.update(mat)                    
                
                """
                if(normal != None and cp != None):                    
                    if(cp.begin != None):
                        self.draw_circle(Color.RED,cp.begin,6)
                    if(cp.end != None):
                        self.draw_circle(Color.RED,cp.end,6)
                
                if(cp.begin != None and cp.end != None):
                    pass
                elif(cp.begin == None and cp.end == None):
                    pass
                else:
                    if(cp.begin != None):
                        r =  cp.begin - test_node.body.center                        
                    elif(cp.end != None):
                        r =  cp.end - test_node.body.center

                    angle = 1e-3
                                        
                    if(is_test_normal):
                        theta = Vector2.cross(r,-normal)*angle*1.0/node.body.mass
                        print("test_normal:",theta,depth)
                        trans_mat = Matrix3.Translate(-node.body.center.x,-node.body.center.y)
                        rot_mat = Matrix3.Rotate(theta)
                        inv_trans_mat = Matrix3.Translate(node.body.center.x,node.body.center.y)                        
                        mat = inv_trans_mat*rot_mat*trans_mat                                        
                        node.body.update(mat)
                        
                    else:                        
                        theta = Vector2.cross(r,normal)*angle*1.0/node.body.mass
                        print("not test_normal:",theta)
                        trans_mat = Matrix3.Translate(-node.body.center.x,-node.body.center.y)
                        rot_mat = Matrix3.Rotate(theta)
                        inv_trans_mat = Matrix3.Translate(node.body.center.x,node.body.center.y)                        
                        mat = inv_trans_mat*rot_mat*trans_mat                                        
                        node.body.update(mat)                    
                
                """
            if(debug): # 衝突の可能性があるオブジェクトを赤色で描写
                """
                self.draw_rect(Color.RED,test_node.box.lower,test_node.box.upper,2)
                self.draw_rect(Color.RED,node.box.lower,node.box.upper,2)
                """
                
                if(normal != None):
                    self.draw_circle(Color.MAGENTA,test_node.body.center,4)
                    self.draw_line(Color.MAGENTA,test_node.body.center,test_node.body.center + normal*depth,3)
                    
                    if(cp != None):
                        self.draw_line(Color.GREEN,data[0].begin,data[0].end,3) # ref
                        self.draw_circle(Color.GREEN,data[0].begin,2) # inc
                        self.draw_circle(Color.GREEN,data[0].end,4) # inc
                        
                        self.draw_line(Color.BLUE,data[1].begin,data[1].end,3) # inc
                        self.draw_circle(Color.BLUE,data[1].begin,2) # inc
                        self.draw_circle(Color.BLUE,data[1].end,4) # inc
                        
                        self.draw_line(Color.YELLOW,data[3],data[3] + data[2]*20,3)
                        
                        if(cp.begin != None):
                            self.draw_circle(Color.RED,cp.begin,6)
                        if(cp.end != None):
                            self.draw_circle(Color.RED,cp.end,6)                    
                    
            return

         # Bounding Boxと接触している場合、接触している子を再帰的に探す
        if(test_aabb_aabb(test_node.box,node.box)):
            self.test_aabb(test_node,node.left,debug) 
            self.test_aabb(test_node,node.right,debug)
            
        else: # 接触していない場合
            return
                
    def collision_test(self):
        """
        すべてのオブジェクトに対し衝突判定をして、めり込みを解消する。
        """        
        for i in range(len(self.nodes)):
            self.test_aabb(self.nodes[i],self.tree.root,self.debug) # Broad phase        

    def update_objs(self):
        """
        オブジェクトの位置を更新(重力、回転など)
        """

        """
        angles = [0.01,0.02,0.03,0.04]
        num_angles = len(angles)
        """
        
        for i in range(len(self.nodes)):
            if(self.nodes[i].body.is_static): continue # 静的オブジェクトは動かさない
            
            grav_mat = Matrix3.Identity()
            if(self.nodes[i].body.use_gravity):  # 重力を使用するオブジェクトのみ重力を作用
                grav_mat = Matrix3.Translate(0,-0.4)
                
            """
            k = i%num_angles            
            trans_mat = Matrix3.Translate(-self.nodes[i].body.center.x,-self.nodes[i].body.center.y)
            inv_trans_mat = Matrix3.Translate(self.nodes[i].body.center.x,self.nodes[i].body.center.y)        
            rot_mat = Matrix3.Rotate(angles[k])
            mat =  grav_mat*inv_trans_mat*rot_mat*trans_mat
            mat =  grav_mat
            self.nodes[i].body.update(mat)
            """

            self.nodes[i].body.update(grav_mat)

    def update(self):
        self.update_objs() # すべてのオブジェクトを更新(回転、並進など)
                
        for node in self.nodes: # ノードをAABB木に再挿入
            self.tree.reinsert(node)        
        pass
    
    def draw_node(self,node):
        if(node == None): return
        
        if(node.is_leaf()): # Bounding Boxを黒色で描写
            self.draw_rect(Color.BLACK,node.box.lower,node.box.upper)
            return
        else: # AABB木の枝に対応するBounding Boxを黒色で描写
            self.draw_rect(Color.BLACK,node.box.lower,node.box.upper)

        # 再帰で下に"下る"
        self.draw_node(node.left)            
        self.draw_node(node.right)

    def draw_objs(self,color):
        for node in self.nodes:            
            if(isinstance(node.body,Triangle)):
                self.draw_triangle(color,node.body,1)
            elif(isinstance(node.body,Mesh)):
                self.draw_mesh(color,node.body,1)
            else:
                print("ERROR(draw_objs):そのようなオブジェクトは存在しません")
                exit(-1)

    def debug_draw(self):
        if(self.debug):
            #self.draw_node(self.tree.root)
            #self.test_aabb(self.nodes[self.moving_obj_idx],self.tree.root,self.debug)
            pass
        
    def draw(self):
        self.debug_draw() # デバッグ用
        self.collision_test() # 衝突判定
        self.draw_objs(Color.BLACK) # すべてのオブジェクトを描写
    
    def timer(self):
        self.clock.tick()
        #self.set_window_title("{0} FPS".format(int(self.clock.get_fps())))
        
    def draw_triangle(self,color,triangle,width=1):
        super().draw_triangle(color,triangle.vecs[0],triangle.vecs[1],triangle.vecs[2],width)

    def draw_mesh(self,color,mesh,width=1):
        for triangle in mesh.triangles:
            self.draw_triangle(color,triangle,width)

    def turn_on_debug(self):
        self.debug = True

    def turn_off_debug(self):
        self.debug = False
    
    def set_tree(self,tree):
        self.tree = tree 

    def set_nodes(self,nodes):
        self.nodes = nodes 

def _append_circle_mesh_to_nodes(x,y,radius,num_triangles,mass,nodes):
    delta_ang = 2*math.pi/num_triangles
    origin = Vector2(x,y)
    vecs = []
    
    for i in range(num_triangles):        
        c = x + radius*math.cos(i*delta_ang)
        s = y + radius*math.sin(i*delta_ang)
        vecs.append(Vector2(c,s))
        
    mesh = Mesh(10)
    vecs_len = len(vecs)
    for i in range(0,len(vecs)-1,1):
        mesh.push(Triangle(origin,vecs[i],vecs[i+1],mass))
    mesh.push(Triangle(origin,vecs[vecs_len-1],vecs[0],mass))

    nodes.append(Node(mesh,mesh.aabb)) 
    
def _append_random_triangle_to_nodes(x_min,x_max,y_min,y_max,size,num,mass,nodes):    
    for i in range(num):
        x = random.randint(x_min,x_max)
        y = random.randint(y_min,y_max)        
        t = Triangle(Vector2(x,y),Vector2(x+size,y),Vector2(x+size*0.5,y+size),1)        
        nodes.append(Node(t,t.aabb))

def main():    
    tree = AABBTree()
    nodes = []

    """
    _append_circle_mesh_to_nodes(300,400,20,10,10,nodes)    
    _append_random_triangle_to_nodes(10,400,10,400,20,1,1,nodes)    
    t = Triangle(Vector2(20,20),Vector2(320,-10),Vector2(600,20),10000)
    t.use_gravity = False
    t.is_static  = True
    nodes.append(Node(t,t.aabb))
    """
    
    #_append_circle_mesh_to_nodes(300,400,20,10,10,nodes)    
    _append_random_triangle_to_nodes(10,400,100,400,30,30,1,nodes)
    
    t = Triangle(Vector2(20,100),Vector2(320,40),Vector2(600,100),100000)
    t.use_gravity = False
    t.is_static  = True
    nodes.append(Node(t,t.aabb))
    
    
    for node in nodes: # すべてのノードをAABB木に挿入
        tree.insert(tree.root,node)
    
    graphics = MyGraphics(640,480)
    graphics.set_window_title("DYNAMIC AABB DEMO")
    #graphics.turn_on_debug()
    
    graphics.set_tree(tree)
    graphics.set_nodes(nodes)
    
    graphics.init()    
    graphics.run()
    
if __name__=='__main__':
    main()
