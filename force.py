from vector2 import Vector2
from rigid import Triangle

class ForceRegistry:
    def __init__(self):
        self.objs  = []

    def append_obj(self,obj):
        self.objs.append(obj)
        return len(self.objs) - 1
        
class RigidGravity(ForceRegistry):
    def __init__(self):
        super().__init__()
        self.gravity = Vector2(0,0)
        
    def set_gravity(self,gravity):
        self.gravity = gravity

    def update_force(self,rigid,delta_time):
        if(not rigid.has_mass()): return
        rigid.add_force(self.gravity*rigid.mass)
        
class RigidDrag(ForceRegistry):
    def __init__(self):
        super().__init__()
        self.k1 = None
        self.k2 = None

    def set_drag(self,k1,k2):                        
        self.k1 = k1
        self.k2 = k2
        
    def update_force(self,rigid,delta_time):        
        force = rigid.vel
        drag_coeff = force.norm()
        if(drag_coeff == 0): return #速度がゼロの場合
            
        force /= drag_coeff
        drag_coeff = (self.k1 + self.k2*drag_coeff)*drag_coeff

        force *= - drag_coeff
        rigid.add_force(force)

class RigidRotationDrag(ForceRegistry):
    def __init__(self):
        super().__init__()
        self.k1 = None
        self.k2 = None

    def set_drag(self,k1,k2):                        
        self.k1 = k1
        self.k2 = k2
        
    def update_force(self,rigid,delta_time):        
        ang_vel = rigid.ang_vel
        if(ang_vel == 0): return #角速度がゼロの場合            
        force = -(self.k1 + self.k2*abs(ang_vel))*ang_vel
        rigid.add_torque(force)
