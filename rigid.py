from aabb import *
from const import PI,PI2

def _calculate_min(a,b,c):
    """
    3つのスカラーの最小値を求める
    """
    
    min_val = a
    
    if(b < min_val):
        min_val = b
    if(c < min_val):
        min_val = c

    return min_val

def _calculate_max(a,b,c):
    """
    3つのスカラーの最大値を求める
    """

    max_val = a
    if(b > max_val):
        max_val = b
    if(c > max_val):
        max_val = c

    return max_val

def _calculate_min_max(vec1,vec2,vec3):
    min_x_val = _calculate_min(vec1.x,vec2.x,vec3.x)
    max_x_val = _calculate_max(vec1.x,vec2.x,vec3.x)
    min_y_val = _calculate_min(vec1.y,vec2.y,vec3.y)
    max_y_val = _calculate_max(vec1.y,vec2.y,vec3.y)

    return (min_x_val,max_x_val,min_y_val,max_y_val)

class ObjectType:
    TRIANGLE = 0
    MESH = 1

class Object:
    def __init__(self,center=Vector2(0,0),vel=Vector2(0,0),ang=0.0,ang_vel=0.0,mass=1.0,moment_of_inertia=1.0):
        
        self.center = center # 質量中心
        self.vel = vel # 速度
        
        self.ang = ang # オブジェクトの向き(x軸から半時計回り)
        self.ang_vel = ang_vel # 角速度

        self.mass = mass # 質量
        self.inv_mass = 1.0/self.mass # 質量の逆数

        self.moment_of_inertia = moment_of_inertia # 慣性モーメント
        self.inv_moment_of_inertia = 1.0/moment_of_inertia # 慣性モーメントの逆数

        self.force = Vector2(0,0) # 合力
        self.torque = 0.0 # トルク

        self.translation = None
        self.rotation = None
        
    def integrate(self,delta_time):
        assert(delta_time > 0.0)
        
        accum_acc = self.inv_mass*self.force #合力による加速度追加
        delta_pos = (self.vel + 0.5*delta_time*accum_acc)*delta_time 
        self.center +=  delta_pos #位置を更新        
        self.vel += accum_acc*delta_time #速度更新

        accum_ang_acc = self.inv_moment_of_inertia*self.torque
        delta_ang = (self.ang_vel + 0.5*delta_time*accum_ang_acc)*delta_time
        self.ang +=  delta_ang #角度を更新
        self.ang = self.ang % PI2 # 角度がオーバーフロー或いはアンダーフローしないように調整
        self.ang_vel += accum_ang_acc*delta_time #角速度更新
        
        self.translation = Matrix3.Translate(delta_pos.x,delta_pos.y)
        self.rotation = Matrix3.Rotate(delta_ang)
        
        self.clear_force() # 合力を0に初期化
        self.clear_torque() # トルクを0に初期化
        
    def clear_force(self):
        self.force.clear()

    def clear_torque(self):
        self.torque = 0

    def add_force(self,force):
        """
        [注意]引数forceはWorld座標でのベクトルであることに注意
        """
        self.force += force

    def add_torque(self,torque):
        self.torque += torque
        
    def add_force_at_point(self,pos,force):
        self.force  += force
        self.torque += Vector2.cross((pos-self.center),force)

    def add_vel_impulse(self,impulse):
        self.vel += impulse*self.inv_mass

    def add_ang_impulse(self,vector,impulse):
        self.ang_vel += Vector2.cross(vector,impulse)*self.inv_moment_of_inertia
        
    def has_mass(self):
        return (self.mass > 0)

class Triangle(Object):
    def __init__(self,vec1,vec2,vec3,mass=1.0,moment_of_inertia=1.0):
        self.center = (vec1 + vec2 + vec3)/3 # 質量中心を計算        
        self.vecs = [vec1,vec2,vec3]
        
        self.surfaces = self._calculate_surfaces(vec1,vec2,vec3) # 各々の2頂点を結ぶベクトルを計算   
        self.normals = self._calculate_surface_normals(self.surfaces) # 各々の面に対する単位法線ベクトルを計算        
        self.aabb = self._calculate_bounding_box(self.vecs) # Bouding Boxを計算

        super().__init__(center=self.center,vel=Vector2(0,0),ang=0.0,ang_vel=0.0,mass=mass,moment_of_inertia=moment_of_inertia)
        
    def update(self,delta_time):
        assert(delta_time > 0.0)        
        super().integrate(delta_time)
        
        to_origin = Matrix3.Translate(-self.center.x,-self.center.y)
        inv_to_origin = Matrix3.Translate(self.center.x,self.center.y)
        mat = self.translation*inv_to_origin*self.rotation*to_origin
        
        self.transform(mat)
                
    def transform(self,mat,update_center=False):
        if(update_center): self.center  = mat*self.center
        
        self.vecs[0] = mat*self.vecs[0]
        self.vecs[1] = mat*self.vecs[1]
        self.vecs[2] = mat*self.vecs[2]
        self._update_bounding_box(self.vecs)
        
        # TODO: 法線ベクトルに関しては、行列matの逆行列を転置した行列を作用することで変換
        self.surfaces = self._calculate_surfaces(self.vecs[0],self.vecs[1],self.vecs[2])
        self.normals = self._calculate_surface_normals(self.surfaces)

    def _calculate_surfaces(self,vec1,vec2,vec3):
        """
        各々の2頂点を結ぶベクトルを計算
        """
        return [(vec2-vec1),(vec3-vec2),(vec1-vec3)]
    
    def _calculate_surface_normals(self,surfaces):
        """
        各々の面に対する単位法線ベクトルを計算
        """
        n1 = Vector2(surfaces[0].y,-surfaces[0].x)
        n2 = Vector2(surfaces[1].y,-surfaces[1].x)
        n3 = Vector2(surfaces[2].y,-surfaces[2].x)
        n1.normalize()
        n2.normalize()
        n3.normalize()

        return [n1,n2,n3]
    
    def _calculate_bounding_box(self,vecs):
        """
        Bounding Boxを計算
        """        
        (min_x_val,max_x_val,min_y_val,max_y_val) = _calculate_min_max(vecs[0],vecs[1],vecs[2])

        return AABB(Vector2(min_x_val,min_y_val),Vector2(max_x_val,max_y_val))

    def _update_bounding_box(self,vecs):
        """
        Bounding Boxを更新
        """        
        (min_x_val,max_x_val,min_y_val,max_y_val) = _calculate_min_max(vecs[0],vecs[1],vecs[2])
        
        self.aabb.lower.x = min_x_val
        self.aabb.lower.y = min_y_val
        
        self.aabb.upper.x = max_x_val
        self.aabb.upper.y = max_y_val

