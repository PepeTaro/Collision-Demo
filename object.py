from aabb import *

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
    def __init__(self,object_type,mass):
        self.object_type = object_type
        self.mass = mass
        self.is_static = False
        self.use_gravity = True

    """
    def update(self,delta_time):
        raise NotImplementedError('Error(Object.update):Override me!')
    """
    
class Mesh(Object):
    def __init__(self,mass,triangles=None):
        super().__init__(ObjectType.MESH,mass)
        
        self.triangles = [] # メッシュに属する三角形を格納
        self.center = None  # 質量中心ベクトル
        self.aabb = None    # Bounding Box 
        self.num_triangles = 0 # メッシュに属する三角形の数
            
    def push(self,triangle):
        """
        三角形をメッシュに追加、それに応じてメンバ変数を再計算
        """
        
        self.triangles.append(triangle)
        self.center = self._calculate_center(triangle)
        self.aabb = self._calculate_bounding_box(triangle)
        
        self.num_triangles += 1

    def update(self,mat):
        """
        行列matによりメッシュを変換、それに応じてBounding Boxも再計算
        """
        assert(isinstance(mat,Matrix3))
        for i in range(self.num_triangles):
            self.triangles[i].update(mat)

        self.center = mat*self.center
        self._update_bounding_box()
    
    def _calculate_center(self,triangle):
        """
        質量中心を漸化式で計算
        """
        
        # TODO: 以下の漸化式でcenterを求める場合、trinagleの個数が増加するにしたがって、数値誤差が膨らむ(つまり、centerが本来の中心からずれる)
        if(self.num_triangles == 0): return triangle.center
    
        inv_num_triangles_plus_1 = 1.0/(self.num_triangles + 1)
        # c_n = c_(n-1) * (n-1)/n + triangle.center/n
        # 以下のコードは上記の恒等式を計算している、ここでc_nは、meshにn個のtriangleが存在するときのcenterを表す(c_(0) := 0)
        return self.center*(self.num_triangles*inv_num_triangles_plus_1) + triangle.center*inv_num_triangles_plus_1

    def _calculate_bounding_box(self,triangle):
        """
        引数triangleは新たに追加する三角形。
        現在のBounding Boxであるself.aabbとその三角形のBounding Boxを"和"を計算
        """
        if(self.num_triangles == 0): return triangle.aabb
        return union_aabb_aabb(self.aabb,triangle.aabb)

    def _update_bounding_box(self):
        """
        わざわざ、メッシュのBounding Boxをいちから再計算する理由は以下の通り:
        ([注意]  このメソッドはupdateメソッドから呼ばれることに注意、行列matはupdateメソッドの引数)
        
        1) 行列matを単にBounding Boxに作用させるとAABBでなくなる(回転する可能性あり)
        2) 行列matから回転部分を除くことも考えられるが、それはそれでめんどくさい
        3) いい解決方法がないため、素朴に再計算
        """
        
        box = None
        for t in self.triangles:
            box = union_aabb_aabb(box,t.aabb)
            
        self.aabb.lower.x = box.lower.x
        self.aabb.lower.y = box.lower.y
        self.aabb.upper.x = box.upper.x
        self.aabb.upper.y = box.upper.y
        
class Triangle(Object):
    def __init__(self,vec1,vec2,vec3,mass):
        super().__init__(ObjectType.TRIANGLE,mass)

        self.center = (vec1 + vec2 + vec3)/3 # 質量中心を計算
        self.vecs = [vec1,vec2,vec3]
        
        self.surfaces = self._calculate_surfaces(vec1,vec2,vec3) # 各々の2頂点を結ぶベクトルを計算   
        self.normals = self._calculate_surface_normals(self.surfaces) # 各々の面に対する単位法線ベクトルを計算        
        self.aabb = self._calculate_bounding_box(self.vecs) # Bouding Boxを計算
        
    def update(self,mat):
        assert(isinstance(mat,Matrix3))
        
        self.vecs[0] = mat*self.vecs[0]
        self.vecs[1] = mat*self.vecs[1]
        self.vecs[2] = mat*self.vecs[2]
        self.center  = mat*self.center
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
