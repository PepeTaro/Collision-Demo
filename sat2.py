from object import Triangle,Mesh
from vector2 import Vector2
from matrix3 import Matrix3

EPSILON     = 1e-8 # 数値誤差の許容範囲として使用

class CollisionData:
    def __init__(self,contact_point=None,contact_normal=None,depth=None,moving_vector=None,does_collide=False):
        self.contact_point = contact_point
        self.contact_normal = contact_normal
        self.depth = depth
        self.moving_vector = moving_vector
        self.does_collide = does_collide

def _get_min_max_of_triangle_along_axis(t,axis):
    """
    三角形tの各3頂点をaxisに射影し、その3つの値(スカラー)の(最小値,最小値のインデックス,最大値,最大値のインデックス)を求め返す
    ここで最小値のインデックスは、その最小値の値に対応する三角形の頂点のインデックスを意味する
    """    
    (p_max_idx,p_min_idx) = (2,2)
    p_max = Vector2.dot(t.vecs[2],axis)
    p_min = p_max
    
    for i in range(2):
        val = Vector2.dot(t.vecs[i],axis)
        if(p_max < val):
            p_max = val
            p_max_idx = i            
        if(p_min > val):
            p_min = val
            p_min_idx = i

    return (p_min,p_min_idx,p_max,p_max_idx)

def _get_max_depth(t1,t2,candidate_collision_data):        
    collision_data = sat_triangles(t1,t2)
    
    if(collision_data == None): # t1とt2が衝突していない場合、現在の衝突情報を保持
        return candidate_collision_data

    if(not candidate_collision_data.does_collide):
        return collision_data
    
    if(collision_data.depth > 0.0): # めり込みが発生した場合
    #if(collision_data.depth > EPSILON): # めり込みが発生した場合 
        if(collision_data.depth > candidate_collision_data.depth): # メッシュの内、最大のめり込みがある情報を保存
            return collision_data
        
    return candidate_collision_data

def sat_objs(obj1,obj2):
    candidate_collision_data = CollisionData()
    
    if(isinstance(obj1,Triangle) and isinstance(obj2,Triangle)):
        return sat_triangles(obj1,obj2)
    
    elif(isinstance(obj1,Mesh) and isinstance(obj2,Mesh)):
        for t1 in obj1.triangles:
            for t2 in obj2.triangles:                
                candidate_collision_data = _get_max_depth(t1,t2,candidate_collision_data)
                
        return candidate_collision_data
    
    elif(isinstance(obj1,Mesh)): # obj1がMeshの場合
        for t in obj1.triangles:            
            candidate_collision_data = _get_max_depth(t,obj2,candidate_collision_data)
            
        return candidate_collision_data
    
    elif(isinstance(obj2,Mesh)): # obj2がMeshの場合
        for t in obj2.triangles:                        
            candidate_collision_data = _get_max_depth(obj1,t,candidate_collision_data)
            
        return candidate_collision_data
    
    else:
        print("Error(sat_objs):そのようなオブジェクトは存在しません")
        exit(-1)

def sat_triangles(t1,t2):    
    collision_data = CollisionData()
    
    # t1,t2の各々の法線ベクトルを使用してSATテスト
    if(not sat(t1,t2,t1.normals[0],True,collision_data)): return None    
    if(not sat(t1,t2,t1.normals[1],True,collision_data)): return None
    if(not sat(t1,t2,t1.normals[2],True,collision_data)): return None
    if(not sat(t1,t2,t2.normals[0],False,collision_data)): return None
    if(not sat(t1,t2,t2.normals[1],False,collision_data)): return None
    if(not sat(t1,t2,t2.normals[2],False,collision_data)): return None

    # めり込みを解消するときに、質量に応じてより軽いオブジェクトがより動くように調整
    mass_weight = (t2.mass/(t1.mass + t2.mass))
    collision_data.moving_vector = collision_data.contact_normal*collision_data.depth*mass_weight
    
    return collision_data

def sat(t1,t2,axis,is_t1_axis,collision_data):
    
    (p_min,p_min_idx,p_max,p_max_idx) = _get_min_max_of_triangle_along_axis(t1,axis)
    (q_min,q_min_idx,q_max,q_max_idx) = _get_min_max_of_triangle_along_axis(t2,axis)
    
    if(q_min > p_max or p_min > q_max): # 衝突していない場合
        return False
    else:                               # 衝突している場合
        
        if(is_t1_axis): # axisがt1の法線ベクトルである場合
            depth_candidate = p_max - q_min
            #contact_point = t2.vecs[q_min_idx]
            
            contact_point = t2.vecs[q_min_idx]
            
        else:           # axisがt2の法線ベクトルである場合
            depth_candidate = q_max - p_min
            #contact_point = t1.vecs[p_min_idx]

            contact_point = t1.vecs[p_min_idx]
            
        #print(depth_candidate)
        
        if((not collision_data.does_collide) or depth_candidate < collision_data.depth): # "最大"のdepth(めり込み深さ)である情報を優先する
            collision_data.contact_point = contact_point
            
            if(is_t1_axis): collision_data.contact_normal = -axis
            else: collision_data.contact_normal = axis
                
            collision_data.depth = depth_candidate
            collision_data.does_collide = True
            
        return True

def collision_response(body,collision_data):        
    if(collision_data == None or (not collision_data.does_collide)): return # 衝突していない場合
    """
    r = collision_data.contact_point - body.center
    theta = Vector2.cross(r,collision_data.contact_normal)*1.0/body.mass*0.01
    
    trans_mat = Matrix3.Translate(-body.center.x,-body.center.y)
    rot_mat = Matrix3.Rotate(theta)
    inv_trans_mat = Matrix3.Translate(body.center.x,body.center.y)
    
    mat1 = inv_trans_mat*rot_mat*trans_mat

    response_mat = Matrix3.Translate(collision_data.moving_vector.x,collision_data.moving_vector.y)
    mat = response_mat*mat1    
    body.update(mat)
    """
        
    response_mat = Matrix3.Translate(collision_data.moving_vector.x,collision_data.moving_vector.y)
    body.update(response_mat)
