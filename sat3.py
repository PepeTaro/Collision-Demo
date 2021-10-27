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

def _sign(p1,p2,p3):
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)

def _is_point_in_triangle(p,t):
    d1 = _sign(p,t.vecs[0],t.vecs[1])
    d2 = _sign(p,t.vecs[1],t.vecs[2])
    d3 = _sign(p,t.vecs[2],t.vecs[0])

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

    return (not (has_neg and has_pos))

def _calculate_depth(p,q,axis):
    p_depth = Vector2.dot(p,axis)
    q_depth = Vector2.dot(q,axis)

    return (q_depth - p_depth)

def _calculate_max_depth_and_normal(p,t):
    depth0 = _calculate_depth(p,t.vecs[0],t.normals[0])
    depth1 = _calculate_depth(p,t.vecs[1],t.normals[1])
    depth2 = _calculate_depth(p,t.vecs[2],t.normals[2])

    if(depth0 < depth1):
        if(depth0 < depth2):
            return (depth0,t.normals[0])
        else:
            return (depth2,t.normals[2])        
    else:
        if(depth2 < depth1):
            return (depth2,t.normals[2])
        else:
            return (depth1,t.normals[1])

def _calculate_collision_data(t1,t2,collision_data):
    assert(isinstance(t1,Triangle))
    assert(isinstance(t2,Triangle))

    for t1_p in t1.vecs:
        if(not _is_point_in_triangle(t1_p,t2)): continue
        (max_depth,max_normal) = _calculate_max_depth_and_normal(t1_p,t2)

        collision_data.contact_point = t1_p
        collision_data.contact_normal = max_normal
        collision_data.depth = max_depth
        collision_data.moving_vector = max_depth*max_normal
        collision_data.does_collide = True

        return
    
def sat_objs(obj1,obj2):
    candidate_collision_data = CollisionData()
    
    if(isinstance(obj1,Triangle) and isinstance(obj2,Triangle)):
        _calculate_collision_data(obj1,obj2,candidate_collision_data)
    
    elif(isinstance(obj1,Mesh) and isinstance(obj2,Mesh)):
        for t1 in obj1.triangles:
            for t2 in obj2.triangles:                
                if(not sat_triangles(t1,t2)): continue
                _calculate_collision_data(t1,t2,candidate_collision_data)
                
    elif(isinstance(obj1,Mesh)): # obj1がMeshの場合
        for t in obj1.triangles:
            if(not sat_triangles(t,obj2)): continue
            _calculate_collision_data(t,obj2,candidate_collision_data)
            
    elif(isinstance(obj2,Mesh)): # obj2がMeshの場合
        for t in obj2.triangles:
            if(not sat_triangles(obj1,t)): continue
            _calculate_collision_data(obj1,t,candidate_collision_data)
            
    else:
        print("Error(sat_objs):そのようなオブジェクトは存在しません")
        exit(-1)

                    
    return candidate_collision_data

def sat_triangles(t1,t2):        
    # t1,t2の各々の法線ベクトルを使用してSATテスト
    if(not sat(t1,t2,t1.normals[0])): return 
    if(not sat(t1,t2,t1.normals[1])): return 
    if(not sat(t1,t2,t1.normals[2])): return 
    if(not sat(t1,t2,t2.normals[0])): return 
    if(not sat(t1,t2,t2.normals[1])): return 
    if(not sat(t1,t2,t2.normals[2])): return 

    return True

def sat(t1,t2,axis,is_t1_axis,collision_data):    
    (t1_min,t1_min_idx,t1_max,t1_max_idx) = _get_min_max_of_triangle_along_axis(t1,axis)
    (t2_min,t2_min_idx,t2_max,t2_max_idx) = _get_min_max_of_triangle_along_axis(t2,axis)
    
    if(t2_min > t1_max or t1_min > t2_max):
        # 衝突していない場合
        return False
    else:
        # 衝突している場合        
        return True
        
def collision_response(body,collision_data):        
    if(collision_data == None or (not collision_data.does_collide)): return # 衝突していない場合
        
    response_mat = Matrix3.Translate(collision_data.moving_vector.x,collision_data.moving_vector.y)
    body.update(response_mat)
