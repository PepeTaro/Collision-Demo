from object import Triangle,Mesh
from vector2 import Vector2
from matrix3 import Matrix3

EPSILON     = 1e-8 # 数値誤差の許容範囲として使用
RESTITUTION = 1.0  # [注意 この値は正数であることに注意] めり込み後の、"復元力"(この数値が大きいほど反発する、小さい場合ゆっくりとめり込みが戻る)

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
    
def sat(t1,t2,axis,is_t1_axis,contact_context):
    """
    t1,t2は三角形
    axisはSATテストに使用する軸
    is_t1_axisは、引数axisが三角形t1の法線ベクトルの場合True、そうでない場合False
    contact_contextは、後のCollision Responseに必要な情報を格納する
    """
    
    (p_min,p_min_idx,p_max,p_max_idx) = _get_min_max_of_triangle_along_axis(t1,axis)
    (q_min,q_min_idx,q_max,q_max_idx) = _get_min_max_of_triangle_along_axis(t2,axis)
    
    if(q_min > p_max or p_min > q_max): # 衝突していない場合
        return False
    else:                               # 衝突している場合
        
        if(is_t1_axis): # axisがt1の法線ベクトルである場合
            depth_candidate = p_max - q_min
            #depth_candidate = q_max - p_min
            
        else:           # axisがt2の法線ベクトルである場合
            #depth_candidate = p_max - q_min
            depth_candidate = q_max - p_min
        
        if(contact_context[1] == None): # SATテスト(複数の軸でテストする)内において初衝突の場合、単に情報を保存
            contact_context[0] = axis
            contact_context[1] = depth_candidate
            contact_context[2] = is_t1_axis
            
        elif(depth_candidate < contact_context[1]): # 最小のdepth(めり込み長さ)である情報を優先する
            contact_context[0] = axis
            contact_context[1] = depth_candidate
            contact_context[2] = is_t1_axis
            
        return True

def _update_max_delta(max_delta,max_move_opposite_direction,delta,move_opposite_direction):
    """
    delta(めり込み)が、現在最大のめり込み(max_delta)より大きい場合、その値を返す(つまり、最大のめり込みを保存しておきたい)
    """
    if(delta != None and (not delta.is_almost_zero(EPSILON))): # めり込みが発生した場合 
        if(max_delta == None or delta.norm() > max_delta.norm()): # 最大のめり込みがある情報を保存
            #(max_delta,max_move_opposite_direction) = (delta,move_opposite_direction)
            return (delta,move_opposite_direction)
        
    return (max_delta,max_move_opposite_direction)

def _get_max_delta(t1,t2,max_delta,max_move_opposite_direction):
    """
    三角形t1,t2との衝突判定をして、めり込み具合(delta)を計算し、新たなめり込みが、現在のめり込みより"大きい"ならその値を返す
    """
    (delta,move_opposite_direction) = test_sat_triangles(t1,t2)
    (max_delta,max_move_opposite_direction) = _update_max_delta(max_delta,max_move_opposite_direction,delta,move_opposite_direction)
    
    return (max_delta,max_move_opposite_direction)
    
def test_sat_objs(obj1,obj2):
    max_delta = None
    max_move_opposite_direction = None
    
    # TODO: 三角形もメッシュに統合する?
    
    if(isinstance(obj1,Triangle) and isinstance(obj2,Triangle)):
        return test_sat_triangles(obj1,obj2)
    
    elif(isinstance(obj1,Mesh) and isinstance(obj2,Mesh)):
        for t1 in obj1.triangles:
            for t2 in obj2.triangles:                
                (max_delta,max_move_opposite_direction) = _get_max_delta(t1,t2,max_delta,max_move_opposite_direction)                
        return (max_delta,max_move_opposite_direction)
    
    elif(isinstance(obj1,Mesh)): # obj1がMeshの場合
        for t in obj1.triangles:            
            (max_delta,max_move_opposite_direction) = _get_max_delta(t,obj2,max_delta,max_move_opposite_direction)            
        return (max_delta,max_move_opposite_direction)
    
    elif(isinstance(obj2,Mesh)): # obj2がMeshの場合
        for t in obj2.triangles:                        
            (max_delta,max_move_opposite_direction) = _get_max_delta(obj1,t,max_delta,max_move_opposite_direction)            
        return (max_delta,max_move_opposite_direction)
    
    else:
        print("Error(test_sat_objs):そのようなオブジェクトは存在しません")
        exit(-1)
        
def test_sat_triangles(t1,t2):
    contact_context = [None,None,None] # (法線ベクトル,めり込み長さ,三角形t1の法線を使用したか否か)を保存

    # t1,t2の各々の法線ベクトルを使用してSATテスト
    if(not sat(t1,t2,t1.normals[0],True,contact_context)): return (None,None)    
    if(not sat(t1,t2,t1.normals[1],True,contact_context)): return (None,None)
    if(not sat(t1,t2,t1.normals[2],True,contact_context)): return (None,None)    
    if(not sat(t1,t2,t2.normals[0],False,contact_context)): return (None,None)
    if(not sat(t1,t2,t2.normals[1],False,contact_context)): return (None,None)
    if(not sat(t1,t2,t2.normals[2],False,contact_context)): return (None,None)

    # めり込みを解消するときに、質量に応じてより軽いオブジェクトがより動くように調整
    mass_weight = (t2.mass/(t1.mass + t2.mass))
    delta = contact_context[0]*contact_context[1]*mass_weight

    return (delta,contact_context[2])

def collision_response(body,delta,move_opposite_direction):        
    if(delta == None or move_opposite_direction == None): return # 衝突していない場合

    # めり込みを直すためにmove_opposite_directionに応じて、オブジェクトを動かす方向を決定
    if(move_opposite_direction):
        mat = Matrix3.Translate(-delta.x*RESTITUTION,-delta.y*RESTITUTION)        
        body.update(mat)
    else:
        mat = Matrix3.Translate(delta.x*RESTITUTION,delta.y*RESTITUTION)
        body.update(mat)
    
