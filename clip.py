"""
https://dyn4j.org/2011/11/contact-points-using-clipping/
"""

from vector2 import Vector2
from object import *

EPSILON = 1e-8

class ClippedPoints:
    def __init__(self,begin=None,end=None):
        self.begin = begin
        self.end = end

    def length(self):
        count = 0
        if(self.begin != None): count += 1
        if(self.end != None): count += 1

        return count
    
class Edge:
    def __init__(self,begin=None,end=None):
        self.begin = begin
        self.end = end

    def dot(self,v):
        return Vector2.dot(self.end - self.begin,v)

    def get_edge(self):
        return (self.end - self.begin)

def get_min_max_points_along_axis(t,n):
    min_val  = max_val   = Vector2.dot(t.vecs[0],n)
    min_p = max_p = t.vecs[0]
    min_idx  = max_idx   = 0

    for i in range(1,3): # i = 1 to 2
        val = Vector2.dot(t.vecs[i],n)

        if(val > max_val + EPSILON):
            max_val = val
            max_p   = t.vecs[i]
            max_idx = i

        if(val < min_val - EPSILON):
            min_val = val
            min_p   = t.vecs[i]
            min_idx = i

    return (min_p,min_val,min_idx,max_p,max_val,max_idx)

def best_edge(t,n):
    (min_p,min_val,min_idx,max_p,max_val,max_idx) = get_min_max_points_along_axis(t,n)

    v = t.vecs[max_idx]
    v1 = t.vecs[(max_idx + 1)%3]
    v0 = t.vecs[(max_idx - 1)%3]

    l = v - v1
    l.normalize()
    r = v - v0
    r.normalize()
    
    if(Vector2.dot(r,n) <= Vector2.dot(l,n)):
        return Edge(v0,v)
    else:
        return Edge(v,v1)
    
def get_contact_points(t1,t2,n,data):
    
    e1 = best_edge(t1,n)
    e2 = best_edge(t2,-n)

    ref = Edge()
    inc = Edge()
    is_flip = False
    
    if(abs(e1.dot(n)) <= abs(e2.dot(n))):
        ref = e1
        inc = e2        
    else:
        #print("FLIP!!")
        #print(abs(e1.dot(n)),abs(e2.dot(n)))
        ref = e2
        inc = e1
        is_flip = True
    
    ref_edge = ref.get_edge()
    ref_edge.normalize()
        
    data.append(ref)
    data.append(inc)
    
    o1 = Vector2.dot(ref_edge,ref.begin)
    cp = clip(inc.begin,inc.end,ref_edge,o1)
    if(cp.length() < 2): return None

    o2 = Vector2.dot(ref_edge,ref.end)
    cp = clip(cp.begin,cp.end,-ref_edge,-o2)
    if(cp.length() < 2): return None
    
    #ref_normal = -n    #?
    ref_normal = -Vector2(ref_edge.y,-ref_edge.x)
    if(is_flip): ref_normal.invert()
    
    data.append(ref_normal)
    data.append(ref.end)
    
    max_val1 = Vector2.dot(ref_normal,ref.begin)
    max_val2 = Vector2.dot(ref_normal,ref.end)
    if(max_val1 > max_val2): max_val = max_val1
    else: max_val = max_val2        
    #max_val =  Vector2.dot(ref_normal,ref.end)
    
    if(Vector2.dot(ref_normal,cp.begin) < max_val):cp.begin = None
    if(Vector2.dot(ref_normal,cp.end) < max_val): cp.end = None

    return cp

def clip(v1,v2,n,o):
    """
    始点v1と終点v2を結ぶ線分を、法線ベクトルnに直交する平面oに対してクリップ
    """
    cp = ClippedPoints()
    
    # 後に平面oと線分(v2-v1)との交点を求める際に必要
    d1 = Vector2.dot(n,v1) - o 
    d2 = Vector2.dot(n,v2) - o 

    # v1,v2が平面oより"前方"にある場合(d1 >= 0.0),(d2 >= 0.0)となる
    if(d1 >= 0.0): cp.begin = v1
    if(d2 >= 0.0): cp.end = v2

    if(d1*d2 < 0.0): # 線分が平面oを跨いでいる場合
        line_segment = v2 - v1
        parameter = d1/(d1 - d2)
        intersection = v1 + parameter*line_segment
        
        if(d1 < 0.0): cp.begin = intersection
        else: cp.end = intersection
        
    return cp

def sat(t1,t2):
    (depth1,normal1) = get_min_depth_normal(t1,t2)
    if(depth1 == None): return (None,None,None)
    
    (depth2,normal2) = get_min_depth_normal(t2,t1)
    if(depth2 == None): return (None,None,None)
    
    if(depth1 <= depth2):
        return (depth1,normal1,True)
    else:
        return (depth2,normal2,False)

def get_min_depth_normal(t1,t2):
    depth = normal = None

    for n1 in t1.normals:
        (t1_min_p,t1_min_val,t1_min_idx,t1_max_p,t1_max_val,t1_max_idx) = get_min_max_points_along_axis(t1,n1)
        (t2_min_p,t2_min_val,t2_min_idx,t2_max_p,t2_max_val,t2_max_idx) = get_min_max_points_along_axis(t2,n1)
        
        if(t1_max_val < t2_min_val or t2_max_val < t1_min_val):
            return (None,None)
        
        if(t2_min_val <= t1_max_val):
            depth_candidate = (t1_max_val - t2_min_val)
            if((depth == None) or depth_candidate < depth):
                (depth,normal) = (depth_candidate,n1)
                
        elif(t1_min_val <= t2_max_val):
            depth_candidate = (t2_max_val - t1_min_val)
            if((depth == None) or depth_candidate < depth):
                (depth,normal) = (depth_candidate,n1)
        
    return (depth,normal)


if __name__ == '__main__':
    main()
