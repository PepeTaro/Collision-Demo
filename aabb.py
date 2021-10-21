from vector2 import Vector2
from matrix3 import Matrix3
    
class AABB:
    def __init__(self,lower,upper):
        #　x軸が左から右、y軸が下から上であることに注意
        self.lower = lower # 長方形の左下の点
        self.upper = upper # 長方形の右上の点

def min_vec(u,v):
    """
    2つのベクトルの各々成分の最小値を格納したベクトルを計算
    """    
    w = Vector2()
    w.x = min(u.x,v.x)
    w.y = min(u.y,v.y)
    return w

def max_vec(u,v):
    """
    2つのベクトルの各々成分の最大値を格納したベクトルを計算
    """
    w = Vector2()
    w.x = max(u.x,v.x)
    w.y = max(u.y,v.y)
    return w

def get_growth_of_volume(box1,box2):
    """
    2つのAABBであるbox1とbox2を合併をし、その面積を求める
    """
    
    assert(box1 != None or box2 != None)
    
    if(box1 == None):
        return get_aabb_volume(box2)
    elif(box2 == None):
        return get_aabb_volume(box1)
    else:
        new_box = union_aabb_aabb(box1,box2)
        return get_aabb_volume(new_box)

def get_aabb_volume(a):
    """
    AABBの面積を求める
    """
    width = abs(a.lower.x - a.upper.x)
    height = abs(a.lower.y - a.upper.y)
    return width*height

def union_aabb_aabb(a,b):
    """
    2つのAABBの合併を計算
    """
    assert(a != None or b != None)
    if(a == None): return b
    elif(b == None): return a
    
    lower = min_vec(a.lower,b.lower)
    upper = max_vec(a.upper,b.upper)
    return AABB(lower,upper)

def test_aabb_aabb(a,b):
    """
    2つのAABBの衝突判定
    """
    if(a.upper.x < b.lower.x or a.lower.x > b.upper.x): return False
    if(a.upper.y < b.lower.y or a.lower.y > b.upper.y): return False
    
    return True

def update_aabb(a,mat):
    """
    AABBを行列matによって変換
    """
    a.lower = mat*a.lower
    a.upper = mat*a.upper
