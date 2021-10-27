import math
import utils

class Vector2:
    def __init__(self,x=0.0,y=0.0):
        self.x = x
        self.y = y

    def __str__(self):
        return "[{0},{1}]".format(self.x,self.y)
    
    def __add__(self,rhs):
        x = self.x + rhs.x
        y = self.y + rhs.y
        
        return Vector2(x,y)

    def __sub__(self,rhs):
        x = self.x - rhs.x
        y = self.y - rhs.y
        
        return Vector2(x,y)

    def __truediv__(self,rhs):
        assert(utils.is_number(rhs) and rhs != 0.0)
        x = self.x/rhs
        y = self.y/rhs
        
        return Vector2(x,y)

    def __mul__(self,rhs):
        assert(utils.is_number(rhs))
        x = self.x*rhs
        y = self.y*rhs

        return Vector2(x,y)

    def __rmul__(self,lhs):
        assert(utils.is_number(lhs))
        x = lhs*self.x
        y = lhs*self.y
        
        return Vector2(x,y)
    
    def __mod__(self,rhs):
        """
        [注意] modオペレーターをオーバーロードしているが,実際に行っていることはアダマール積
        つまり,a%b => (a.x*b.x , a.y*b.y , a.z*b.z)
        """
        x = self.x*rhs.x
        y = self.y*rhs.y
        
        return Vector2(x,y)

    def __neg__(self):
        return Vector2(-self.x,-self.y)
    
    def __eq__(self,vec):
        if(vec == None): return False
        elif(self.x == vec.x or self.y == vec.y): return True
        return False
    
    def invert(self):
        self.x = -self.x
        self.y = -self.y
    
    def norm(self):
        return math.sqrt(Vector2.dot(self,self))

    def normalize(self):
        norm_val = self.norm()
        if(norm_val > 0.0):
            self.x /= norm_val
            self.y /= norm_val

    def clear(self):
        self.x = self.y = 0.0

    def is_zero(self):
        return (self.x == 0 and self.y == 0)

    def is_almost_zero(self,epsilon):
        return (abs(self.x) < epsilon and abs(self.y) < epsilon)

    @staticmethod
    def dot(a,b):
        return (a.x*b.x + a.y*b.y)

    @staticmethod
    def cross(a,b):
        return (a.x*b.y - a.y*b.x)

    @staticmethod
    def dist(a,b):
        c = a - b
        return math.sqrt(Vector2.dot(c,c))

