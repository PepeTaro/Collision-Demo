from vector2 import Vector2
from math import cos,sin
import utils

class Matrix3:
    def __init__(self,data=[0]*9):
        self.data = data # 3x3ゼロ行列を生成

    def __str__(self):
        s = "["
        s += "[{0},{1},{2}]\n".format(self.data[0],self.data[1],self.data[2])
        s += "[{0},{1},{2}]\n".format(self.data[3],self.data[4],self.data[5])
        s += "[{0},{1},{2}]".format(self.data[6],self.data[7],self.data[8])
        s += "]"
        return s
        
    def __add__(self,rhs):
        mat = Matrix3.Zero()
        for i in range(9):
            mat[i] = self.data[i] + rhs.data[i]
            
        return mat

    def __sub__(self,rhs):
        mat = Matrix3.Zero()
        for i in range(9):
            mat[i] = self.data[i] - rhs.data[i]
            
        return mat
    
    def __truediv__(self,rhs):
        assert(utils.is_number(rhs) and rhs != 0.0)

        mat = Matrix3.Zero()
        for i in range(9):
            mat[i] = self.data[i]/rhs
            
        return mat

    def __mul__(self,rhs):
        if(utils.is_number(rhs)):
            return self._mul_num(rhs)
        elif(isinstance(rhs,Matrix3)):
            return self._mul_data(rhs)
        elif(isinstance(rhs,Vector2)):
            return self._mul_vec(rhs)        
        else:
            print("Error:Matrix Multiplication")
            exit(-1)
        
    def __rmul__(self,lhs):
        assert(utils.is_number(lhs))

        mat = Matrix3.Zero()
        for i in range(9):
            mat[i] = self.data[i]*lhs
            
        return mat    

    def __getitem__(self,i):
        assert(i >= 0 and i < 9)
        return self.data[i]

    def __setitem__(self,i,value):
        assert(i >= 0 and i < 9)
        self.data[i] = value

    def __eq__(self,mat):
        for i in range(9):
            if(self.data[i] != mat[i]): return False

        return True
    
    def _mul_num(self,num):

        mat = Matrix3.Zero()
        for i in range(9):
            mat[i] = self.data[i]*num
        return mat

    def _mul_data(self,rhs):

        mat = Matrix3.Zero()
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    mat[j + 3*i] += self.data[k + 3*i]*rhs.data[j + 3*k]

        return mat

    def _mul_vec(self,vec):
        x = vec.x*self.data[0] + vec.y*self.data[1] + self.data[2] 
        y = vec.x*self.data[3] + vec.y*self.data[4] + self.data[5]        
        return Vector2(x,y)

    def inverse(self):
        det = self.data[0]*self.data[4]*self.data[8]
        det += self.data[3]*self.data[7]*self.data[2]
        det += self.data[6]*self.data[1]*self.data[5]
        det -= self.data[0]*self.data[7]*self.data[5]
        det -= self.data[6]*self.data[4]*self.data[2]
        det -= self.data[3]*self.data[1]*self.data[8]

        if(det == 0): return None # 逆行列が存在しない場合
        
        mat = Matrix3.Zero()
        mat[0] = self.data[4]*self.data[8] - self.data[5]*self.data[7]
        mat[1] = self.data[2]*self.data[7] - self.data[1]*self.data[8]
        mat[2] = self.data[1]*self.data[5] - self.data[2]*self.data[4]
        
        mat[3] = self.data[5]*self.data[6] - self.data[3]*self.data[8]
        mat[4] = self.data[0]*self.data[8] - self.data[2]*self.data[6]
        mat[5] = self.data[2]*self.data[3] - self.data[0]*self.data[5]
        
        mat[6] = self.data[3]*self.data[7] - self.data[4]*self.data[6]
        mat[7] = self.data[1]*self.data[6] - self.data[0]*self.data[7]
        mat[8] = self.data[0]*self.data[4] - self.data[1]*self.data[3]

        mat /= det
        return mat

    def transpose(self):
        mat = Matrix3.Zero()
        mat[0] = self.data[0]
        mat[1] = self.data[3]
        mat[2] = self.data[6]
        mat[3] = self.data[1]
        mat[4] = self.data[4]
        mat[5] = self.data[7]
        mat[6] = self.data[2]
        mat[7] = self.data[5]
        mat[8] = self.data[8]

        return mat
    
    @staticmethod
    def Zero():
        m=[0]*9
        return Matrix3(m)

    @staticmethod
    def Identity():
        mat = Matrix3.Zero()
        mat[0] = 1.0
        mat[4] = 1.0
        mat[8] = 1.0
        return mat

    @staticmethod
    def Translate(x,y):
        mat = Matrix3.Identity()
        mat[2] = x
        mat[5] = y
        return mat

    @staticmethod
    def Rotate(angle):
        mat = Matrix3.Identity()
        c = cos(angle)
        s = sin(angle)
        mat[0] = c
        mat[1] = -s
        mat[3] = s
        mat[4] = c
        return mat


def unittest():
    zero = Matrix3.Zero()
    assert(zero == zero.transpose())

    identity = Matrix3.Identity()
    assert(identity == identity.inverse())
    assert(identity == identity.transpose())
    
    print("Translate:\n",Matrix3.Translate(1,2)*Vector2(1,1))
    print("Rotate:\n",Matrix3.Rotate(3.141592/2.0)*Vector2(1,0))
    
    m1 = Matrix3([1,2,3,4,5,6,7,8,9])
    m2 = -1*Matrix3([1,2,3,4,5,6,7,8,9])
    v1 = Vector2(1,0)
    print("m1:\n",m1)
    print("transpose m1:\n",m1.transpose())
    print("m2:\n",m2)
    print("transpoe m2:\n",m2.transpose())
    print("v1:\n",v1)
    print("m1+m2:\n",m1+m2)
    print("m1-m2:\n",m1-m2)
    print("m1*m2:\n",m1*m2)
    print("m1*v1:\n",m1*v1)
        
def main():
    unittest()        

if __name__ == '__main__':
    main()
