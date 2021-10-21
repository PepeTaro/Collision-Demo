from aabb import *
from vector2 import Vector2
from matrix3 import Matrix3

class Node:
    def __init__(self,body=None,box=None,left=None,right=None,parent=None):
        self.body   = body # オブジェクトを格納(三角形,メッシュ)
        self.box    = box  # AABBを格納
        self.left   = left # 左の子
        self.right  = right # 右の子
        self.parent = parent # 親

    def is_leaf(self):
        return (self.body != None)
    
class AABBTree:
    def __init__(self):
        self.root = None

    def insert(self,node,new):
        """
        木に新たなノードであるnewを挿入。
        この関数を呼び出すとは,self.insert(self.root,node)
        """
        
        if(node == None): # 木が空の場合、根に新たなノードを代入
            self.root = new 
            
        elif(node.is_leaf()): # nodeが葉である場合、２つの葉に分岐させる(ひとつの葉はnode,もう一方はnew)
            new_node = Node(None,union_aabb_aabb(node.box,new.box),node,new,node.parent) # 左にnode,右にnewをもつ親ノードを生成
            
            if(node.parent == None): # nodeが根の場合(ひとつのノードしか存在しない場合)
                self.root = new_node
                
            else:
                # new_nodeを"適切"な場所に置く
                if(node.parent.left == node):
                    node.parent.left = new_node
                else:
                    node.parent.right = new_node

            node.parent = new_node
            new.parent = new_node
        else: # nodeが枝の場合
            
            # nodeの左右にあるBounding Box(node.left.box,node.right.box)と新たに追加するnew.boxの合併の大きさを計算し、
            # それを現在のノード(node)にあるBounding Box(node.box)と大きさを比較。
            # これを行う理由は、 Bounding Boxはできるだけ小さくしたいため。
            
            new_vol1 = get_growth_of_volume(node.left.box,new.box)
            new_vol2 = get_growth_of_volume(node.right.box,new.box)            

            if(new_vol1 < new_vol2):
                self.insert(node.left,new)
            else:
                self.insert(node.right,new)
                
            node.box = union_aabb_aabb(node.left.box,node.right.box)

    def remove(self,node): # nodeは必ず葉となることに注意(AABB木の定義)
        """
        木からnodeを削除
        """
        if(node.parent == None): # nodeが根の場合(つまり、nodeが最後のひとつノードである場合)
            self.root = None # 木を空にする

        elif(node.parent.parent == None): # node.parentが根の場合、nodeの兄弟であるノードを根にする
            if(node.parent.left == node):
                self.root = node.parent.right
            else:
                self.root = node.parent.left

            self.root.parent = None # 根であるため親は無し
            
        else:
            # nodeの兄弟であるノードsiblingを求める
            if(node.parent.left == node):
                sibling = node.parent.right
            else:
                sibling = node.parent.left
                
            # nodeの親が、nodeの親の親からみて右(あるいは左)にあるかチェック
            if(node.parent.parent.right == node.parent):
                # nodeの親をnodeの兄弟であるsiblingに置き換え、nodeの親の親のBoudingBoxを再計算(nodeが削除されたため、BoudingBoxが小さくなる可能性がある)
                node.parent.parent.right = sibling
                node.parent.parent.box = union_aabb_aabb(node.parent.parent.left.box,sibling.box)
            else:
                node.parent.parent.left = sibling
                node.parent.parent.box = union_aabb_aabb(node.parent.parent.right.box,sibling.box)
                
            sibling.parent = node.parent.parent

    def reinsert(self,node):
        """
        ノードを再挿入
        """
        self.remove(node)
        self.insert(self.root,node)
