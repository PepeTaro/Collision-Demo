import pygame
from color import Color
from vector2 import Vector2

"""
pygameの簡易ラッパークラスであるGraphicsを定義しているが、必要最低限の機能しか導入していない、
例えばキーボード入力はごく一部のみしかラッピングしていない。
"""

QUIT = pygame.QUIT
KEYUP = pygame.KEYUP
KEYDOWN = pygame.KEYDOWN
MOUSEBUTTONDOWN = pygame.MOUSEBUTTONDOWN
MOUSEBUTTONUP = pygame.MOUSEBUTTONUP

K_RIGHT = pygame.K_RIGHT
K_LEFT = pygame.K_LEFT
K_DOWN = pygame.K_DOWN
K_UP = pygame.K_UP
K_q  = pygame.K_q

class Graphics:
    
    def __init__(self,width,height):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode([self.width,self.height])
        
        self.running = False        
        self.is_holding_key = False
        self.holding_which_key = None
        self.is_holding_mouse_button = False
                
        self.clock = pygame.time.Clock()

    def set_window_title(self,title):
        pygame.display.set_caption(title)
        
    def init(self):
        pygame.init()
        pygame.font.init()
        
    def _event_handler(self):        
        self._holding_key_handler()
        self._holding_mouse_handler()

        for event in pygame.event.get():
            self.event_handler(event)
        
    def event_handler(self,event):
        pass
    
    def _quit_handler(self):
        self.running = False
        self.quit_handler()

    def quit_handler(self):
        pass
    
    def _keyboard_up_handler(self):
        self.is_holding_key = False
        self.holding_which_key = None
        self.keyboard_up_handler()

    def keyboard_up_handler(self):
        pass
    
    def _keyboard_down_handler(self,key):
        self.is_holding_key = True
        self.holding_which_key = key
        self.keyboard_down_handler(key)
        
    def keyboard_down_handler(self,key):        
        pass
    
    def _mouse_button_down_handler(self):
        self.is_holding_mouse_button = True
        self.mouse_button_down_handler()
        
    def mouse_button_down_handler(self):
        pass
    
    def _mouse_button_up_handler(self):
        self.is_holding_mouse_button = False
        self.mouse_button_up_handler()

    def mouse_button_down_handler(self):
        pass
    
    def _holding_mouse_handler(self):
        self.holding_mouse_handler()

    def holding_mouse_handler(self):
        pass
    
    def _holding_key_handler(self):
        self.holding_key_handler()

    def holding_key_handler(self):
        pass
        
    def run(self):
        self.running = True
        while self.running:
            self._event_handler()
            self._update()
            self._draw()
            self._timer()
        pygame.quit()
        
    def _calculate_cartesian_coordinate(self,vec):
        # 左下を原点に設定
        return (vec.x,self.height - vec.y)
    
    def draw_circle(self,color,vec,radius,width=0):
        (x,y) = self._calculate_cartesian_coordinate(vec)
        pygame.draw.circle(self.screen,color,(x,y),radius,width)

    def draw_line(self,color,vec1,vec2,width=1):
        (x1,y1) = self._calculate_cartesian_coordinate(vec1)
        (x2,y2) = self._calculate_cartesian_coordinate(vec2)
        pygame.draw.line(self.screen,color,(x1,y1),(x2,y2),width)
        
    def draw_triangle(self,color,vec1,vec2,vec3,width=1):
        (x1,y1) = self._calculate_cartesian_coordinate(vec1)
        (x2,y2) = self._calculate_cartesian_coordinate(vec2)
        (x3,y3) = self._calculate_cartesian_coordinate(vec3)        
        pygame.draw.line(self.screen,color,(x1,y1),(x2,y2),width)
        pygame.draw.line(self.screen,color,(x2,y2),(x3,y3),width)
        pygame.draw.line(self.screen,color,(x3,y3),(x1,y1),width)

    def draw_rect(self,color,vec1,vec2,width=1):
        (x1,y1) = self._calculate_cartesian_coordinate(vec1)
        (x2,y2) = self._calculate_cartesian_coordinate(vec2)        
        pygame.draw.rect(self.screen,color,(x1,y1,(x2-x1),(y2-y1)),width)
        
    def _update(self):
        self.update()
        
    def _draw(self):
        self.screen.fill(Color.WHITE)
        self.draw()
        pygame.display.flip()        
    
    def _timer(self):
        self.timer()

    def draw(self):
        raise NotImplementedError('Error(draw):Override me!')
    def update(self):
        raise NotImplementedError('Error(update):Override me!')    
    def timer(self):        
        pass
    
    def mouse_get_pos(self):
        """
        左下を原点とした座標成分に変換
        """
        pos = pygame.mouse.get_pos()
        pos_vec = Vector2(pos[0],self.height - pos[1])
        return pos_vec
