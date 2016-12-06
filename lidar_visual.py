import sys
import sdl2
import sdl2.ext
import numpy as np

def map_value(value, in_min, in_max, out_min, out_max):
    percent = float(value - in_min) / (in_max/out_max)
    return out_min + (percent * (out_max - out_min))

#returns an OpenGL-style perspective matrix- column major
def perspective(left, right, top, bottom, near, far):
    width = right - left
    height = top - bottom
    depth = far-near
    return np.matrix([[2*near/width,       0,                   0,                   0],
                     [0,                  2*near/height,       0,                   0],
                     [(right+left)/width, (top+bottom)/height, -(far+near)/depth,  -1],
                     [0,                  0,                   -(2*far*near)/depth, 0]])

#returns an OpenGL-style viewport matrix to map 4D points to 3D screenspace- column major
def view(x, y, w, h):
    return np.matrix([[w/2,     0,      0,   0],
                     [0,       h/2,     0,   0],
                     [0,       0,       1/2, 0],
                     [x+(w/2), y+(h/2), 1/2, 1]])

class LidarVisualizer:
    
    def __init__(self, win_w, win_h):
        
        sdl2.ext.init()
        self.window = sdl2.ext.Window("LIDAR Visualizer", size=(win_w, win_h))
        self.window.show()
        
