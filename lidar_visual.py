import sys
import sdl2
import sdl2.ext
import ctypes
import math
import time
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

MAX_DIST = 6000.0

class LidarVisualizer:
    
    def __init__(self, win_w, win_h, lidar):
        
        self.lidar = lidar
        
        self.projection = perspective(-MAX_DIST, MAX_DIST, win_h/2, -win_h/2, 5/MAX_DIST, 1.0)
        self.viewport = view(0, 0, win_w, win_h)
        
        self.height = win_h
        self.width = win_w
        self.raw_data = []
        self.points = []

        self.running = True
        self.mode = 0
        
        sdl2.ext.init()
        self.window = sdl2.ext.Window("LIDAR Visualizer", size=(win_w, win_h))
        self.window.show()

    def update_data(self, distances):
        self.raw_data = distances
        self.points = []
        for i in range(len(distances)):
            c = -math.cos(math.pi * i / 180)
            s = -math.sin(math.pi * i / 180)
            
            x = c * (distances[i][0] / MAX_DIST)
            z = s * (distances[i][0] / MAX_DIST)
            
            ground = np.array([x, 0, z, 1])
            ceiling = np.array([x, self.height, z, 1])
            self.points.append(ground)
            self.points.append(ceiling)
    
    def graph_dists(self, surface, dists, colors):
        white = sdl2.ext.Color(255, 255, 255)
        sdl2.ext.fill(surface, white)
        
        pix_view = sdl2.ext.pixels2d(surface)
        
        for i in range(len(dists)):
           pix_view[i][self.height - dists[i] - 1] = colors[i]

    def draw_raw(self):
        dists = [d[0] for d in self.raw_data]
        quals = [q[0] for q in self.raw_data]
        colors = []
        max_qual = max(quals)
        
        for i in range(len(self.raw_data)):
            r = float(max_qual - quals[i]) / max_qual
            g = 1.0
            b = float(quals[i])/max_qual
            
            gray = dists[i] / MAX_DIST
            r = r * gray
            g = gray
            b = b * gray
            
            color = sdl2.ext.Color(int(255 * r), int(255 * g), int(255 * b))
            colors.append(color)
            dists[i] = int(self.height * dists[i]/MAX_DIST)
        
        win_surf = self.window.get_surface()
        self.graph_dists(win_surf, dists, colors)
            
    
    def draw_3D(self):
        pass
        
    def run(self):
        
        event = sdl2.SDL_Event()
        while self.running:
            #time.sleep(0.00001)
            self.update_data(self.lidar.get_image())
            while sdl2.SDL_PollEvent(ctypes.byref(event)) != 0:
                if event.type == sdl2.SDL_QUIT:
                    self.running = False
                    break
                
            print("Drawing...")
            if self.mode == 0:
                self.draw_raw()
            
            print("Refreshing...")
            self.window.refresh()
        
        sdl2.SDL_Quit()
        return 0
