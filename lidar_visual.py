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

def round(value):
    frac = value - int(value)
    if frac >= 0.5:
        return int(value) + 1
    else:
        return int(value)

MAX_DIST = 6000.0

RAW = 0
POLAR = 1
EUCLID_3D = 2
PRETTY_POLAR = 3
PRETTY_3D = 4

class LidarVisualizer:
    
    def __init__(self, win_w, win_h, lidar):
        
        self.lidar = lidar
        
        self.projection = perspective(-MAX_DIST, MAX_DIST, win_h/2, -win_h/2, 1, MAX_DIST)
        self.viewport = view(0, 0, win_w, win_h)
        
        self.height = win_h
        self.width = win_w
        self.raw_data = []
        self.points = []
        self.depth_buf = [[MAX_DIST-1] * self.width] * self.height

        self.running = True
        self.mode = POLAR
        
        sdl2.ext.init()
        self.window = sdl2.ext.Window("LIDAR Visualizer", size=(win_w, win_h))
        self.window.show()

    def update_data(self, distances):
        self.raw_data = distances
    
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
            g = float(quals[i])/max_qual
            b = 1.0
            
            gray = dists[i] / MAX_DIST
            r = r * gray
            g = g * gray
            b = b * gray
            
            color = sdl2.ext.Color(int(255 * r), int(255 * g), int(255 * b))
            colors.append(color)
            dists[i] = int(self.height * dists[i]/MAX_DIST)
        
        win_surf = self.window.get_surface()
        self.graph_dists(win_surf, dists, colors)
        
    def draw_polar(self):
        dists = [d[0] for d in self.raw_data]
        quals = [q[0] for q in self.raw_data]

        max_qual = max(quals)

        coords = []
        colors = []
        
        for i in range(len(self.raw_data)):
            c = -math.cos(math.pi * i / 180)
            s = -math.sin(math.pi * i / 180)
            
            x = (c * dists[i]) / MAX_DIST
            z = (s * dists[i]) / MAX_DIST
            
            r = float(max_qual - quals[i]) / max_qual
            g = float(quals[i])/max_qual
            b = 1.0
            
            gray = 1.0 - (dists[i] / MAX_DIST)
            r = r * gray
            g = g * gray
            b = gray
            
            color = sdl2.ext.Color(int(255 * r), int(255 * g), int(255 * b))
            colors.append(color)
            
            #transform to screen space
            point = (np.array([x, z, 0, 1]) * self.viewport).A1
            point = [int(point[0] / point[3]), self.height - int(point[1] / point[3])]
            
            coords.append(point)
        
        win_surf = self.window.get_surface()
        
        white = sdl2.ext.Color(255, 255, 255)
        sdl2.ext.fill(win_surf, white)
        
        pix_view = sdl2.ext.pixels2d(win_surf)
        for i in range(len(coords)):
            color = colors[i]
            pixel_x = coords[i][0]
            pixel_y = coords[i][1]
            pix_view[coords[i][1]][coords[i][0]] = color
        
    
    def draw_3D(self):
        dists = [d[0] for d in self.raw_data]
        quals = [q[0] for q in self.raw_data]

        max_qual = max(quals)

        coords = []
        colors = []
        
        for i in range(len(self.raw_data)):
            c = -math.cos(math.pi * i / 180)
            s = -math.sin(math.pi * i / 180)
            
            x = c * dists[i]
            z = s * dists[i]
            
            r = float(max_qual - quals[i]) / max_qual
            g = float(quals[i])/max_qual
            b = 1.0
            
            gray = 1.0 - (dists[i] / MAX_DIST)
            r = r * gray
            g = g * gray
            b = gray
            
            color = sdl2.ext.Color(int(255 * r), int(255 * g), int(255 * b))
            colors.append(color)
            
            floor = (np.array([x, -self.height/2.0, z, 1]) * self.projection * self.viewport).A1
            ceil = (np.array([x, self.height/2.0, z, 1]) * self.projection * self.viewport).A1
            
            floor = np.array([int(floor[0]), int(floor[1]), int(floor[2])])
            ceil = np.array([int(ceil[0]), int(ceil[1]), int(ceil[2])])
            
            coords.append((floor, ceil))
        
        win_surf = self.window.get_surface()
        
        white = sdl2.ext.Color(255, 255, 255)
        sdl2.ext.fill(win_surf, white)
        
        pix_view = sdl2.ext.pixels2d(win_surf)
        for i in range(len(coords)):
            x = coords[i][0][0]
            y = coords[i][0][1]
            z = coords[i][0][2]
            
            while y < coords[i][1][1]:
                if self.depth_buf[y][x] > z:
                    self.depth_buf[y][x] = z
                    
                    color = colors[i]
                    pix_view[y][x] = color
                    y = y + 1
                
    def refresh(self):
        self.update_data(self.lidar.get_image())
        while sdl2.SDL_PollEvent(ctypes.byref(event)) != 0:
            if event.type == sdl2.SDL_QUIT:
                self.running = False
                break
                
        print("Drawing...")
        if self.mode == RAW:
            self.draw_raw()
        elif self.mode == POLAR:
            self.draw_polar()
        elif self.mode == EUCLID_3D:
            self.draw_3D()
             
        print("Refreshing...")
        self.window.refresh()
    
    def run(self):
        
        event = sdl2.SDL_Event()
        while self.running:
            
        
        sdl2.SDL_Quit()
        return 0
