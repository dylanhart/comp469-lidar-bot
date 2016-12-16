import sys
import sdl2
import sdl2.ext
import ctypes
import math
import time
import numpy as np
import wall

def map_value(value, in_min, in_max, out_min, out_max):
    percent = float(value - in_min) / (in_max/out_max)
    return out_min + (percent * (out_max - out_min))

#returns an OpenGL-style perspective matrix- column major
def perspective(left, right, top, bottom, near, far):
    width = right - left
    height = top - bottom
    depth = far-near
    return np.matrix([[(2*near)/width, 0,             (right+left)/width,                    0],
                     [0,             (2*near)/height, (top+bottom)/height,                   0],
                     [0,             0,               -(far+near)/depth,   -(2*far*near)/depth],
                     [0,             0,               -1,                                   0]])

#returns an OpenGL-style viewport matrix to map 4D points to 3D screenspace- column major
def view(x, y, w, h):
    return np.matrix([[w/2,     0,      0,              0],
                     [0,       h/2,     0,              0],
                     [0,       0,       (MAX_DIST-1)/2, 0],
                     [x+(w/2), y+(h/2), MAX_DIST/2,     1]])

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
        
        self.projection = perspective(-win_w/32, win_w/32, win_h/16, -win_h/16, 1, 6000)
        self.viewport = view(0, 0, win_w, win_h)
        
        self.height = win_h
        self.width = win_w
        self.raw_data = []
        self.points = []

        self.running = True
        self.mode = EUCLID_3D
        
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
        quals = [q[1] for q in self.raw_data]
        colors = []
        
        max_dist = max(dists)
        max_qual = max(quals)
        
        for i in range(len(self.raw_data)):
            r = float(max_qual - quals[i]) / max_qual
            g = float(quals[i])/max_qual
            b = 1.0
            
            gray = dists[i] / max_dist
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
        quals = [q[1] for q in self.raw_data]

        max_dist = max(dists)
        max_qual = max(quals)

        coords = []
        colors = []
        
        for i in range(len(self.raw_data)):
            c = math.cos(math.pi * i / 180)
            s = math.sin(math.pi * i / 180)
            
            x = (c * dists[i]) / MAX_DIST
            z = (s * dists[i]) / MAX_DIST
            
            r = float(max_qual - quals[i]) / max_qual
            g = float(quals[i])/max_qual
            b = 1.0
            
            gray = 1.0 - (dists[i] / max_dist)
            r = r * gray
            g = g * gray
            b = gray
            
            color = sdl2.ext.Color(int(255 * r), int(255 * g), int(255 * b))
            colors.append(color)
            
            #transform to screen space
            point = (np.array([x, z, 0, 1]) * self.viewport).A1
            point = [int(point[0] / point[3]), int(point[1] / point[3])]
            
            coords.append(point)
        
        win_surf = self.window.get_surface()
        
        black = sdl2.ext.Color(0, 0, 0)
        sdl2.ext.fill(win_surf, black)
        
        pix_view = sdl2.ext.pixels2d(win_surf)
        for i in range(len(coords)):
            color = colors[i]
            pixel_x = coords[i][0]
            pixel_y = coords[i][1]
            pix_view[pixel_y][pixel_x] = color
        
    
    def draw_3D(self):
        dists = [d[0] for d in self.raw_data]
        quals = [q[1] for q in self.raw_data]
        dists = dists[90:270]
        
        max_dist = max(dists)
        max_qual = max(quals)

        coords = []
        colors = []
        
        for i in range(len(dists)):
            c = -math.cos(math.pi * i / 180)
            s = math.sin(math.pi * i / 180)
            
            x = c * dists[i]
            z = s * dists[i]
             
            if(z <= 0.0):
                continue
            
            r = float(max_qual - quals[i]) / max_qual
            g = float(quals[i])/max_qual
            b = 1.0
            
            gray = 1.0 - (dists[i] / max_dist)
            r = r * gray
            g = g * gray
            b = gray
            
            color = sdl2.ext.Color(int(255*r), int(255*g), int(255*b))
            colors.append(color)
            
            floor = (np.array([x, self.height, z, 1]) * self.projection).A1
            ceil = (np.array([x, -self.height, z, 1]) * self.projection).A1

            floor = floor/floor[3]
            ceil = ceil/ceil[3]
            
            floor = (floor * self.viewport).A1
            ceil = (ceil * self.viewport).A1
            
            floor = np.array([int(floor[0]), int(floor[1]), int(floor[2])])
            ceil = np.array([int(ceil[0]), int(ceil[1]), int(ceil[2])])
            
            coords.append((floor, ceil))
        
        win_surf = self.window.get_surface()
        
        black = sdl2.ext.Color(0, 0, 0)
        sdl2.ext.fill(win_surf, black)
        
        for i in range(len(coords)):
            x1 = coords[i][0][0]
            y1 = coords[i][0][1]
            x2 = coords[i][1][0]
            y2 = coords[i][1][1]
            
            if x < 0 or x > self.width:
                continue
            
            values = (x1, y1, x2, y2)
            sdl2.ext.line(win_surf, colors[i], values)
    
    def filtered_points(self):
        dist_filtered = [d for d in enumerate(self.raw_data) if d[1][0] >= 150] #ignore things still on the robot
        qual_filtered = [q for q in dist_filtered if q[1][1] > 0] #ignore things that cannot be read correctly
        return qual_filtered
    
    def draw_filtered_polar(self):
        points = self.filtered_points()
        angles = [a[0] for a in points]
        dists = [d[1][0] for d in points]
        quals = [q[1][1] for q in points]
        
        max_dist = max(dists)
        max_qual = max(quals)
        
        coords = []
        colors = []
        
        for i in range(len(points)):
            c = math.cos(math.pi * angles[i] / 180)
            s = math.sin(math.pi * angles[i] / 180)
            
            x = (dists[i] * c) / MAX_DIST
            z = (dists[i] * s) / MAX_DIST
        
            r = float(max_qual - quals[i]) / max_qual
            g = float(quals[i])/max_qual
            b = 1.0
            
            gray = 1.0 - (dists[i] / max_dist)
            r = r * gray
            g = g * gray
            b = gray
            
            color = sdl2.ext.Color(int(255 * r), int(255 * g), int(255 * b))
            colors.append(color)
            
            #transform to screen space
            point = (np.array([x, z, 0, 1]) * self.viewport).A1
            point = [int(point[0] / point[3]), int(point[1] / point[3])]
            
            coords.append(point)
        
        win_surf = self.window.get_surface()
        
        black = sdl2.ext.Color(0, 0, 0)
        sdl2.ext.fill(win_surf, black)
        
        pix_view = sdl2.ext.pixels2d(win_surf)
        for i in range(len(coords)):
            color = colors[i]
            pixel_x = coords[i][0]
            pixel_y = coords[i][1]
            pix_view[pixel_y][pixel_x] = color
    
    def draw_pretty_polar(self):
        walls = wall.find_walls(self.filtered_points())
        win_surf = self.window.get_surface()
        
        white = sdl2.ext.Color(255, 255, 255)
        black = sdl2.ext.Color(0, 0, 0)
        sdl2.ext.fill(win_surf, black)
        
        for w in walls:
            
            #transform to screen space
            start = (np.array([w.start[1]/MAX_DIST, w.start[0]/MAX_DIST, 0, 1]) * self.viewport).A1
            start = [int(start[0] / start[3]), int(start[1] / start[3])]

            end = (np.array([w.end[1]/MAX_DIST, w.end[0]/MAX_DIST, 0, 1]) * self.viewport).A1
            end = [int(end[0] / end[3]), int(end[1] / end[3])]
            
            values = (start[0], start[1], end[0], end[1])
            sdl2.ext.line(win_surf, white, values)
    
    def fill_quad(self, surf, quad, color):
        floor = (quad[0], quad[2])
        ceil = (quad[1], quad[3])
        
        #swap if the start is on the right
        if(floor[1][0] < floor[0][0]):
            floor = (floor[1], floor[0])
            ceil = (ceil[1], ceil[0])
        
        dx = floor[1][0] - floor[0][0]
        dy_floor = floor[1][1] - floor[0][1]
        dy_ceil = ceil[1][1] - ceil[0][1]
                
        derr_floor = abs(dy_floor) * 2
        derr_ceil = abs(dy_ceil) * 2
        
        err_floor = 0
        err_ceil = 0
        
        y_ceil = ceil[0][1]
        y_floor = floor[0][1]
        x = floor[0][0]
        
        while x < floor[1][0] and y_floor < y_ceil:
            
            if 0 < x < self.width:
                values = (self.width - x, y_floor, self.width - x, y_ceil)
                sdl2.ext.line(surf, color, values)
            
            err_floor += derr_floor
            err_ceil += derr_ceil
            
            if err_floor > dx:
                adjustment = 1 if (floor[1][1] > floor[0][1]) else -1
                y_floor += adjustment
                err_floor -= dx * 2
            
            if err_ceil > dx:
                adjustment = 1 if (ceil[1][1] > ceil[0][1]) else -1
                y_ceil += adjustment
                err_ceil -= dx * 2
            
            x += 1
    
    def draw_pretty_3D(self):
        filtered = self.filtered_points()
        
        max_dist = max([d[1][0] for d in filtered])
        max_qual = max([q[1][1] for q in filtered])
        
        walls = wall.find_walls(filtered)
        
        quads = []
        colors = []
        
        for w in walls:
            
            x_s = w.start[0]
            z_s = w.start[1]
            
            x_e = w.end[0]
            z_e = w.end[1]
            
            if (z_s < 0 and z_e < 0):
                continue
            
            #set up quad polygon
            points = []
            points.append(np.array([x_s, self.height, z_s, 1]))
            points.append(np.array([x_s, -self.height, z_s, 1]))
            points.append(np.array([x_e, self.height, z_e, 1]))
            points.append(np.array([x_e, -self.height, z_e, 1]))
            
            #print("Points: \n\t", points, "\n")
            
            #calculate color as a gray based on distance of the midpoint of the wall from the LIDAR
            v = w.start + ((w.end - w.start)/2.0)
            distance = math.sqrt(np.dot(v, v))
            
            r = (distance / max_dist)
            g = 0.3 + (0.7 * (1.0 - (distance / max_dist)))
            b = 1.0 - (distance / max_dist)
            
            color = sdl2.ext.Color(int(255 * r), int(255 * g), int(255 * b))
            colors.append(color)
            
            for i in range(4):
                #project into clip space
                points[i] = (points[i] * self.projection).A1
                
                #do normalized device coordinate division
                points[i] = points[i]/points[i][3]
                
                #transform into screen space
                points[i] = (points[i] * self.viewport).A1
                
                #take the 2D coordinates
                points[i] = np.array([int(points[i][0]), int(points[i][1])])
            
            #print("Transformed Points: \n\t", points, "\n\n")
            
            #add to quad list
            quads.append(points)
            
        win_surf = self.window.get_surface()
        
        black = sdl2.ext.Color(0, 0, 0)
        sdl2.ext.fill(win_surf, black)
        
        for i in range(len(quads)):
            self.fill_quad(win_surf, quads[i], colors[i])
    
    def refresh(self):
        event = sdl2.SDL_Event()
        self.update_data(self.lidar.get_image())
        while sdl2.SDL_PollEvent(ctypes.byref(event)) != 0:
            if event.type == sdl2.SDL_QUIT:
                self.running = False
                break
            elif event.type == sdl2.SDL_KEYDOWN:
                if event.key.keysym.sym == sdl2.SDLK_ESCAPE:
                    self.running = False 
                elif event.key.keysym.sym == sdl2.SDLK_1:
                    self.mode = RAW
                elif event.key.keysym.sym == sdl2.SDLK_2:
                    self.mode = POLAR
                elif event.key.keysym.sym == sdl2.SDLK_3:
                    self.mode = EUCLID_3D
                elif event.key.keysym.sym == sdl2.SDLK_4:
                    self.mode = PRETTY_POLAR
                elif event.key.keysym.sym == sdl2.SDLK_5:
                    self.mode = PRETTY_3D
                        
        if self.mode == RAW:
            self.draw_raw()
        elif self.mode == POLAR:
            self.draw_polar()
        elif self.mode == EUCLID_3D:
            self.draw_3D()
        elif self.mode == PRETTY_POLAR:
            self.draw_pretty_polar()
        elif self.mode == PRETTY_3D:
            self.draw_pretty_3D()
        
        self.window.refresh()
    
    def run(self):
        
        while self.running:
            start = time.clock()
            self.refresh()
            end = time.clock()
            
            if (end-start) < 0.167:
                time.sleep(0.167 - (end-start))
        
        sdl2.SDL_Quit()
        return 0
