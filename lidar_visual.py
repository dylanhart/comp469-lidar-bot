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
    return np.matrix([[2*near/width, 0,             (right+left)/width,                  0],
                     [0,             2*near/height, (top+bottom)/height,                 0],
                     [0,             0,             -(far+near)/depth, -(2*far*near)/depth],
                     [0,             0,             -1,                                 0]])

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
        
        self.projection = perspective(-win_w/8, win_w/8, win_h/4, -win_h/4, 1, MAX_DIST)
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
        quals = [q[1] for q in self.raw_data]

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
            
            gray = 1.0 - (dists[i] / MAX_DIST)
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
            
            gray = 1.0 - (dists[i] / MAX_DIST)
            r = r * gray
            g = g * gray
            b = gray
            
            color = sdl2.ext.Color(0, 0, 0)
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
        
        pix_view = sdl2.ext.pixels2d(win_surf)
        for i in range(len(coords)):
            x = coords[i][0][0]
            y = coords[i][0][1]
            
            color = colors[i]

            while y < coords[i][1][1]:
                if y > 0 and y < self.height:
                    pix_view[x][y] = color
                y = y + 1
    
    def filtered_points(self):
        dist_filtered = [d for d in enumerate(self.raw_data) if d[1][0] >= 150] #ignore things still on the robot
        qual_filtered = [q for q in dist_filtered if q[1][1] > 0] #ignore things that cannot be read correctly
        return qual_filtered
    
    def draw_filtered_polar(self):
        points = self.filtered_points()
        angles = [a[0] for a in points]
        dists = [d[1][0] for d in points]
        quals = [q[1][1] for q in points]
        
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
            
            gray = 1.0 - (dists[i] / MAX_DIST)
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
    
    def find_walls(self):
        filtered = self.filtered_points()
        angles = [a[0] for a in filtered]
        distances = [d[1] for d in filtered]
        dists = [d[0] for d in distances]
        quals = [q[1] for q in distances]
        points = []
        
        for i in range(len(dists)):
            c = math.cos(math.pi * angles[i] / 180)
            s = math.sin(math.pi * angles[i] / 180)
            
            x = (c * dists[i])
            z = (s * dists[i])
            
            points.append(np.array([x, z]))

        walls = []
        
        i = 0

        while (i+1) < len(points):
            j = i + 1
            wall_dists = []
            wall_angles = []
            
            angle = 180 * math.atan2(points[j][1] - points[i][1], points[j][0] - points[i][0]) / math.pi
            
            k = i - 1
            l = k + 1
            running_angle = 180 * math.atan2(points[k][1] - points[l][1], points[k][0] - points[l][0]) / math.pi
            
            while ((l-1) > -len(points)) and (-15 < running_angle - angle < 15):
                wall_dists.append([dists[k], quals[k]])
                wall_angles.append(angles[l])
                k = k - 1
                l = k - 1
                running_angle = 180 * math.atan2(points[k][1] - points[l][1], points[k][0] - points[l][0]) / math.pi
            
            wall_dists.reverse()
            wall_angles.reverse()
            
            running_angle = 180 * math.atan2(points[j][1] - points[i][1], points[j][0] - points[i][0]) / math.pi
            
            while ((j+1) < len(points)) and (-15 < running_angle - angle < 15):
                wall_dists.append([dists[j], quals[j]])
                wall_angles.append(angles[i])
                j = j + 1
                i = j - 1
                running_angle = 180 * math.atan2(points[j][1] - points[i][1], points[j][0] - points[i][0]) / math.pi
            
            if(len(wall_dists) > 2):
                w = wall.Wall(wall_dists, wall_angles)
                walls.append(w)
            
            i = i + 1
        
        return walls
    
    def draw_pretty_polar(self):
        walls = self.find_walls()
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
        
    def refresh(self):
        event = sdl2.SDL_Event()
        self.update_data(self.lidar.get_image())
        walls = False
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
                elif self.mode == PRETTY_POLAR and event.key.keysym.sym == sdl2.SDLK_SPACE:
                    walls = True
                
        if self.mode == RAW:
            self.draw_raw()
        elif self.mode == POLAR:
            self.draw_polar()
        elif self.mode == EUCLID_3D:
            self.draw_3D()
        elif self.mode == PRETTY_POLAR and not walls:
            self.draw_pretty_polar()
        else:
            self.draw_filtered_polar()
        
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
