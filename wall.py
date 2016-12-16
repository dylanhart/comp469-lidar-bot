import numpy as np
from operator import itemgetter
import math

def find_walls(filtered):
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
    
    while i < len(points) - 1:
        j = i + 1
        wall_dists = []
        wall_angles = []
        
        angle = 180 * math.atan2(points[j][1] - points[i][1], points[j][0] - points[i][0]) / math.pi
        
        k = i - 1
        l = k + 1

        running_angle = 180 * math.atan2(points[l][1] - points[k][1], points[l][0] - points[k][0]) / math.pi
        
        while ((l-1) > -len(points)) and (-15 < running_angle - angle < 15):
            wall_dists.append([dists[k], quals[k]])
            wall_angles.append(angles[k])
            k = k - 1
            l = k + 1
            running_angle = 180 * math.atan2(points[l][1] - points[k][1], points[l][0] - points[k][0]) / math.pi
        
        wall_dists.reverse()
        wall_angles.reverse()
        
        running_angle = 180 * math.atan2(points[j][1] - points[i][1], points[j][0] - points[i][0]) / math.pi
        
        while ((j+1) < len(points)) and (-15 < running_angle - angle < 15):
            wall_dists.append([dists[j], quals[j]])
            wall_angles.append(angles[i])
            j = j + 1
            i = j - 1
            running_angle = 180 * math.atan2(points[j][1] - points[i][1], points[j][0] - points[i][0]) / math.pi
        
        if(len(wall_dists) >= 2):
            w = Wall(wall_dists, wall_angles)
            walls.append(w)
        
        i = i + 1
    
    return walls

class Wall:
    def __init__(self, distances, angles):
        dists = [d[0] for d in distances]
        quals = [q[1] for q in distances]
        points = []
        
        for i in range(len(dists)):
            c = math.cos(math.pi * angles[i] / 180)
            s = math.sin(math.pi * angles[i] / 180)
            points.append(np.array([c * dists[i], s * dists[i]]))
        
        assumed_angle = None
        
        assumed_angle = math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0])
        best_quality = 0
        
        for i in range(len(distances) - 1):
            j = i + 1
            vec = points[j] - points[i]
            angle = math.atan2(vec[1], vec[0])
            
            avg_qual = (quals[i] + quals[j]) / 2.0
            if(avg_qual > ((quals[best_quality] + quals[best_quality+1])/2.0)):
                assumed_angle = angle
                best_quality = i
        
        perp_angle = assumed_angle + 90
        
        intercept = points[best_quality] if quals[best_quality] > quals[best_quality+1] else points[best_quality+1]
        
        start_inaccurate = points[min(enumerate(angles), key=itemgetter(1))[0]]
        end_inaccurate = points[max(enumerate(angles), key=itemgetter(1))[0]]
        
        #start_x = ()
        #start_z = assumed_slope * start_x
        #self.start = np.array([start_x, start_z])
        self.start = start_inaccurate
        
        #end_x = ()
        #end_z = assumed_slope * end_x
        #self.end = np.array([end_x, end_z])
        self.end = end_inaccurate
