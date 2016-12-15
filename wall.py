import numpy as np
from operator import itemgetter
import math

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
