import numpy as np
from operators import itemgetter
import math
import sdl2

class Wall:
    def __init__(self, distances, angles):
        dists = [d[0] for d in distances]
        quals = [q[0] for q in distances]
        points = []
        
        for i in range(len(dists)):
            c = math.cos(math.PI * angles[i] / 180)
            s = math.sin(math.PI * angles[i] / 180)
            points.append(np.array([c * dists[i], s * dists[i]]))
        
        assumed_slope = None
        best_quality = 0
        
        for i in range(len(distances) - 1):
           j = i + 1
           vec = points[j] - points[i]
           slope = vec[1] / vec[0]
           
           avg_qual = (quals[i] + quals[j]) / 2.0
           if(avg_qual > ((quals[best_quality] + quals[best_quality+1])/2.0)):
               assumed_slope = slope
               best_quality = i
           
       perp_slope = -1/assumed_slope
       intercept = points[best_quality] if quals[best_quality] > quals[best_quality+1] else points[best_quality+1]
       
       start_inaccurate = points[min(enumerate(angles), key=itemgetter(1))[0]]
       end_inaccurate = points[max(enumerate(angles), key=itemgetter(1))[0]]
       
       start_x = ((start_inaccurate[1]-perp_slope*start_inaccurate[0])-(intercept[1]-assumed_slope*intercept[0]))/(assumed_slope - perp_slope)
       start_z = assumed_slope * start_x
       self.start = np.array([start_x, start_z])
       
       end_x = ((end_inaccurate[1]-perp_slope*end_inaccurate[0])-(intercept[1]-assumed_slope*intercept[0]))/(assumed_slope - perp_slope)
       end_z = assumed_slope * end_x
       self.end = np.array([end_x, end_z])
