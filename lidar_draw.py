import ciNeuroBotLidar
import sample_read
import lidar_visual
import sys
import numpy
import math

#lidar = ciNeuroBotLidar.Lidar()
lidar = sample_read.Lidar()
visualizer = lidar_visual.LidarVisualizer(360, 360, lidar)

visualizer.run()
lidar.quit = True
