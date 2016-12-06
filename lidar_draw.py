import ciNeuroBotLidar
import lidar_visual
import sys
import numpy
import math

lidar = ciNeuroBotLidar.Lidar()
visualizer = lidar_visual.LidarVisualizer(360, 480, lidar)

visualizer.run()
