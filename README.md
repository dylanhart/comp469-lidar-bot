
# Lidar Bot - COMP469 Final
### Dylan Hart and Kelsey Geiger

## Installation

Run `downloadLibs.sh` to download the `picoborgrev` library.
A python3 version will be compiled and placed in the `picoborgrev3` folder.

## Usage

The `data` folder contains the configuration files for the robot.
`config.json` contains the configuration settings for the robot.
The config file may be overriden using the `BOT_CONFIG` environment variable.

#### Config.json
|Setting|Description|
|---|---|
|`AI`|the name of the ai module to use|
|`START_POS`|x and y starting location of the robot|
|`START_DIR`|vector of the starting direction of the robot|
|`MAP`|relative path to map file|
|`LIDAR_MODULE`|name of the lidar module to use|

The AI module controls the robot.
The AI module is nearly source compatible with the simulator.
`AI.decide` receives the robot object, lidar image, and map as parameters.
One thing to note is that no units are scaled.

The Lidar module allows for multiple lidar implentations.
This is mainly an artifact from testing using the `dummy_lidar` module.

To run the robot do run `main.py`:

```
$ python3 main.py
```

The robot may be stopped with `Ctrl-C`.

## Visualization

A separate process for visualization can be started to view a visualization of the data being seen by the LiDAR.

This visualization process has five different rendering modes to view the data in different ways for debugging purposes.

#### Raw

![Raw plotting](/images/RawPlotting.png)

Select this mode with the 1 key to view raw output from the lidar-reading module.


#### 2D

![2D plotting](/images/PolarPlotting.png)

Select this mode with the 2 key to get the points being read by LiDAR in polar coordinates.


#### 3D Plotting

![3D Point Plotting using Lines](/images/3DPointRendering.png)

Select this mode with the 3 key to get the points being read by LiDAR projected into a 3D space, colored based on signal strength and distance. They are plotted as vertical lines which also scale with distance.


#### 2D Wall Plotting

![2D Wall Plotting](/images/PolarWallPlotting.png)

Select this mode with the 4 key to filter points by quality and distance, assuring no points directly on the robot or with weak signals. The filtered points are then processed to find straight lines between them, assumed to be walls if there are more than two points in a line. The whole wall is extracted and rendered as a 2D line.


#### 3D Wall Plotting

![3D Wall Plotting](/images/3DWallPlotting.png)

Select this mode with the 5 key to filter points and extract walls as in 2D wall plot mode. These points are instead transformed into 3D quadrilaterals and filled to render. These are colored by the distance of the midpoint of the wall.


To quit, press the X in the top corner (left or right depending on OS), or press ESC.

This process can be run with

```
$ python3 lidar_draw.py
```
