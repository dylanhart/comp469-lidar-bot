
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


