# Turtlebot3Sim
Gazebo simulator of Turtlebot3 on Docker.<br />
This includes;
- Building map by slam
- Save the created map
- Random walk by move_base with randomly published goals

Goal positions will be generated and published by "goal_manager" packege in this repository.<br/>
Please refer to [README.md](./goal_manager/README.md) for the detail.

If you want to use "goal_manager" without docker, just copy the directory "goal_manager" to "\<your_root\>/catkin_ws/src" and build it by "catkin_make".

## Dependency
Please install adequate [docker and docker-compose](https://www.docker.com/) according to your computers.

- It was confirmed that the programs can work with Win10 and Ubuntu with X11 server without OpenGL
- mac with M1 chip hasn't supported yet.

## How to build the docker image

When you use this for the first time, build the docker image by a command;

```bash
docker-compose build ros_world
```
After that, you can find a new docker image named "turtlebot3sim".
Then, you can start simulator by commands as followings.
 
## Usuage

For your convinient, I reccomend to write bash or MS-DOS scripts to execute commands shown as followings.

### Random walk

This repository has already had a map of the world of turtlebot3_gazebo.
To start random walk, input the command;
```bash
docker-compose -f .\docker-compose.yaml -f .\docker-compose.nav_random.yaml up -d
```
Then, a robot in gazebo will start running about 30 second after the window of gazebo is launched.
Please wait patiantly.

To quit,
```bash
docker-compose -f .\docker-compose.yaml -f .\docker-compose.nav_random.yaml down
```

### Building and save map

To start,
```bash
docker-compose -f .\docker-compose.yaml -f .\docker-compose.buildmap.yaml up -d
```

Then, a map will be built gradualy. To save the map, input a command;
```bash
docker-compose -f .\docker-compose.yaml -f .\docker-compose.savemap.yaml up -d
```

Files of the map will be saved in ```Turtlebot3Sim/map/``` on your computer. <br />
To quit,
```bash
docker-compose -f .\docker-compose.yaml -f .\docker-compose.buildmap.yaml -f .\docker-compose.savemap.yaml down
```

