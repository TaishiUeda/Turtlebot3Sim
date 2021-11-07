# Turtlebot3Sim
Gazebo simulator of Turtlebot3 on Docker

## Dependency
Please install adequate [docker](https://www.docker.com/) according to your computers.

- Win10 and Ubuntu with X11 server without OpenGL
- mac with M1 chip hasn't supported yet.

## Usuage

When you use this for the first time, build the docker image by a command;

```bash
docker-compose build ros_master
```

After that, you can start simulator by a command;

```bash
docker-compose up
```
