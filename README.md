# remote_docker_launcher_ros1

This ROS1 package contains tools to support remotely launching ROS1-based subsystems on a remote machine in a Docker container. It utilizes a ROS1 node that can start a Docker container containig ROS1 via SSH on a remote machine. 

A shell script in `docker/launch.sh` is provided that has to be placed into the root folder of the Docker image. This script does the following:

- It sets the `ROS_MASTER_URI`, `ROS_IP`, and `ROS_HOSTNAME` environment variables inside the Docker container right before launching the ROS1-based launch file.
- It invokes roslaunch to start a launch file of a given package. The ROS1 environment and the specified package is assumed to be present inside the Docker container.

A simple docker image can be built using the `Dockerfile` in the `docker` folder for testing the functionality of this package (it will contain a ROS Noetic base installation and the `roscpp_tutorials` package that contains the talker and listener nodes covered in the [ROS beginner tutorials](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)):
```
$ docker build -f Dockerfile -t ros:noetic-cpp-tutorials .
```

## remote_docker_launcher node

The ROS1 remote Docker launcher node is implemented in `remote_docker_launcher.cpp`. It has to be invoked by specifying four command line arguments in the following order:

```
$ rosrun remote_docker_launcher_ros1 remote_docker_launcher USER MACHINE DOCKER_IMAGE DOCKER_COMMAND
```
- `USER` and `MACHINE` are the remote user and the IP or hostname of the remote machine which is accessed via SSH (default port). It is assumed that the remote machine is accessible via SSH without having to enter a password, i.e., an SSH key is created and installed to the remote machine. If you are unfamiliar with this topic, check this [tutorial](https://www.digitalocean.com/community/tutorials/how-to-configure-ssh-key-based-authentication-on-a-linux-server).

- `DOCKER_IMAGE` is the name of the Docker image prepared on the remote machine. It is assumed to contain a ROS1 environment, the desired ROS1 package and the `launch.sh` script mentioned above. A container based on this image is started by the node using the `docker run` command with the `-it --rm --net=host` options.
- `DOCKER_COMMAND` is a string specifying the shell command which will be issued inside the Docker container after it is started, typically:
  ```
  "/launch.sh -m ROS_MASTER_URI -l ROS_LOCAL_IP -p PACKAGE -f LAUNCH_FILE"
  ```
  The arguments of the `launch.sh` are the following:
  - `-m ROS_MASTER_URI`: The ROS master URI in the form of `http://[ROS_MASTER_IP]:11311`. This parameter will be used for specifying the `ROS_MASTER_URI` environment variable inside the container.
  - `-l ROS_LOCAL_IP`: IP (or hostname) of the remote machine containing the Docker container. This parameter will be used for specifying both `ROS_IP` and `ROS_HOSTNAME` environment variables inside the container.
  - `-p PACKAGE`: Name of the ROS1 package
  - `-f LAUNCH_FILE`: The launch file of the package to be launched inside the container

## remote_docker_launcher launch file

To facilitate the parameterized call of the `remote_docker_launcher` node, a launch file `remote_docker_launcher.launch` is also provided in this package. This launch file accepts launch arguments and assembles the necessary command line arguments (especially the docker command) for the `remote_docker_launcher` node. The following launch arguments are available:

- `remote_user`: User of the remote machine
- `remote_machine`: IP address (or hostname) of the remote machine containing the Docker image
- `ros_master_uri`: ROS master URI n the form of `http://[ROS_MASTER_IP]:11311`
- `docker_image`: Name of the Docker image on the remote machine
- `docker_ros_package`: ROS1 package name inside the container
- `docker_roslaunch_file`: Launch file name in the ROS1 package
- `docker_roslaunch_args`: Launch arguments for the launch file (optional)

Usage:
```
$ roslaunch remote_docker_launcher_ros1 remote_docker_launcher.launch \
  remote_user:=<USER> \
  remote_machine:=<HOSTNAME_OR_IP> \
  ros_master_uri:=http://<ROS_MASTER_IP>:11311 \
  docker_image:=<MY_DOCKER_IMAGE> \
  docker_ros_package:=<MY_ROS_PACKAGE> \
  docker_roslaunch_file:=<MY_LAUNCH_FILE>
```

The launch file specifies dummy default vaules for these. You can replace them to appropriate values to avoid specifying the arguments in the command line:
```
$ roslaunch remote_docker_launcher_ros1 remote_docker_launcher.launch
```