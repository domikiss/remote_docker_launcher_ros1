# remote_launcher_ros1

This ROS1 package contains tools to support remotely launching ROS1-based subsystems on a remote machine natively or in a Docker container. It utilizes ROS1 nodes that can start a roslaunch file via SSH on a remote machine, optionally in a Docker container. 

A shell script in `docker/launch.sh` is provided that has to be placed into the home folder of the selected user of the remote machine, or into the root folder of the Docker image. This script does the following:

- It sets the `ROS_MASTER_URI`, `ROS_IP`, and `ROS_HOSTNAME` environment variables in the remote machine or inside the remote Docker container right before launching the ROS1-based launch file.
- It invokes roslaunch to start a launch file of a given package. The ROS1 environment and the specified package is assumed to be present inside the remote machine or remote Docker container.

A simple docker image can be built using the `Dockerfile` in the `docker` folder for testing the functionality of this package (it will contain a ROS Noetic base installation and the `roscpp_tutorials` package that contains the talker and listener nodes covered in the [ROS beginner tutorials](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)):
```
$ docker build -f Dockerfile -t ros:noetic-cpp-tutorials .
```


## remote_launcher node

The ROS1 remote launcher node is implemented in `src/remote_launcher.cpp`. It has to be invoked by specifying three command line arguments in the following order:

```
$ rosrun remote_launcher_ros1 remote_launcher USER MACHINE COMMAND
```
- `USER` and `MACHINE` are the remote user and the IP or hostname of the remote machine which is accessed via SSH (default port). It is assumed that the remote machine is accessible via SSH without having to enter a password, i.e., an SSH key is created and installed to the remote machine. If you are unfamiliar with this topic, check this [tutorial](https://www.digitalocean.com/community/tutorials/how-to-configure-ssh-key-based-authentication-on-a-linux-server).

- `COMMAND` is a string specifying the shell command which will be issued on the remote computer, typically:
  ```
  "/launch.sh -m ROS_MASTER_URI -l ROS_LOCAL_IP -p PACKAGE -f LAUNCH_FILE"
  ```
  The arguments of the `launch.sh` are the following:
  - `-m ROS_MASTER_URI`: The ROS master URI in the form of `http://[ROS_MASTER_IP]:11311`. This parameter will be used for specifying the `ROS_MASTER_URI` environment variable in the remote session.
  - `-l ROS_LOCAL_IP`: IP (or hostname) of the remote machine. This parameter will be used for specifying both `ROS_IP` and `ROS_HOSTNAME` environment variables inside the container.
  - `-p PACKAGE`: Name of the ROS1 package
  - `-f LAUNCH_FILE`: The launch file of the package to be launched inside the container


## remote_launcher launch file

To facilitate the parameterized call of the `remote_launcher` node, a launch file `remote_launcher.launch` is also provided in this package. This launch file accepts launch arguments and assembles the necessary command line arguments (especially the remote shell command) for the `remote_launcher` node. The following launch arguments are available:

- `remote_user`: User of the remote machine
- `remote_machine`: IP address (or hostname) of the remote machine
- `ros_master_uri`: ROS master URI in the form of `http://[ROS_MASTER_IP]:11311`
- `ros_package`: ROS1 package name available on the remote machine
- `roslaunch_file`: Launch file name in the selected ROS1 package
- `roslaunch_args`: Launch arguments for the launch file (optional)

Usage:
```
$ roslaunch remote_launcher_ros1 remote_launcher.launch \
  remote_user:=<USER> \
  remote_machine:=<HOSTNAME_OR_IP> \
  ros_master_uri:=http://<ROS_MASTER_IP>:11311 \
  ros_package:=<MY_ROS_PACKAGE> \
  roslaunch_file:=<MY_LAUNCH_FILE>
```

The launch file specifies dummy default values for these. You can replace them with appropriate values to avoid specifying the arguments in the command line:
```
$ roslaunch remote_launcher_ros1 remote_launcher.launch
```


## remote_docker_launcher node

The ROS1 remote Docker launcher node is implemented in `src/remote_docker_launcher.cpp`. It has to be invoked by specifying four command line arguments in the following order:

```
$ rosrun remote_launcher_ros1 remote_docker_launcher USER MACHINE DOCKER_IMAGE DOCKER_COMMAND
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
- `ros_master_uri`: ROS master URI in the form of `http://[ROS_MASTER_IP]:11311`
- `docker_image`: Name of the Docker image on the remote machine
- `docker_ros_package`: ROS1 package name inside the container
- `docker_roslaunch_file`: Launch file name in the selected ROS1 package
- `docker_roslaunch_args`: Launch arguments for the launch file (optional)

Usage:
```
$ roslaunch remote_launcher_ros1 remote_docker_launcher.launch \
  remote_user:=<USER> \
  remote_machine:=<HOSTNAME_OR_IP> \
  ros_master_uri:=http://<ROS_MASTER_IP>:11311 \
  docker_image:=<MY_DOCKER_IMAGE> \
  docker_ros_package:=<MY_ROS_PACKAGE> \
  docker_roslaunch_file:=<MY_LAUNCH_FILE>
```

The launch file specifies dummy default values for these. You can replace them with appropriate values to avoid specifying the arguments in the command line:
```
$ roslaunch remote_launcher_ros1 remote_docker_launcher.launch
```
