#include "ros/ros.h"
#include <signal.h>
#include <string>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "remote_docker_launcher");

    ros::NodeHandle nh;

    if(argc<4){
        ROS_ERROR("Too few arguments. Remote user (arg 1), remote machine (arg 2), and command (arg 3) has to be specified.");
    }

    std::string user = argv[1];
    std::string machine = argv[2];
    std::string command = argv[3];

    ROS_INFO("Remote user: %s", user.c_str());
    ROS_INFO("Remote machine: %s", machine.c_str());
    ROS_INFO("Command: %s", command.c_str());

    std::string startup_command = "ssh -t " + user + "@" + machine + " \"bash -i -c '" + command + "'\"";

    ROS_INFO("Assembled startup command:\n----------------------------\n%s\n----------------------------", startup_command.c_str());

    // Invoke the startup command
    system(startup_command.c_str());
    
    return 0;
}
