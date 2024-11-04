echo $0:

# Default values for package and file input parameters
package="roscpp_tutorials"
file="talker_listener.launch"

OPTIND=1         # Reset in case getopts has been used previously in the shell

while getopts p:f:m:l: flag
do
    case "${flag}" in
        p)  package=${OPTARG};;
        f)  file=${OPTARG};;
        m)  
            rosmaster_uri=${OPTARG}
            export ROS_MASTER_URI=$rosmaster_uri
            echo "ROS master: $rosmaster_uri"
            ;;
        l)  
            local_ip=${OPTARG}
            export ROS_IP=$local_ip
            export ROS_HOSTNAME=$local_ip
            echo "Local IP: $local_ip"
            ;;
    esac
done

shift $((OPTIND-1))
[ "${1:-}" = "--" ] && shift

echo "roslaunch $package $file $@"

roslaunch $package $file $@
