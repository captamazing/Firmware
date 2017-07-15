no_sim=1 make posix_sitl_default gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
no_gpu=0
est="ekf2"
while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    --no_gpu)
    no_gpu=1
    shift # past argument
    ;;
    --use_ekf)
    est="ekf2"
    shift # past argument
    ;;
    --use_lpe)
    est="lpe"
    shift # past argument
    ;;
esac
done
roslaunch px4 posix_sitl.launch vehicle:=ursa est:="$est" no_gpu:="$no_gpu"