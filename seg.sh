cd ~/Robocup2024;
export ROS_MASTER_URI=http://hsrb.local:11311 export PS1="\[\033[41;1;37m\]<hsrb>\[\033[0m\]\w$ "; # hsrb_mode
source ./venv/bin/activate;
source /home/tidy/deeplab_ws/devel/setup.bash
cd /home/tidy/deeplab_ws/src/deeplab_ros/nodes;
python deeplab_ros_node.py;

