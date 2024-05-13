cd ~/Robocup2024;
export ROS_MASTER_URI=http://hsrb.local:11311 export PS1="\[\033[41;1;37m\]<hsrb>\[\033[0m\]\w$ "; # hsrb_mode
source ./venv/bin/activate;
cd /module/waver_detector;
python run_openpose.py;