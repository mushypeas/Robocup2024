cd ~/Robocup2024;
export ROS_MASTER_URI=http://hsrb.local:11311 export PS1="\[\033[41;1;37m\]<hsrb>\[\033[0m\]\w$ "; # hsrb_mode
conda activate yolov10;
cd /home/tidy/Robocup2024/module/yolov10;
python run_yolov10.py;
