#!/bin/bash

# 입력받은 인자를 변수에 저장
start_index=$1

# 첫 번째 명령 실행
roslaunch openni2_launch openni2.launch &
ROS_PID=$!

# 몇 초 동안 대기하여 첫 명령이 실행될 시간을 줍니다.
sleep 5

# 두 번째 명령 실행
python openni2_ros_capture_v2.py --start_idx $start_index

# 첫 번째 명령 중지
kill $ROS_PID
