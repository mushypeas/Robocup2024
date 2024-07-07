#!/bin/bash


# 인자 파싱
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --idx) start_index="$2"; shift ;;
        --input-class) input_classes="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# 입력값 확인
if [ -z "$input_classes" ]; then
    echo "Error: --input-class is required."
    exit 1
fi

# 첫 번째 명령 실행
roslaunch openni2_launch openni2.launch &
ROS_PID=$!

# 몇 초 동안 대기하여 첫 명령이 실행될 시간을 줍니다.
sleep 1.5


# 데이터 수집 및 라벨링 병렬 실행
python openni2_ros_capture_v4.py --start_idx $start_index --input_classes "$input_classes"
