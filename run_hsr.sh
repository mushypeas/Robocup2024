tmux new-session -s robocup2024

tmux set -g mouse on


tmux split-window -h
tmux split-window -h
tmux split-window
tmux select-pane -t 1
tmux split-window
tmux split-window
tmux select-pane -t 0

tmux send "source venv/bin/activate" C-m
tmux send "hsrb_mode" C-m
tmux send "python main_client.py" C-m

tmux select-pane -t 1
tmux send "source venv/bin/activate" C-m
tmux send "hsrb_mode" C-m
tmux send "cd module/yolov7" C-m
tmux send "python run_yolov7.py" C-m

#tmux select-pane -t 2
#tmux send "source venv/bin/activate" C-m
#tmux send "hsrb_mode" C-m
#tmux send "python module/ByteTrack/tools/demo_track.py" C-m

tmux select-pane -t 3
tmux send "source venv/bin/activate" C-m
tmux send "hsrb_mode" C-m
tmux send "cd robotside_runfile/hsr_head_display" C-m
tmux send " expect shell_monitor_and_stt" C-m

tmux select-pane -t 4
tmux send "source venv/bin/activate" C-m
tmux send "hsrb_mode" C-m
tmux send "cd module/openpose" C-m
tmux send "python run_openpose.py" C-m


#tmux select-pane -t 5
#tmux send "source venv/bin/activate" C-m
#tmux send "hsrb_mode" C-m
#tmux send "python module/yolov7/floor_yolo.py" C-m
