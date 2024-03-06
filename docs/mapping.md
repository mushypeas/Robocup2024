# Mapping


## Hector slam

1. Log into the robot and start up SLAM
```shell
source /opt/ros/noetic/setup.bash
rosnode kill /pose_integrator
# hector slam
rosrun hector_mapping hector_mapping _map_size:=2048 _map_resolution:=0.05 _pub_map_odom_transform:=true _scan_topic:=/hsrb/base_scan _use_tf_scan_transformation:=true _map_update_angle_thresh:=2.0 _map_update_distance_thresh:=0.10 _scan_subscriber_queue_size:=1 _update_factor_free:=0.39 _update_factor_occupied:=0.85 _base_frame:=base_link
```
2. show rviz
```shell
<hsrb>~$ rviz
```
3. Save the map
```shell
source /opt/ros/noetic/setup.bash
rosrun map_server map_saver
```

4. move map file to MAP_PATH 
```shell
MAP_PATH=/etc/opt/tmc/robot/conf.d/map
```


ref
- hsr.io mapping manual link : [`Ours`](https://docs.hsr.io/hsr_develop_manual_en/customize/base_map.html?highlight=map)
