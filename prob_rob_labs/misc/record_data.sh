LOG_FILE=/run/host/workdir/local_stt2126/ros2_ws/src/prob_rob_labs_ros_2/prob_rob_labs/misc/feature_mean.csv
max_runs=5
count=1

echo "running up to $max_runs loops"

while [ $count -le $max_runs ]
do
    echo "Run $count of $max_runs"
    ros2 servive call /reset_simulation std_srvs/srv/Empty {}
    sleep 1

    ros2 launch prob_rob_labs door_opener_launch.py  forward_speed:=0.0 threshold:=240
    PID=$!

    wait $PID
    count=$((count+1))

done

echo "data saved $LOG_FILE"