#!/bin/bash

FORWARD_SPEED=0.1
LOG_FILE=~/feature_mean_log.csv
MAX_RUNS=5
COUNT=1

echo "Running up to $MAX_RUNS loops"
echo "Logging all data to $LOG_FILE"

# Loop until COUNT exceeds MAX_RUNS
while [ $COUNT -le $MAX_RUNS ]
do
    echo "=== Run $COUNT of $MAX_RUNS ==="

    # Reset simulation
    ros2 service call /reset_world std_srvs/srv/Empty
    sleep 1

    # Log run start
    echo "=== RUN $COUNT START ===" >> "$LOG_FILE"

    # Launch the node
    ros2 launch prob_rob_labs Lab3_DataCollection_launch.py &
    PID=$!

    # Wait until it finishes
    wait $PID

    # Log run end
    echo "=== RUN $COUNT END ===" >> "$LOG_FILE"
    echo "Run $COUNT finished."

    # Increment counter
    COUNT=$((COUNT + 1))
done


echo "All $MAX_RUNS runs complete."
echo "Data saved to $LOG_FILE"
