# Start roscore
roscore &

sleep 2

# Optional: launch your odometry node or other necessary nodes
roslaunch first_project launch.launch &
echo "Launching first_project..."

# Give everything a moment to spin up
sleep 5

# Start rosbag recording
rosbag record -O ../data/my_drive.bag /odom /gps_odom /tf &
echo "Recording started..."

sleep 2

# Start rosbag playback 
rosbag play --clock -r 3.0 ../data/project.bag & 
echo "Playing back project.bag..."
# Optional: wait for a few seconds to ensure playback starts
sleep 2

# Optional: wait for user to press [ENTER] to stop
read -p "Press ENTER to stop recording..."

# Kill the rosbag process cleanly
pkill -f "rosbag record"
