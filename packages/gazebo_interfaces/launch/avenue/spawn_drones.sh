#!/bin/bash

NUM_DRONES=$(rosparam get /number_of_drones)
DRONE_NAME=$(rosparam get /drone_name)

LIDAR_MIN_RANGE=$(rosparam get /lidar_min_range)
LIDAR_MAX_RANGE=$(rosparam get /lidar_max_range)
VERTICAL_MIN_ANGLE_DEG=$(rosparam get /vertical_min_angle)
VERTICAL_MAX_ANGLE_DEG=$(rosparam get /vertical_max_angle)
GAUSSIAN_NOISE=$(rosparam get /gaussian_noise)
RANGE_NOISE_SLOPE=$(rosparam get /range_noise_slope)

# Convert angles from degrees to radian
VERTICAL_MIN_ANGLE_RAD=$(echo "scale=10; $VERTICAL_MIN_ANGLE_DEG * 3.141592653589793 / 180" | bc)
VERTICAL_MAX_ANGLE_RAD=$(echo "scale=10; $VERTICAL_MAX_ANGLE_DEG * 3.141592653589793 / 180" | bc)

USING_SIMULATION=$(rosparam get /using_simulation)

echo "Spawning $NUM_DRONES drones with namespace $DRONE_NAME"

# If not given, use px4 drone pilot. Check launch/pilot folder for the available pilots
if [[ "$USING_SIMULATION" == "true" ]]; then
    PILOT_TYPE="simulation"
elif [[ "$DRONE_NAME" == "iris_velodyne" ]]; then
    PILOT_TYPE="px4"
elif [[ "$DRONE_NAME" == "intelaero_bugwright" ]]; then
    PILOT_TYPE="gazebo"
else
    PILOT_TYPE="simulation"
fi

# Loop through the number of drones and launch each
for ((i=0; i<NUM_DRONES; i++)); do
  echo "Launching ${DRONE_NAME}_${i}..."
    roslaunch chroma_gazebo_interfaces spawn_drone.launch drone_name:=$DRONE_NAME pilot_type:=$PILOT_TYPE id:=${i} lidar_min_range:=$LIDAR_MIN_RANGE lidar_max_range:=$LIDAR_MAX_RANGE vertical_min_angle:=$VERTICAL_MIN_ANGLE_RAD vertical_max_angle:=$VERTICAL_MAX_ANGLE_RAD gaussian_noise:=$GAUSSIAN_NOISE range_noise_slope:=$RANGE_NOISE_SLOPE  &
done

# Wait for all launched processes to complete
wait