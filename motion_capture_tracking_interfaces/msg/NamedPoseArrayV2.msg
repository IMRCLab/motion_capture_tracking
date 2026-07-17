std_msgs/Header header  # depending on the setting, the timestamp might be in ROS time when the information arrived or in camera time
uint64 timestamp        # vendor specific, in microseconds
LatencyInfo[] latencies
NamedPose[] poses
