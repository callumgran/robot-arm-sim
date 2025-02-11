ros2 topic pub /joint_trajectory_controller/joint_trajectory \
trajectory_msgs/JointTrajectory '{
  header: {stamp: {sec: 0, nanosec: 0}},
  joint_names: ["joint_1", "joint_2", "joint_3"],
  points: [
    { positions: [1, -1, 1], time_from_start: {sec: 2, nanosec: 0} }
  ]
}'
