# Inverse kinematics solution — one entry per cable leg
std_msgs/Header header

bool success                         # false if target is unreachable
float64[] leg_lengths                # commanded length for each cable [m]
float64[] leg_velocities             # desired velocity per cable [m/s]
geometry_msgs/Pose end_effector_pose # FK-verified pose
string failure_reason                # populated when success == false
