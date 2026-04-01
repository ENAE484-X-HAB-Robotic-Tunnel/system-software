# Real-time platform state published by the controller
std_msgs/Header header

geometry_msgs/Pose current_pose # estimated end-effector pose (FK)
float64[] current_leg_lengths   # measured/estimated cable lengths [m]
float64[] leg_errors            # error per cable (commanded - actual) [m]
float64 pose_error              # scalar total pose error [m + rad combined]
bool at_target                  # true when pose_error < threshold
string phase                    # IDLE | TARGETING | MOVING | DOCKING | DONE | FAULT
