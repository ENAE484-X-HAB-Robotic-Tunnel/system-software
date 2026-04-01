# Detected target pose in world frame
std_msgs/Header header

geometry_msgs/Pose pose # 6-DOF target (position + quaternion)
float64 confidence      # detection confidence [0.0, 1.0]
bool is_valid           # set false if detection failed
string target_id        # optional label / fiducial ID
