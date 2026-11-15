# One rack found by the LiDAR leg-pattern detector.
# Everything is in header.frame_id (base_footprint), so (0, 0) is the robot centre.
std_msgs/Header header
geometry_msgs/Pose2D pose   # rack centre x, y (m); theta = rack axis, folded into [-pi/4, pi/4) because a square rack looks the same every 90 degrees
uint8 legs_matched          # 2, 3 or 4 of the four legs were actually seen
float32 confidence          # 0.0 .. 1.0
float32 side                # leg-centre to leg-centre spacing used for the match (m)
