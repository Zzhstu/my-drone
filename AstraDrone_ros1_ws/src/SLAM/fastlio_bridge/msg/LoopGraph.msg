# Paired raw/optimized poses from ONE graph snapshot. Revision changes only
# after a committed loop optimization, not when an ordinary keyframe is added.
std_msgs/Header header
uint64 revision
nav_msgs/Path raw
nav_msgs/Path optimized
