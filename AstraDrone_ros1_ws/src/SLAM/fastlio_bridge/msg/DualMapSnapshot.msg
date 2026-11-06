# Read-only paired map product. Never use invalid or stale pairs for planning.
std_msgs/Header header
bool valid
string invalid_reason
uint64 pair_revision
fastlio_bridge/LoopMapSnapshot current
fastlio_bridge/DufoMapSnapshot background
geometry_msgs/TransformStamped current_from_background
sensor_msgs/PointCloud2 current_only
sensor_msgs/PointCloud2 background_only
std_msgs/String status
