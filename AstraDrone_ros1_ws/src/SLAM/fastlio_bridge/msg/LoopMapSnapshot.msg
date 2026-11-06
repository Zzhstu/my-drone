# Atomic global planning contract. No consumer may mix separate revisions.
std_msgs/Header header
uint64 revision
uint64 graph_revision
uint64 requested_graph_revision
uint64 input_frames
uint64 processed_frames
uint64 input_bytes
bool rebuilding
bool capacity_limited
bool frozen
float64 processing_ms
float64 rebuild_wall_seconds
sensor_msgs/PointCloud2 map
sensor_msgs/PointCloud2 free
sensor_msgs/PointCloud2 suspects
std_msgs/String ghost_status
std_msgs/String ghost_history
fastlio_bridge/LoopGraph graph
# Transform loop_map <- the original continuous local map at snapshot time.
geometry_msgs/TransformStamped global_from_local
