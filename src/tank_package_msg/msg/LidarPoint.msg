# Unity → ROS  좌표 매핑: x = X, y = Z, z = Y  (x-z-y)
float32 angle
float32 vertical_angle
float32 distance
geometry_msgs/Point position          # point.x=X, point.y=Z, point.z=Y
bool     is_detected
int32    channel_index
