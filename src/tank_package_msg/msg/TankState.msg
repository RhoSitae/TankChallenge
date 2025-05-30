# Unity → ROS  좌표 매핑: x = X, y = Z, z = Y  (x-z-y)
builtin_interfaces/Time stamp
geometry_msgs/Point  position        # playerPos
geometry_msgs/Vector3 body_euler     # bodyX,Z,Y  (roll,pitch,yaw 순서 보존)
geometry_msgs/Vector3 turret_euler   # turretX,Z,Y
float64              speed           # playerSpeed
float32              health          # playerHealth
float32				 distance