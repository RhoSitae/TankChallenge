# Unity → ROS  좌표 매핑: x = X, y = Z, z = Y  (x-z-y)
# 축 순서가 x-z-y 이므로 z 관련 min/max 는 Z(앞뒤), y 관련은 Y(높이)
float64 x_min
float64 x_max
float64 y_min        # Unity Z  (ROS y)
float64 y_max
float64 z_min        # Unity Y  (ROS z)
float64 z_max