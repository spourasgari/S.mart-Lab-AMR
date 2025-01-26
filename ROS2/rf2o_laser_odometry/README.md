# Laser Scan Matcher (rf2o_laser_odometry)

[Link to the original repository](https://github.com/MAPIRlab/rf2o_laser_odometry)

The latest launch file is in this directory.

Be careful that some Cpp packages, particularly in this package, look for `tf2_geometry_msgs.hpp`, while its extension is `.h` in the installed directory. For example in the file `/home/AMR/ros2_ws/src/rf2o_laser_odometry/include/rf2o_laser_odometry/CLaserOdometry2DNode.hpp`, the following line is correct instead of the original one:

```#include <tf2_geometry_msgs/tf2_geometry_msgs.h>```

Note the name of the odom's topic published by this package, and don't confuse it with the wheel odometry topic (/odom in this project)

Keep the `publish_tf` on `False` if you are publishing tf somewhere else, for example using the robot_localization package.

If you are only using this package to publish the odometery (the wheel odometry is disconnected), don't forget to publish the transformation like this:
```ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser```