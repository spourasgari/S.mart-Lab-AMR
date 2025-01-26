# Intel RealSense Camera and IMU

I launched the IMU data with the following command:

```ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true enable_depth:=false enable_color:=false enable_infra1:=false enable_infra2:=false unite_imu_method:=2 pointcloud.enable:=true```

But maybe using _imu_filter_madgwick_ would be better. Try it.

It is needed to publish the transformation between the camera frame and other parts, so:
```ros2 run tf2_ros static_transform_publisher 0.25 0 0.5 0 -0.2618 0 laser camera_link```

Using my command, the IMU data is published on the frame `camera_imu_optical_frame`, so there is a need to have a transformation from that to the camera_link as well:

```ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link camera_imu_optical_frame```
