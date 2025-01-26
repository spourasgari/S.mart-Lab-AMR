# Arduino Output Package

There are two nodes here: `encoder_to_odom` and `serial_reader_node`.

* The `serial_reader_node` is required for any communication from ROS2 to Arduino.

* The `encoder_to_odom` node is required for wheel odometry, while it also encompasses multiple required transformations between frames: base_footprint -> base_link (static), base_link -> laser (static), odom -> base_footprint (dynamic)

    * _To adjust the dimensions and position of elements on the robot, modify the transform values in this node_
    * If you are using sensor fusion, DON'T let this transform (```self.tf_broadcaster.sendTransform(t_odom_base)```) to be published! The fusion package will do that.

The nodes can be started individually (to see the outputs separately), or using the launch file to run them in one attempt terminal:
```ros2 launch arduino_output arduino_connection_and_odom.py```
