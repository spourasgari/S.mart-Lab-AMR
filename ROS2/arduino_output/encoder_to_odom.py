import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped, Point, Quaternion, Vector3
import serial
import math
import tf2_ros
import tf_transformations
from time import time

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.encoder_subscriber = self.create_subscription(String, 'motor_encoder_data', self.encoder_callback, 10)
        
        # Initialize serial connection (same as previous)
        self.serial_port = '/dev/ttyACM0'  # Adjust this if necessary
        self.baud_rate = 9600
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate)
        
        # Initial odometry values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Robot's orientation

        # Velocity placeholders
        self.vx = 0.0  # Example linear x velocity
        self.vy = -0.0  # Example linear y velocity
        self.vth = 0.0  # Example angular velocity

        # Robot geometry (adjust according to your robot)
        self.wheel_diameter = 0.1  # meters
        self.encoder_ticks_per_revolution = 324  # encoder ticks per full wheel rotation
        self.wheel_base = 0.5525  # meters (distance between wheels)

        # Initialize the TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timing for velocity calculation
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        # Initialize encoder previous values
        self.right_encoder_prev = 0
        self.left_encoder_prev = 0

        self.timer = self.create_timer(0.1, self.publish_odometry)  # Publish odometry every 100ms

        # A timestamp to control logging frequency
        self.last_log_time = 0.0  # Initialize to 0 or the current time
        self.log_interval = 2.0  # Log every 2 seconds

    def encoder_callback(self, msg):
        # Parse the encoder data
        data = msg.data
        if data.startswith("R:") and "L:" in data:
            right_ticks = int(data.split("R:")[1].split(" L:")[0])
            left_ticks = int(data.split("L:")[1])

            # Compute the displacement from the encoder ticks
            delta_right = right_ticks - self.right_encoder_prev
            delta_left = left_ticks - self.left_encoder_prev

            # Update encoder values
            self.right_encoder_prev = right_ticks
            self.left_encoder_prev = left_ticks

            # Compute the distance traveled by each wheel (in meters)
            distance_right = (delta_right / self.encoder_ticks_per_revolution) * math.pi * self.wheel_diameter
            distance_left = (delta_left / self.encoder_ticks_per_revolution) * math.pi * self.wheel_diameter

            # Compute velocities
            self.current_time = self.get_clock().now()
            dt = (self.current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

            # Compute the robot's displacement
            delta_distance = (distance_left + distance_right) / 2
            delta_theta = (distance_right - distance_left) / self.wheel_base

            self.vx = delta_distance / dt
            self.vth = delta_theta / dt

            # Update positions
            delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
            delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
            delta_th = self.vth * dt

            # Update the robot's pose
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_th

            # Update last time
            self.last_time = self.current_time

            # Log the encoder data and calculated values for debugging
            current_time = time()
            if current_time - self.last_log_time >= self.log_interval:
                self.get_logger().info(f"Received encoder data: R={right_ticks}, L={left_ticks}")
                self.get_logger().info(f"Displacement: delta_distance={delta_distance}, delta_theta={delta_theta}")
                self.get_logger().info(f"Current pose: x={self.x}, y={self.y}, theta={math.degrees(self.theta):.2f}")
                self.last_log_time = current_time

    def publish_odometry(self):
        # Publish odometry as before
        odom_msg = Odometry()
        current_time = self.get_clock().now()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'

        # Create quaternion from yaw (theta)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # # Pose
        # odom_msg.pose.pose = Pose()
        # odom_msg.pose.pose.position.x = self.x
        # odom_msg.pose.pose.position.y = self.y
        # odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        # odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        # # Velocity (for now, zero)
        # odom_msg.twist.twist = Twist()

        # Set pose
        odom_msg.pose.pose = Pose(
            position=Point(x=float(self.x), y=float(self.y), z=0.0),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )

        # Set twist
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = Twist(
            linear=Vector3(x=float(self.vx), y=float(self.vy), z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(self.vth))
        )

        # Publish the odometry message
        self.publisher_.publish(odom_msg)

        # Create and publish the transform from odom to base_footprint
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = current_time.to_msg()
        t_odom_base.header.frame_id = 'odom'  # Parent frame
        t_odom_base.child_frame_id = 'base_footprint'  # Child frame

        t_odom_base.transform.translation.x = self.x
        t_odom_base.transform.translation.y = self.y
        t_odom_base.transform.translation.z = 0.0  # Robot is on a flat plane, no vertical translation
        t_odom_base.transform.rotation.x = quat[0]
        t_odom_base.transform.rotation.y = quat[1]
        t_odom_base.transform.rotation.z = quat[2]
        t_odom_base.transform.rotation.w = quat[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t_odom_base) # If you are using sensor fusion, DON'T let this transform to be published! The fusion package will do that.


        # Static transform: base_footprint -> base_link
        t_base_footprint_link = TransformStamped()
        t_base_footprint_link.header.stamp = self.get_clock().now().to_msg()
        t_base_footprint_link.header.frame_id = 'base_footprint'
        t_base_footprint_link.child_frame_id = 'base_link'

        # Static transform: Adjust these values if necessary
        t_base_footprint_link.transform.translation.x = 0.0
        t_base_footprint_link.transform.translation.y = 0.0
        t_base_footprint_link.transform.translation.z = 0.0  # Adjust for height offset
        t_base_footprint_link.transform.rotation.x = 0.0
        t_base_footprint_link.transform.rotation.y = 0.0
        t_base_footprint_link.transform.rotation.z = 0.0
        t_base_footprint_link.transform.rotation.w = 1.0  # Identity quaternion

        self.tf_broadcaster.sendTransform(t_base_footprint_link)

        # Static transform: base_link -> laser
        t_base_link_laser = TransformStamped()
        t_base_link_laser.header.stamp = self.get_clock().now().to_msg()
        t_base_link_laser.header.frame_id = 'base_link'
        t_base_link_laser.child_frame_id = 'laser'

        # Static transform: Adjust these values for the actual laser position on your robot
        t_base_link_laser.transform.translation.x = 0.0  # Example offset in meters
        t_base_link_laser.transform.translation.y = 0.0
        t_base_link_laser.transform.translation.z = 0.0  # Height of the laser sensor
        t_base_link_laser.transform.rotation.x = 0.0
        t_base_link_laser.transform.rotation.y = 0.0
        t_base_link_laser.transform.rotation.z = 0.0
        t_base_link_laser.transform.rotation.w = 1.0  # Identity quaternion

        self.tf_broadcaster.sendTransform(t_base_link_laser)

        # # Log the published odometry
        # self.get_logger().info(f"Publishing odometry: x={self.x}, y={self.y}, theta={self.theta}")


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.serial_connection.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
