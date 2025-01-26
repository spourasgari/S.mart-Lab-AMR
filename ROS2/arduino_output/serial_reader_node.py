import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        self.publisher_ = self.create_publisher(String, 'motor_encoder_data', 10)

        self.subscription_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.serial_port = '/dev/ttyACM0'  # Adjust this if necessary (e.g., '/dev/ttyACM1')
        self.baud_rate = 9600
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate)
        self.timer = self.create_timer(0.1, self.read_serial_data)  # Timer to read serial data every 100ms

    def read_serial_data(self):
        if self.serial_connection.in_waiting > 0:
            # Read line from the serial buffer
            data = self.serial_connection.readline().decode('utf-8').strip()
            if data:
                self.get_logger().info(f'Received data: {data}')
                # Create a message and publish the data
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)


    def cmd_vel_callback(self, msg):
        # Convert Twist message to a string format that Arduino can parse
        cmd_string = f"{msg.linear.x} {msg.linear.y} {msg.angular.z}\n"

        # Send the command to Arduino
        self.serial_connection.write(cmd_string.encode('utf-8'))

        self.get_logger().info(f'Sent cmd_vel: {cmd_string.strip()}')



def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup on shutdown
    node.serial_connection.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
