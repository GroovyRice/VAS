import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32

class TFmini(Node):
    def __init__(self, rate=115200, dev_path="/dev/ttyAMA0"):
        super().__init__('tfmini_node')
        self.ser = serial.Serial(dev_path, rate, timeout=1)
        self.get_logger().info(f'Serial Connected, reading from {dev_path} at a Baud Rate of {rate}')
        self.timer = self.create_timer(0.1, self.read)
        self.publisher_ = self.create_publisher(Float32, 'tfmini_distance', 10)
        self.log_count = 0

    def read(self):
        count = self.ser.in_waiting
        if count > 8:
            recv = self.ser.read(9)
            self.ser.reset_input_buffer()
            if recv[0] == 0x59 and recv[1] == 0x59:
                distance = recv[2] + recv[3] * 256
                strength = recv[4] + recv[5] * 256
                temperature = (recv[6] + recv[7] * 256) / 100
                if distance >= 1200:
                    self.get_logger().info('Upper bound reached.')
                    return
                if strength < 20 or strength > 20000:
                    self.get_logger().info(f'Unreliable strength: {strength}')
                    return
                self.log_count += 1
                if self.log_count > 100:
                    self.get_logger().info(f'Distance: {distance}, Strength: {strength}, Temperature: {temperature}')
                    self.log_count = 0

                # Publish the distance to the topic
                msg = Float32()
                msg.data = float(distance)
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFmini()
    rclpy.spin(node)
    node.destroy_node()
    rcply.shutdown()

if __name__ == '__main__':
    main()
