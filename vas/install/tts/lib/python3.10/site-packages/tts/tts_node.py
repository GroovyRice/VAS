import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import pyttsx3

class TTSSpeaker(Node):
    def __init__(self):
        super().__init__('tts_speaker')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            String,
            '/tts',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.tts_engine = pyttsx3.init()

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.tts_engine.say(msg.data)
        self.tts_engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    tts_speaker = TTSSpeaker()
    rclpy.spin(tts_speaker)
    tts_speaker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
