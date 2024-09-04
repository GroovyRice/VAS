import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import subprocess
import os
import tempfile

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

        # Define Piper model path
        self.piper_model_path = "/opt/piper/voices/aru/medium/en_GB-aru-medium.onnx"

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.speak(msg.data)

    def speak(self, text):
        # Create a temporary file to store the audio
        temp_wav_path = "/opt/temp.wav"  # Adjust this to your desired path if needed

        # Construct the full shell command as a single string
        command = (
            f'echo "{text}" | '
            f'/opt/piper/build/piper --model /opt/piper/voices/aru/medium/en_GB-aru-medium.onnx --output_file {temp_wav_path}'
        )

        # Run the Piper command using subprocess.run() with shell=True to mimic shell behavior
        process = subprocess.run(command, shell=True, capture_output=True)

        # Check if there are any errors
        if process.returncode != 0:
            self.get_logger().error(f"Piper Error: {process.stderr.decode('utf-8')}")
            return

        # Play the generated WAV file using aplay
        os.system(f"aplay {temp_wav_path}")

        # Log success
        self.get_logger().info(f"Piper completed successfully, audio saved to {temp_wav_path}")


def main(args=None):
    rclpy.init(args=args)
    tts_speaker = TTSSpeaker()
    rclpy.spin(tts_speaker)
    tts_speaker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
