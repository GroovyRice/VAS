import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import subprocess
import os
import tempfile
import threading
import time
import queue

class TTSSpeaker(Node):
    def __init__(self):
        super().__init__('tts_speaker')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            String,
            '/tts',
            self.listener_callback,
            qos_profile)
        
        # Piper model path
        self.piper_model_path = "/opt/piper/voices/alan/low/en_GB-alan-low.onnx"
        
        # Queue with a maximum size of 1
        self.audio_queue = queue.Queue(maxsize=1)

        # Latest message
        self.latest_message = None

        # Thread for audio generation
        self.audio_generation_thread = threading.Thread(target=self.audio_worker)
        self.audio_generation_thread.start()

        # Thread for audio playback
        self.audio_playback_thread = threading.Thread(target=self.playback_worker)
        self.audio_playback_thread.start()

    def listener_callback(self, msg):
        # Update the latest message when a new one arrives
        self.latest_message = msg.data

    def audio_worker(self):
        """Generates the WAV file and adds it to the queue."""
        while rclpy.ok():
            # Only generate audio if the queue is empty
            if self.latest_message and self.audio_queue.empty():
                wav_file_path = self.generate_wav(self.latest_message)

                if wav_file_path:
                    # Wait if the queue is full (maxsize=1), ensuring only one audio file in the queue
                    self.audio_queue.put(wav_file_path)

                # Reset the message to avoid repeated generation
                self.latest_message = None

    def playback_worker(self):
        """Plays audio files from the queue."""
        while rclpy.ok():
            # Wait for an audio file to be available in the queue
            wav_file_path = self.audio_queue.get()

            if wav_file_path:
                # Play the audio file
                self.play_audio(wav_file_path)

                # After playback, remove the file
                os.remove(wav_file_path)

            # Indicate task completion to the queue
            self.audio_queue.task_done()

    def generate_wav(self, text):
        if text is None:
            return None

        # Create a temporary file to store the output of Piper
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav_file:
            wav_file_path = temp_wav_file.name

        # Construct the shell command to generate the wav file using Piper
        command = (
            f'echo "{text}" | '
            f'/opt/piper/build/piper --model {self.piper_model_path} --output_file {wav_file_path}'
        )

        # Run the Piper command
        process = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Check if there are any errors
        if process.returncode != 0:
            self.get_logger().error(f"Piper Error: {process.stderr.decode('utf-8')}")
            return None

        # Speed up the audio by 1.2x using SoX
        sped_up_wav_path = self.speed_up_audio(wav_file_path)

        # Remove the original wav file
        os.remove(wav_file_path)

        return sped_up_wav_path

    def speed_up_audio(self, input_wav_path):
        # Create a temporary file for the sped-up audio
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav_file:
            sped_up_wav_file_path = temp_wav_file.name

        # Use SoX to speed up the audio by 1.2x
        sox_command = f"sox {input_wav_path} {sped_up_wav_file_path} speed 1.2"
        process = subprocess.run(sox_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        if process.returncode != 0:
            self.get_logger().error(f"SoX Error: {process.stderr.decode('utf-8')}")
            return None

        return sped_up_wav_file_path

    def play_audio(self, wav_file_path):
        if wav_file_path is None:
            return

        # Play the sped-up audio using aplay
        start_time = time.time()
        os.system(f"aplay {wav_file_path}")
        end_time = time.time()
        self.get_logger().info(f"Time to play audio with aplay: {end_time - start_time:.4f} seconds")

    def destroy_node(self):
        super().destroy_node()
        # Ensure threads exit cleanly
        if self.audio_generation_thread.is_alive():
            self.audio_generation_thread.join()
        if self.audio_playback_thread.is_alive():
            self.audio_playback_thread.join()

def main(args=None):
    rclpy.init(args=args)
    tts_speaker = TTSSpeaker()
    rclpy.spin(tts_speaker)
    tts_speaker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
