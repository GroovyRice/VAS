import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from depthai_ros_msgs.msg import TrackDetection2DArray
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import cv2
import math


# Define the label map and filter map
LABEL_MAP = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

FILTER_MAP = [
    "person",
    "bicycle",
    "car",
    "motorbike",
    "bus",
    "truck"
]

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.get_logger().info('Initialising Subscription...')
        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.subscription_image = self.create_subscription(
            Image,
            '/color/image',
            self.listener_image_callback,
            qos_profile)
        self.subscription_image 
        self.get_logger().info('> /color/image created.')

        self.subscription_detection = self.create_subscription(
            TrackDetection2DArray,
            '/color/yolov4_Spatial_tracklets', 
            self.listener_detection_callback,
            qos_profile)
        self.subscription_detection 
        self.get_logger().info('> /color/yolov4_Spatial_tracklets created.')

        self.subscription_tfmini = self.create_subscription(
            Float32,  
            '/tfmini_distance',
            self.listener_tfmini_callback,
            qos_profile)
        self.subscription_tfmini 
        self.get_logger().info('> /tfmini_distance created.')

        self.tts_publisher = self.create_publisher(
            String, 
            '/tts', 
            qos_profile)

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = None
        self.latest_distance = None
        self.error_count = 0

    def listener_tfmini_callback(self, msg):
        self.latest_distance = msg.data
        self.process_data()

    def listener_image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_data()
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {0}'.format(e))

    def listener_detection_callback(self, msg):
        self.latest_detections = msg
        self.process_data()

    def process_data(self):
        if self.latest_image is None:
            self.error_count += 1
            if self.error_count > 100:
                self.get_logger().info('> /color/image is NONE')
                self.error_count = 0
            return

        if self.latest_detections is None:
            self.error_count += 1
            if self.error_count > 100:
                self.get_logger().info('> /color/yolov4_Spatial_tracklets is NONE')
                self.error_count = 0
            return
        
        if self.latest_distance is None:
            self.error_count += 1
            if self.error_count > 100:
                self.get_logger().info('> /tfmini_distance is NONE')
                self.error_count = 0
            return
        
        objects = []
        objects.append([self.latest_distance / 100, "Object", "Forward", 0])
        for detection in self.latest_detections.detections:
            label = LABEL_MAP[int(detection.results[0].hypothesis.class_id)]
            id = detection.tracking_id
            age = detection.tracking_age

            if label not in FILTER_MAP and age > 30:
                continue
            
            x = detection.results[0].pose.pose.position.x         # X coordinate of the object's pose
            y = detection.results[0].pose.pose.position.y         # Y coordinate of the object's pose
            z = detection.results[0].pose.pose.position.z         # Z coordinate of the object's pose
            if z == 0:
                continue

            angle = math.degrees(math.atan(abs(x)/z))

            if angle < 2.5:
                direction = "Forward"
            elif x > 0:
                direction = "Right"
            else:
                direction = "Left"

            self.get_logger().info(f'{label}: {x:.1f}, {y:.1f}, {z:.1f}, {angle:.0f} Degrees, {direction}, {id}, {age}')

            objects.append([z, label, direction, int(angle)])

            bbox = detection.bbox
            center = bbox.center.position
            size_x = bbox.size_x
            size_y = bbox.size_y

            top_left = (int(center.x - size_x / 2), int(center.y - size_y / 2))
            bottom_right = (int(center.x + size_x / 2), int(center.y + size_y / 2))

            cv2.rectangle(self.latest_image, top_left, bottom_right, (0, 255, 0), 2)

        tf_text = f"TF: {self.latest_distance:.2f}"
        cv2.putText(self.latest_image, tf_text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow("Camera Image", self.latest_image)
        cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

        self.determinations(objects)

        self.latest_image = None
        self.latest_detections = None
        self.latest_distance = None

    def determinations(self, objects):
        msg = ""
        small = min(objects, key=lambda x: x[0])
        if small[0] < 3:
            msg += f"{small[1]}, "
            if small[0] < 1:
                msg += f"{int(small[0]*100)} centimetres ahead "
            else:
                msg += f"{int(small[0])} metres ahead "
            if small[3] != 0:
                msg += f"to the {small[2]}, {small[3]} Degrees"
        else:
            msg += f"{len(objects)} Objects Ahead:"
            for obj in objects:
                if obj[0] < 1:
                    msg += f"{int(obj[0]*100)} centimetres,"
                else:
                    msg += f"{int(obj[0])} metres,"
                msg += f"{obj[1]}"
                if obj[3] == 0:
                    msg += f"{obj[2]}"
                else:
                    msg += f"{obj[3]} Degrees {obj[2]}. "

        # IF (OBJECT within 3m)
        # > TTS CLOSEST OBJECT
        # ELSE IF (OBJECT exists)
        # > TTS CLOSEST OBJECT
        # ELSE
        # > TTS ALL OBJECTS
        if msg != "":
            self.publish(msg)
    
    def publish(self, message):
        msg = String()
        msg.data = message
        self.tts_publisher.publish(msg)
        self.get_logger().info(f'Published TTS message: {message}')


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
