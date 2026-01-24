import rclpy
from rclpy.node import Node

from robot_msgs.msg import Persons, PersonMarker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import struct
from multiprocessing import shared_memory
import numpy as np
import time

# ============================================================
# Shared memory image parameters
# ============================================================

WIDTH, HEIGHT = 640, 480
SHAPE = (HEIGHT, WIDTH, 3)
DTYPE = np.uint8

# ============================================================
# Shared memory person parameters (with timestamp)
# ============================================================

MAX_TARGETS = 10
MAX_POINTS_PER_TARGET = 15

MAX_BYTES = (
    4 +   # int32 num_targets
    8 +   # uint64 timestamp_ns
    MAX_TARGETS * (4 + 4 + MAX_POINTS_PER_TARGET * 12)
)

# ============================================================
# ROS Node
# ============================================================

class CameraPublisher(Node):
    """
    ROS 2 node for publishing camera-based person detections from shared memory.

    This node reads prediction data written by an external process into shared
    memory and publishes it as ROS messages. Each detected person is represented
    by a unique ID and a sequence of 3D points.

    Additionally, the node can publish RGB images from shared memory as
    `sensor_msgs/Image` messages.

    Shared memory layout for person predictions:

        - int32  num_targets
        - uint64 timestamp_ns
        - for each target:
            - int32  person_id
            - int32  num_points
            - float32 x, y, z (per point)
    """

    def __init__(self):
        """
        Initialize the CameraPublisher node.

        Sets up ROS publishers, shared memory handles, a CV bridge for image
        conversion, and a periodic timer for publishing data.
        """
        super().__init__('camera_person_publisher')

        self.person_pub = self.create_publisher(
            Persons, '/camera_persons', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', 10
        )

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.03, self.publish_all)

        self.shm_persons = None
        self.shm_image = None
        self.shared_image = None

        self.num_targets = 0

        self.get_logger().info("CameraPublisher ready")

    def connect_shm_persons(self):
        """
        Connect to the shared memory segment containing person predictions.

        If the shared memory segment is not available yet, the connection attempt
        fails silently and will be retried later.
        """
        if self.shm_persons is None:
            try:
                self.shm_persons = shared_memory.SharedMemory(
                    name="predictions_shm"
                )
                self.get_logger().info(
                    "Connected to predictions_shm"
                )
            except FileNotFoundError:
                pass

    def connect_shm_image(self):
        """
        Connect to the shared memory segment containing the camera image.

        The shared memory buffer is interpreted as a NumPy array with a fixed
        shape and data type.
        """
        if self.shm_image is None:
            try:
                self.shm_image = shared_memory.SharedMemory(
                    name="image_shm"
                )
                self.shared_image = np.ndarray(
                    SHAPE, dtype=DTYPE, buffer=self.shm_image.buf
                )
                self.get_logger().info(
                    "Connected to image_shm"
                )
            except FileNotFoundError:
                pass

    def read_predictions(self):
        """
        Read person predictions from shared memory.

        The method parses the shared memory buffer according to the predefined
        binary layout and extracts the timestamp, person IDs, and associated
        3D points.

        Returns:
            tuple:
                - int | None: Timestamp in nanoseconds, or None if no data
                  is available
                - list: List of tuples (person_id, list of (x, y, z) points)
        """
        if self.shm_persons is None:
            self.connect_shm_persons()
            if self.shm_persons is None:
                return None, []

        buf = self.shm_persons.buf
        offset = 0

        (num_targets,) = struct.unpack_from("i", buf, offset)
        offset += 4

        self.num_targets = num_targets

        (timestamp_ns,) = struct.unpack_from("Q", buf, offset)
        offset += 8

        results = []

        for _ in range(num_targets):
            (person_id,) = struct.unpack_from("i", buf, offset)
            offset += 4

            (num_points,) = struct.unpack_from("i", buf, offset)
            offset += 4

            points = []
            for _ in range(num_points):
                x, y, z = struct.unpack_from("fff", buf, offset)
                offset += 12
                points.append((x, y, z))

            results.append((person_id, points))

        return timestamp_ns, results

    def publish_persons(self):
        """
        Publish detected persons as a `robot_msgs/Persons` message.

        The ROS message timestamp is derived from the shared memory timestamp
        to preserve temporal consistency with the source data.
        """
        timestamp_ns, preds = self.read_predictions()
        if timestamp_ns is None or self.num_targets == 0:
            return

        msg = Persons()

        msg.header.stamp.sec = int(
            timestamp_ns // 1_000_000_000
        )
        msg.header.stamp.nanosec = int(
            timestamp_ns % 1_000_000_000
        )
        msg.header.frame_id = "camera_link"

        for person_id, points in preds:
            marker = PersonMarker()
            marker.id = person_id

            for p in points[:12]:
                point_msg = Point()
                point_msg.x, point_msg.y, point_msg.z = p
                marker.positions.append(point_msg)

            msg.people.append(marker)

        self.person_pub.publish(msg)
        self.get_logger().info(
            f"Published {len(msg.people)} people"
        )

    def publish_image(self):
        """
        Publish the camera image from shared memory as a ROS image message.

        The image is copied from shared memory, converted to a ROS-compatible
        format, and published with the current ROS time.
        """
        if self.shared_image is None:
            self.connect_shm_image()
            if self.shared_image is None:
                return

        img_copy = self.shared_image.copy()
        ros_img = self.bridge.cv2_to_imgmsg(
            img_copy, encoding='rgb8'
        )

        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_link"

        self.image_pub.publish(ros_img)

    def publish_all(self):
        """
        Periodic timer callback.

        Publishes person detections and, optionally, the camera image.
        """
        self.publish_persons()
        # self.publish_image()  # optional


def main(args=None):
    """
    Entry point for the CameraPublisher node.

    Initializes rclpy, spins the node, and ensures a clean shutdown.
    """
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
