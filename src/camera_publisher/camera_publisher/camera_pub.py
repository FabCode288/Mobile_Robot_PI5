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

# Shared Memory Bild-Parameter
WIDTH, HEIGHT = 640, 480
SHAPE = (HEIGHT, WIDTH, 3)
DTYPE = np.uint8

# Shared Memory Personen-Parameter
MAX_TARGETS = 10
MAX_POINTS_PER_TARGET = 15
MAX_BYTES = 4 + MAX_TARGETS * (4 + MAX_POINTS_PER_TARGET * 12)

class CameraPublisher(Node):
    """
    Publisher für erkannte Personen + Kamerabild aus Shared Memory.
    """

    def __init__(self):
        super().__init__('camera_person_publisher')
        self.person_pub = self.create_publisher(Persons, '/camera_persons', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.03, self.publish_all)

        self.shm_persons = None
        self.shm_image = None
        self.shared_image = None
        self.num_targets = 0

        self.get_logger().info("CameraPublisher ready")

    def connect_shm_persons(self):
        """Versuche, Shared Memory für Personen zu verbinden."""
        if self.shm_persons is None:
            try:
                self.shm_persons = shared_memory.SharedMemory(name="predictions_shm")
                self.get_logger().info("Connected to predictions_shm")
            except FileNotFoundError:
                pass

    def connect_shm_image(self):
        """Versuche, Shared Memory für Bilder zu verbinden."""
        if self.shm_image is None:
            try:
                self.shm_image = shared_memory.SharedMemory(name="image_shm")
                self.shared_image = np.ndarray(SHAPE, dtype=DTYPE, buffer=self.shm_image.buf)
                self.get_logger().info("Connected to image_shm")
            except FileNotFoundError:
                pass

    def read_predictions(self):
        if self.shm_persons is None:
            self.connect_shm_persons()
            if self.shm_persons is None:
                return []
        buf = self.shm_persons.buf
        offset = 0
        (num_targets,) = struct.unpack_from("i", buf, offset)
        self.num_targets = num_targets
        #print("Num targets:", num_targets)
        offset += 4
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
        return results 

    def publish_persons(self):
        preds = self.read_predictions()
        msg = Persons()
        for person_id, points in preds:
            marker = PersonMarker()
            marker.id = person_id  # ID direkt aus Shared Memory
            for p in points[:12]:  # maximal 12 Punkte
                point_msg = Point()
                point_msg.x, point_msg.y, point_msg.z = p
                marker.positions.append(point_msg)
            msg.people.append(marker)
        if self.num_targets != 0:    
            self.person_pub.publish(msg)
            #self.get_logger().info(f"Published {len(msg.people)} people")

    def publish_image(self):
        if self.shared_image is None:
            self.connect_shm_image()
            if self.shared_image is None:
                return
        img_copy = self.shared_image.copy()
        ros_img = self.bridge.cv2_to_imgmsg(img_copy, encoding='rgb8')
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "camera_link"
        self.image_pub.publish(ros_img)
        #self.get_logger().info("Published image")

    def publish_all(self):
        self.publish_persons()
        #self.publish_image()


def main(args=None):
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
