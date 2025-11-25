#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from multiprocessing import shared_memory
import struct

MAX_TARGETS = 4
MAX_POINTS_PER_TARGET = 12

class ShmPredictionWriterNode(Node):
    """
    ROS2 Node, der feste Test-Personendaten in Shared Memory schreibt.
    Jede Person erhält 12 Punkte mit unterschiedlichen Koordinaten.
    """

    def __init__(self):
        super().__init__('shm_prediction_writer_fixed')
        self.timer = self.create_timer(1.0, self.write_predictions)  # 1 Hz

        # Shared Memory erstellen oder anhängen
        self.max_bytes = 4  # num_targets
        self.max_bytes += MAX_TARGETS * (4 + 4 + MAX_POINTS_PER_TARGET * 3 * 4)  # id + num_points + Punkte
        try:
            self.shm = shared_memory.SharedMemory(name="predictions_shm", create=True, size=self.max_bytes)
            self.get_logger().info("Created shared memory for predictions")
        except FileExistsError:
            self.shm = shared_memory.SharedMemory(name="predictions_shm", create=False)
            self.get_logger().info("Attached to existing shared memory")

        # Feste IDs und feste Punkte pro Person
        self.person_data = [
            {"id": 1, "points": [(0.1*i, 0.3, 0.5 + 0.1*i) for i in range(12)]},
            {"id": 2, "points": [(0.0*i, 0.3, 0.6 + 0.1*i) for i in range(12)]},
            {"id": 3, "points": [(-0.2*i, 0.3, 0.7 + 0.1*i) for i in range(12)]},
            {"id": 4, "points": [(0.25*i, 0.3, 0.8 + 0.1*i) for i in range(12)]}
        ]

    def write_predictions(self):
        buf = self.shm.buf
        offset = 0

        # Anzahl Targets
        struct.pack_into("i", buf, offset, MAX_TARGETS)
        offset += 4

        for person in self.person_data:
            # ID
            struct.pack_into("i", buf, offset, person["id"])
            offset += 4

            # Anzahl Punkte
            n_points = len(person["points"])
            struct.pack_into("i", buf, offset, n_points)
            offset += 4

            # Punkte (x, y, z)
            for x, y, z in person["points"]:
                struct.pack_into("fff", buf, offset, x, y, z)
                offset += 12

        #self.get_logger().info(f"Wrote {MAX_TARGETS} fixed targets to shared memory")


def main(args=None):
    rclpy.init(args=args)
    node = ShmPredictionWriterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    node.shm.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
