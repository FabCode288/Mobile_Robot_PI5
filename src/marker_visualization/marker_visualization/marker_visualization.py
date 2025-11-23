#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

from camera_publisher.msg import Persons, PersonMarker

import tf2_ros
import tf2_geometry_msgs

class CameraPointHandler(Node):
    def __init__(self):
        super().__init__('camera_point_handler')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_sub = self.create_subscription(Persons, '/camera_persons', self.persons_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/camera_point_marker', 10)

        self.source_frame = 'front_camera_link'
        self.target_frame = 'odom'

        self.get_logger().info('CameraPointHandler ready (listening for camera points)')

    # ------------------------------------------------------
    # Utility: Jede Person erhält konsistente Farbe
    # ------------------------------------------------------
    def get_color_for_person(self, person_id):
        # einfache deterministische Farbzuweisung
        r = ((person_id * 97) % 255) / 255.0
        g = ((person_id * 53) % 255) / 255.0
        b = ((person_id * 199) % 255) / 255.0
        return r, g, b

    # ------------------------------------------------------
    # Haupt-Callback: Wir empfangen alle Personen
    # ------------------------------------------------------
    def persons_callback(self, msg: Persons):
        for person in msg.people:

            if len(person.positions) < 2:
                continue  # müssen min. 2 Punkte haben

            person_id = person.id

            first_pos = person.positions[0]
            last_pos = person.positions[-1]

            # IDs eindeutig
            marker_id_first = person_id * 10 + 1
            marker_id_last  = person_id * 10 + 2
            marker_id_arrow = person_id * 10 + 3

            # publish points
            self.publish_person_point(first_pos, marker_id_first, person_id)
            self.publish_person_point(last_pos,  marker_id_last,  person_id)

            # publish arrow
            self.publish_arrow(first_pos, last_pos, marker_id_arrow, person_id)

    # ------------------------------------------------------
    # Punktmarker publizieren
    # ------------------------------------------------------
    def publish_person_point(self, p: Point, marker_id, person_id):
        point_stamped = PointStamped()
        point_stamped.header.frame_id = self.source_frame
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.point = p

        self._transform_and_publish_sphere(point_stamped, marker_id, person_id)

    # ------------------------------------------------------
    # Transform + Publish Sphere
    # ------------------------------------------------------
    def _transform_and_publish_sphere(self, point: PointStamped, marker_id, person_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point.header.frame_id,
                rclpy.time.Time()
            )
            tp = tf2_geometry_msgs.do_transform_point(point, transform)

            r, g, b = self.get_color_for_person(person_id)

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"person_{person_id}"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = tp.point.x
            marker.pose.position.y = tp.point.y
            marker.pose.position.z = tp.point.z
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12

            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'Could not transform sphere point: {e}')

    # ------------------------------------------------------
    # ARROW Marker erzeugen
    # ------------------------------------------------------
    def publish_arrow(self, first: Point, last: Point, marker_id, person_id):
        try:
            # transform first
            p1 = PointStamped()
            p1.header.frame_id = self.source_frame
            p1.header.stamp = self.get_clock().now().to_msg()
            p1.point = first
            t1 = tf2_geometry_msgs.do_transform_point(p1,
                    self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time()))

            # transform last
            p2 = PointStamped()
            p2.header.frame_id = self.source_frame
            p2.header.stamp = self.get_clock().now().to_msg()
            p2.point = last
            t2 = tf2_geometry_msgs.do_transform_point(p2,
                    self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time()))

            r, g, b = self.get_color_for_person(person_id)

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"person_arrow_{person_id}"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.scale.x = 0.05  # shaft diameter
            marker.scale.y = 0.1   # head diameter
            marker.scale.z = 0.1   # head length

            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b

            # ARROW uses points[]
            marker.points = [
                Point(x=t1.point.x, y=t1.point.y, z=t1.point.z),
                Point(x=t2.point.x, y=t2.point.y, z=t2.point.z)
            ]

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f'Could not publish arrow: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPointHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
