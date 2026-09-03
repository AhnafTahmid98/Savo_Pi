"""Read-only planar localization markers for the external observer."""

import math

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from savo_observer.localization_math import (
    covariance_ellipse,
    ellipse_points,
    yaw_arc_points,
    yaw_uncertainty_half_span,
)


FILTERED_ODOM_TOPIC = '/odometry/filtered'
WHEEL_ODOM_TOPIC = '/wheel/odom'
VISUAL_ODOM_TOPIC = '/vo/odom'
LOCALIZATION_MARKERS_TOPIC = '/savo_observer/localization_markers'


def _yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw without carrying roll or pitch into an RViz arrow."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _set_color(marker, red, green, blue, alpha=1.0):
    marker.color.r = float(red)
    marker.color.g = float(green)
    marker.color.b = float(blue)
    marker.color.a = float(alpha)


def _point(x, y, z):
    value = Point()
    value.x = float(x)
    value.y = float(y)
    value.z = float(z)
    return value


class LocalizationVisualizer(Node):
    """Render current planar odometry only; never republish or mutate it."""

    def __init__(self):
        super().__init__('localization_visualizer_node')
        self.declare_parameter('filtered_odom_topic', FILTERED_ODOM_TOPIC)
        self.declare_parameter('wheel_odom_topic', WHEEL_ODOM_TOPIC)
        self.declare_parameter('visual_odom_topic', VISUAL_ODOM_TOPIC)
        self.declare_parameter('markers_topic', LOCALIZATION_MARKERS_TOPIC)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('confidence_sigma', 2.0)
        self.declare_parameter('max_ellipse_radius_m', 1.5)
        self.declare_parameter('max_yaw_half_span_rad', math.pi)

        self._confidence_sigma = self._positive_parameter(
            'confidence_sigma'
        )
        self._max_ellipse_radius_m = self._positive_parameter(
            'max_ellipse_radius_m'
        )
        self._max_yaw_half_span_rad = self._positive_parameter(
            'max_yaw_half_span_rad'
        )
        publish_rate_hz = self._positive_parameter('publish_rate_hz')
        self._latest = {'filtered': None, 'wheel': None, 'vo': None}

        self._publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('markers_topic').value,
            10,
        )
        self._subscriptions = [
            self.create_subscription(
                Odometry,
                self.get_parameter('filtered_odom_topic').value,
                lambda message: self._remember('filtered', message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                self.get_parameter('wheel_odom_topic').value,
                lambda message: self._remember('wheel', message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                self.get_parameter('visual_odom_topic').value,
                lambda message: self._remember('vo', message),
                qos_profile_sensor_data,
            ),
        ]
        self._timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_markers,
        )

    def _positive_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and greater than zero')
        return value

    def _remember(self, source, message):
        self._latest[source] = message

    def _base_marker(self, message, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header = message.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _delete_marker(namespace, marker_id, marker_type):
        marker = Marker()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.DELETE
        return marker

    def _pose_arrow(self, message, namespace, marker_id, color, scale):
        marker = self._base_marker(
            message, namespace, marker_id, Marker.ARROW
        )
        pose = message.pose.pose
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        # This tiny offset is visualization-only and prevents z-fighting.
        marker.pose.position.z = pose.position.z + 0.035
        yaw = _yaw_from_quaternion(pose.orientation)
        marker.pose.orientation.z = math.sin(0.5 * yaw)
        marker.pose.orientation.w = math.cos(0.5 * yaw)
        marker.scale.x = scale
        marker.scale.y = 0.055
        marker.scale.z = 0.035
        _set_color(marker, *color)
        return marker

    def _ellipse_marker(self, message):
        geometry = covariance_ellipse(
            message.pose.covariance,
            self._confidence_sigma,
            self._max_ellipse_radius_m,
        )
        if geometry is None:
            return None
        pose = message.pose.pose
        marker = self._base_marker(
            message, 'filtered_xy_covariance', 1, Marker.LINE_STRIP
        )
        marker.scale.x = 0.025
        _set_color(marker, 0.15, 0.85, 1.0, 0.95)
        marker.points = [
            _point(x, y, pose.position.z + 0.018)
            for x, y in ellipse_points(
                pose.position.x,
                pose.position.y,
                geometry,
            )
        ]
        return marker

    def _yaw_marker(self, message):
        arc = yaw_uncertainty_half_span(
            message.pose.covariance[35],
            self._confidence_sigma,
            self._max_yaw_half_span_rad,
        )
        if arc is None:
            return None
        half_span, _clamped = arc
        pose = message.pose.pose
        heading = _yaw_from_quaternion(pose.orientation)
        marker = self._base_marker(
            message, 'filtered_yaw_uncertainty', 2, Marker.LINE_STRIP
        )
        marker.scale.x = 0.025
        _set_color(marker, 1.0, 0.70, 0.10, 0.95)
        marker.points = [
            _point(x, y, pose.position.z + 0.022)
            for x, y in yaw_arc_points(
                pose.position.x,
                pose.position.y,
                heading,
                half_span,
            )
        ]
        return marker

    def _publish_markers(self):
        markers = [
            self._delete_marker('filtered_pose', 0, Marker.ARROW),
            self._delete_marker(
                'filtered_xy_covariance', 1, Marker.LINE_STRIP
            ),
            self._delete_marker(
                'filtered_yaw_uncertainty', 2, Marker.LINE_STRIP
            ),
            self._delete_marker('wheel_pose', 3, Marker.ARROW),
            self._delete_marker('visual_pose', 4, Marker.ARROW),
        ]
        filtered = self._latest['filtered']
        if filtered is not None and filtered.header.frame_id:
            markers[0] = self._pose_arrow(
                filtered,
                'filtered_pose',
                0,
                (0.10, 0.95, 0.25),
                0.34,
            )
            ellipse = self._ellipse_marker(filtered)
            yaw = self._yaw_marker(filtered)
            if ellipse is not None:
                markers[1] = ellipse
            if yaw is not None:
                markers[2] = yaw

        wheel = self._latest['wheel']
        if wheel is not None and wheel.header.frame_id:
            markers[3] = self._pose_arrow(
                wheel, 'wheel_pose', 3, (0.15, 0.55, 1.0), 0.27
            )

        visual = self._latest['vo']
        if visual is not None and visual.header.frame_id:
            markers[4] = self._pose_arrow(
                visual, 'visual_pose', 4, (1.0, 0.30, 0.85), 0.27
            )

        # Exactly five markers update/delete deterministic namespace/ID pairs.
        self._publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
