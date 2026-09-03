"""Read-only markers for Robot SAVO's three physical range sensors."""

from dataclasses import dataclass
import math
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from savo_observer.range_state import range_sample_state


RANGE_MARKERS_TOPIC = '/savo_observer/range_markers'


@dataclass(frozen=True)
class SensorContract:
    label: str
    topic: str
    frame: str
    color: tuple[float, float, float]


SENSORS = (
    SensorContract(
        'LEFT',
        '/savo_perception/range/left_m',
        'tof_left_link',
        (0.10, 0.80, 1.0),
    ),
    SensorContract(
        'RIGHT',
        '/savo_perception/range/right_m',
        'tof_right_link',
        (0.20, 0.50, 1.0),
    ),
    SensorContract(
        'FRONT',
        '/savo_perception/range/front_ultrasonic_m',
        'ultrasonic_front_link',
        (0.20, 1.0, 0.35),
    ),
)


def _set_color(marker, color, alpha=1.0):
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(alpha)


def _point(x, y=0.0, z=0.0):
    value = Point()
    value.x = float(x)
    value.y = float(y)
    value.z = float(z)
    return value


class RangeVisualizer(Node):
    """Display measured ranges in their source-owned TF sensor frames."""

    def __init__(self):
        super().__init__('range_visualizer_node')
        self.declare_parameter('markers_topic', RANGE_MARKERS_TOPIC)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('stale_after_s', 1.0)
        for sensor in SENSORS:
            key = sensor.label.lower()
            self.declare_parameter(f'{key}_topic', sensor.topic)
            self.declare_parameter(f'{key}_frame', sensor.frame)

        publish_rate_hz = self._positive_parameter('publish_rate_hz')
        self._stale_after_s = self._positive_parameter('stale_after_s')
        self._latest = {
            sensor.label: {'value': None, 'received': None}
            for sensor in SENSORS
        }
        self._contracts = tuple(
            SensorContract(
                sensor.label,
                self.get_parameter(
                    f'{sensor.label.lower()}_topic'
                ).value,
                self.get_parameter(
                    f'{sensor.label.lower()}_frame'
                ).value,
                sensor.color,
            )
            for sensor in SENSORS
        )
        self._publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('markers_topic').value,
            10,
        )
        self._subscriptions = [
            self.create_subscription(
                Float32,
                sensor.topic,
                lambda message, label=sensor.label: self._remember(
                    label, message.data
                ),
                qos_profile_sensor_data,
            )
            for sensor in self._contracts
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

    def _remember(self, label, value):
        self._latest[label] = {
            'value': float(value),
            'received': time.monotonic(),
        }

    def _base_marker(self, sensor, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = sensor.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'range_{sensor.label.lower()}'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _delete_marker(self, sensor, marker_id, marker_type):
        marker = self._base_marker(sensor, marker_id, marker_type)
        marker.action = Marker.DELETE
        return marker

    def _sensor_markers(self, sensor, sensor_index, now):
        base_id = 4 * sensor_index
        sample = self._latest[sensor.label]
        state = range_sample_state(
            sample['value'],
            sample['received'],
            now,
            self._stale_after_s,
        )
        valid = state == 'VALID'
        if valid:
            length = sample['value']
            beam = self._base_marker(
                sensor, base_id, Marker.LINE_STRIP
            )
            beam.scale.x = 0.025
            beam.points = [_point(0.0), _point(length)]
            _set_color(beam, sensor.color, 0.95)

            endpoint = self._base_marker(
                sensor, base_id + 1, Marker.SPHERE
            )
            endpoint.pose.position.x = length
            endpoint.scale.x = 0.07
            endpoint.scale.y = 0.07
            endpoint.scale.z = 0.035
            _set_color(endpoint, sensor.color)
        else:
            length = 0.12
            beam = self._delete_marker(
                sensor, base_id, Marker.LINE_STRIP
            )
            endpoint = self._delete_marker(
                sensor, base_id + 1, Marker.SPHERE
            )

        status = self._base_marker(
            sensor, base_id + 2, Marker.SPHERE
        )
        status.scale.x = 0.045
        status.scale.y = 0.045
        status.scale.z = 0.025
        status_color = {
            'VALID': sensor.color,
            'STALE': (0.55, 0.55, 0.55),
            'INVALID': (1.0, 0.35, 0.05),
            'MISSING': (0.35, 0.35, 0.35),
        }[state]
        _set_color(status, status_color)

        label = self._base_marker(
            sensor, base_id + 3, Marker.TEXT_VIEW_FACING
        )
        label.pose.position.x = length
        label.pose.position.z = 0.085
        label.scale.z = 0.065
        label.text = (
            f'{sensor.label} {sample["value"]:.2f} m'
            if valid
            else f'{sensor.label} {state}'
        )
        _set_color(label, status_color)
        return (beam, endpoint, status, label)

    def _publish_markers(self):
        now = time.monotonic()
        markers = []
        for index, sensor in enumerate(self._contracts):
            markers.extend(self._sensor_markers(sensor, index, now))
        # Exactly twelve deterministic marker IDs are updated or deleted.
        self._publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = RangeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
