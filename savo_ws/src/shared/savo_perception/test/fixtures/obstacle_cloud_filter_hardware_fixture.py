#!/usr/bin/env python3

# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Subscriber-only real-hardware validator for the Phase 8A5 D435 cloud filter."""

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


DEFAULT_CONFIG = (
    Path.home()
    / 'Savo_Pi'
    / 'savo_ws'
    / 'src'
    / 'shared'
    / 'savo_perception'
    / 'config'
    / 'edge'
    / 'obstacle_cloud_filter.yaml'
)

DEFAULT_RAW_TOPIC = '/camera/camera/depth/color/points'
DEFAULT_OUTPUT_TOPIC = '/savo_perception/obstacles/points'
DEFAULT_HEALTH_TOPIC = '/savo_perception/obstacle_cloud/health'
DEFAULT_STATUS_TOPIC = '/savo_perception/obstacle_cloud/status'
DEFAULT_HEARTBEAT_TOPIC = '/savo_perception/obstacle_cloud/heartbeat'
DEFAULT_OUTPUT_FRAME = 'base_link'

BODY_CANDIDATE_ENVELOPE = {
    'x': (-0.50, 0.50),
    'y': (-0.45, 0.45),
    'z': (-0.20, 1.00),
}

DEFAULT_FORBIDDEN_TOPICS = (
    '/cmd_vel',
    '/cmd_vel_safe',
    '/cmd_vel_recovery',
    '/cmd_vel_nav',
    '/savo_control/mode_cmd',
    '/savo_control/recovery_request',
    '/goal_pose',
    '/navigate_to_pose/_action/goal',
    '/follow_waypoints/_action/goal',
)

SCENE_NAMES = (
    'clear',
    'front',
    'left',
    'right',
    'low',
    'normal',
    'high',
    'near',
    'mid',
    'far',
    'self',
    'self_body',
    'self_nearby',
)


class ValidationFailure(RuntimeError):
    """A deterministic hardware-validation failure."""


class SubscriberOnlyViolation(RuntimeError):
    """An attempt to create a publisher from the observer."""


class _NoOpParameterEventPublisher:
    """Absorb rclpy's intrinsic parameter event without creating an endpoint."""

    def publish(self, _message):
        """Intentionally emit no ROS message."""


def normalized_frame(frame_id):
    """Normalize a TF frame name without inventing or remapping it."""
    return str(frame_id).strip().lstrip('/')


def stamp_nanoseconds(stamp):
    """Convert a ROS builtin time message to an integer nanosecond stamp."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def safe_mean(values):
    """Return a JSON-safe arithmetic mean or None."""
    if not values:
        return None
    return float(statistics.fmean(values))


def safe_min(values):
    """Return a JSON-safe minimum or None."""
    if not values:
        return None
    return float(min(values))


def safe_max(values):
    """Return a JSON-safe maximum or None."""
    if not values:
        return None
    return float(max(values))


def percentile(values, percentage):
    """Return a JSON-safe NumPy percentile or None."""
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentage))


def summarize_integer_values(values):
    """Summarize integer measurements without retaining raw point data."""
    if not values:
        return {
            'samples': 0,
            'minimum': None,
            'maximum': None,
            'average': None,
            'median': None,
        }

    return {
        'samples': len(values),
        'minimum': int(min(values)),
        'maximum': int(max(values)),
        'average': float(statistics.fmean(values)),
        'median': float(statistics.median(values)),
    }


def summarize_stream(receive_times):
    """Compute rate, gap, and interval-jitter measurements."""
    if len(receive_times) < 2:
        return {
            'messages': len(receive_times),
            'average_rate_hz': None,
            'minimum_instantaneous_rate_hz': None,
            'maximum_instantaneous_rate_hz': None,
            'maximum_gap_s': None,
            'interval_cv': None,
            'observation_span_s': 0.0,
        }

    intervals = [
        current - previous
        for previous, current in zip(receive_times, receive_times[1:])
        if current > previous
    ]

    if not intervals:
        return {
            'messages': len(receive_times),
            'average_rate_hz': None,
            'minimum_instantaneous_rate_hz': None,
            'maximum_instantaneous_rate_hz': None,
            'maximum_gap_s': None,
            'interval_cv': None,
            'observation_span_s': 0.0,
        }

    span = receive_times[-1] - receive_times[0]
    average_rate = (len(receive_times) - 1) / span if span > 0.0 else None
    mean_interval = statistics.fmean(intervals)
    interval_cv = (
        statistics.pstdev(intervals) / mean_interval
        if len(intervals) > 1 and mean_interval > 0.0
        else 0.0
    )

    return {
        'messages': len(receive_times),
        'average_rate_hz': float(average_rate) if average_rate is not None else None,
        'minimum_instantaneous_rate_hz': float(1.0 / max(intervals)),
        'maximum_instantaneous_rate_hz': float(1.0 / min(intervals)),
        'maximum_gap_s': float(max(intervals)),
        'interval_cv': float(interval_cv),
        'observation_span_s': float(span),
    }


def validate_increasing_stamps(stamps, stream_name):
    """Reject zero, duplicate, or backwards message timestamps."""
    if not stamps:
        raise ValidationFailure(f'{stream_name}: no timestamps were observed')

    zero_count = sum(stamp <= 0 for stamp in stamps)
    if zero_count:
        raise ValidationFailure(
            f'{stream_name}: observed {zero_count} zero or negative timestamps'
        )

    duplicate_count = len(stamps) - len(set(stamps))
    if duplicate_count:
        raise ValidationFailure(
            f'{stream_name}: observed {duplicate_count} duplicate timestamps'
        )

    backwards = sum(
        current <= previous
        for previous, current in zip(stamps, stamps[1:])
    )
    if backwards:
        raise ValidationFailure(
            f'{stream_name}: observed {backwards} non-advancing timestamps'
        )


def load_filter_configuration(path):
    """Load and strictly validate the production obstacle-cloud YAML."""
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ValidationFailure(
            f'could not load obstacle-cloud configuration {path}: {error}'
        ) from error

    if not isinstance(document, dict):
        raise ValidationFailure(f'configuration is not a YAML mapping: {path}')

    node_section = document.get('obstacle_cloud_filter_node')
    if not isinstance(node_section, dict):
        raise ValidationFailure(
            'configuration lacks obstacle_cloud_filter_node mapping'
        )

    parameters = node_section.get('ros__parameters')
    if not isinstance(parameters, dict):
        raise ValidationFailure(
            'configuration lacks obstacle_cloud_filter_node.ros__parameters'
        )

    required = (
        'input_topic',
        'output_topic',
        'output_frame',
        'health_topic',
        'status_topic',
        'heartbeat_topic',
        'min_range_m',
        'max_range_m',
        'min_height_m',
        'max_height_m',
        'voxel_size_m',
        'self_filter_enabled',
        'self_min_x_m',
        'self_max_x_m',
        'self_min_y_m',
        'self_max_y_m',
        'self_min_z_m',
        'self_max_z_m',
        'max_output_points',
        'transform_timeout_s',
        'stale_timeout_s',
        'status_publish_hz',
        'heartbeat_hz',
    )

    missing = [name for name in required if name not in parameters]
    if missing:
        raise ValidationFailure(
            'configuration is missing required parameters: '
            + ', '.join(sorted(missing))
        )

    numeric_names = (
        'min_range_m',
        'max_range_m',
        'min_height_m',
        'max_height_m',
        'voxel_size_m',
        'self_min_x_m',
        'self_max_x_m',
        'self_min_y_m',
        'self_max_y_m',
        'self_min_z_m',
        'self_max_z_m',
        'transform_timeout_s',
        'stale_timeout_s',
        'status_publish_hz',
        'heartbeat_hz',
    )

    for name in numeric_names:
        try:
            value = float(parameters[name])
        except (TypeError, ValueError) as error:
            raise ValidationFailure(
                f'configuration parameter {name} is not numeric'
            ) from error

        if not math.isfinite(value):
            raise ValidationFailure(
                f'configuration parameter {name} is not finite'
            )

        parameters[name] = value

    try:
        parameters['max_output_points'] = int(parameters['max_output_points'])
    except (TypeError, ValueError) as error:
        raise ValidationFailure(
            'configuration parameter max_output_points is not an integer'
        ) from error

    if float(parameters['min_range_m']) < 0.0:
        raise ValidationFailure('min_range_m must be nonnegative')
    if float(parameters['max_range_m']) <= float(parameters['min_range_m']):
        raise ValidationFailure('max_range_m must be greater than min_range_m')
    if float(parameters['max_height_m']) <= float(parameters['min_height_m']):
        raise ValidationFailure('max_height_m must be greater than min_height_m')
    if float(parameters['voxel_size_m']) <= 0.0:
        raise ValidationFailure('voxel_size_m must be positive')
    if int(parameters['max_output_points']) <= 0:
        raise ValidationFailure('max_output_points must be positive')

    for axis in ('x', 'y', 'z'):
        if (
            float(parameters[f'self_max_{axis}_m'])
            <= float(parameters[f'self_min_{axis}_m'])
        ):
            raise ValidationFailure(
                f'self-filter {axis} maximum must exceed its minimum'
            )

    for name in (
        'input_topic',
        'output_topic',
        'output_frame',
        'health_topic',
        'status_topic',
        'heartbeat_topic',
    ):
        value = str(parameters[name]).strip()
        if not value:
            raise ValidationFailure(
                f'configuration parameter {name} is empty'
            )
        parameters[name] = value

    parameters['self_filter_enabled'] = bool(
        parameters['self_filter_enabled']
    )
    return parameters


def cloud_layout(message, label, allow_empty, xyz_only):
    """Validate PointCloud2 metadata and return XYZ fields."""
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)

    if height <= 0:
        raise ValidationFailure(f'{label}: height must be positive')
    if width < 0 or (width == 0 and not allow_empty):
        raise ValidationFailure(f'{label}: width must be positive')
    if point_step <= 0:
        raise ValidationFailure(f'{label}: point_step must be positive')
    if bool(message.is_bigendian):
        raise ValidationFailure(f'{label}: big-endian data is unsupported')

    expected_row_step = point_step * width
    expected_data_size = expected_row_step * height

    if row_step != expected_row_step:
        raise ValidationFailure(
            f'{label}: row_step={row_step}, expected {expected_row_step}'
        )
    if len(message.data) != expected_data_size:
        raise ValidationFailure(
            f'{label}: data size={len(message.data)}, '
            f'expected {expected_data_size}'
        )

    names = [str(field.name) for field in message.fields]
    if len(names) != len(set(names)):
        raise ValidationFailure(f'{label}: duplicate PointField names')
    if xyz_only and names != ['x', 'y', 'z']:
        raise ValidationFailure(
            f'{label}: expected XYZ-only fields, received {names!r}'
        )

    by_name = {str(field.name): field for field in message.fields}
    xyz_fields = {}

    for name in ('x', 'y', 'z'):
        field = by_name.get(name)
        if field is None:
            raise ValidationFailure(f'{label}: missing {name!r} field')
        if int(field.datatype) != PointField.FLOAT32:
            raise ValidationFailure(
                f'{label}: field {name!r} is not FLOAT32'
            )
        if int(field.count) != 1:
            raise ValidationFailure(
                f'{label}: field {name!r} count is not one'
            )
        if int(field.offset) < 0 or int(field.offset) + 4 > point_step:
            raise ValidationFailure(
                f'{label}: field {name!r} lies outside point_step'
            )
        xyz_fields[name] = field

    if xyz_only:
        expected_offsets = {'x': 0, 'y': 4, 'z': 8}
        actual_offsets = {
            name: int(field.offset)
            for name, field in xyz_fields.items()
        }
        if actual_offsets != expected_offsets or point_step != 12:
            raise ValidationFailure(
                f'{label}: noncanonical XYZ layout '
                f'offsets={actual_offsets}, point_step={point_step}'
            )
        if not bool(message.is_dense):
            raise ValidationFailure(
                f'{label}: finite filtered output must be marked dense'
            )

    return xyz_fields


def decode_xyz(message, xyz_fields):
    """Decode every XYZ point with a zero-copy structured view."""
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    height = int(message.height)
    width = int(message.width)

    dtype = np.dtype(
        {
            'names': ('x', 'y', 'z'),
            'formats': ('<f4', '<f4', '<f4'),
            'offsets': tuple(
                int(xyz_fields[name].offset)
                for name in ('x', 'y', 'z')
            ),
            'itemsize': point_step,
        }
    )

    view = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=memoryview(message.data),
        strides=(row_step, point_step),
    )

    return np.column_stack(
        (
            np.asarray(view['x']).reshape(-1),
            np.asarray(view['y']).reshape(-1),
            np.asarray(view['z']).reshape(-1),
        )
    ).astype(np.float64, copy=False)


def canonical_quaternion(values):
    """Normalize a quaternion and canonicalize its equivalent sign."""
    quaternion = np.asarray(values, dtype=np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValidationFailure('TF quaternion is not finite')

    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12:
        raise ValidationFailure('TF quaternion has zero norm')

    quaternion = quaternion / norm

    for value in reversed(quaternion):
        if abs(float(value)) <= 1.0e-15:
            continue
        if value < 0.0:
            quaternion = -quaternion
        break

    return quaternion


def transform_components(transform_message):
    """Return finite translation and normalized quaternion arrays."""
    translation_message = transform_message.transform.translation
    rotation_message = transform_message.transform.rotation

    translation = np.asarray(
        (
            translation_message.x,
            translation_message.y,
            translation_message.z,
        ),
        dtype=np.float64,
    )

    if not np.isfinite(translation).all():
        raise ValidationFailure('TF translation is not finite')

    quaternion = canonical_quaternion(
        (
            rotation_message.x,
            rotation_message.y,
            rotation_message.z,
            rotation_message.w,
        )
    )
    return translation, quaternion


def rotation_matrix(quaternion):
    """Convert normalized XYZW quaternion coordinates to a rotation matrix."""
    x_value, y_value, z_value, w_value = quaternion

    return np.asarray(
        (
            (
                1.0 - 2.0 * (y_value * y_value + z_value * z_value),
                2.0 * (x_value * y_value - z_value * w_value),
                2.0 * (x_value * z_value + y_value * w_value),
            ),
            (
                2.0 * (x_value * y_value + z_value * w_value),
                1.0 - 2.0 * (x_value * x_value + z_value * z_value),
                2.0 * (y_value * z_value - x_value * w_value),
            ),
            (
                2.0 * (x_value * z_value - y_value * w_value),
                2.0 * (y_value * z_value + x_value * w_value),
                1.0 - 2.0 * (x_value * x_value + y_value * y_value),
            ),
        ),
        dtype=np.float64,
    )


def transform_points(points, transform_message):
    """Transform raw camera points into the configured output frame."""
    translation, quaternion = transform_components(transform_message)
    matrix = rotation_matrix(quaternion)
    return points @ matrix.T + translation


def endpoint_node_name(endpoint):
    """Return a normalized fully qualified node name."""
    namespace = str(endpoint.node_namespace or '/')
    node_name = str(endpoint.node_name)

    if namespace == '/':
        return '/' + node_name
    return namespace.rstrip('/') + '/' + node_name


def enum_name(value):
    """Render a ROS enum value as a stable string."""
    name = getattr(value, 'name', None)
    if name is not None:
        return str(name)
    return str(value)


def endpoint_to_dict(endpoint):
    """Convert ROS topic endpoint information to JSON-safe data."""
    endpoint_gid = endpoint.endpoint_gid

    try:
        gid = bytes(endpoint_gid).hex()
    except (TypeError, ValueError):
        gid = str(endpoint_gid)

    qos = endpoint.qos_profile
    return {
        'node': endpoint_node_name(endpoint),
        'node_name': str(endpoint.node_name),
        'node_namespace': str(endpoint.node_namespace),
        'topic_type': str(endpoint.topic_type),
        'endpoint_gid': gid,
        'qos': {
            'reliability': enum_name(qos.reliability),
            'durability': enum_name(qos.durability),
            'history': enum_name(qos.history),
            'depth': int(qos.depth),
        },
    }


def scene_region_masks(points, config, tolerance):
    """Classify transformed points into every physical Phase 8A5 region."""
    finite = np.isfinite(points).all(axis=1)
    x_values = points[:, 0]
    y_values = points[:, 1]
    z_values = points[:, 2]
    ranges = np.hypot(x_values, y_values)

    minimum_range = float(config['min_range_m'])
    maximum_range = float(config['max_range_m'])
    minimum_height = float(config['min_height_m'])
    maximum_height = float(config['max_height_m'])

    in_range = (
        finite
        & (ranges >= minimum_range - tolerance)
        & (ranges <= maximum_range + tolerance)
    )
    in_height = (
        finite
        & (z_values >= minimum_height - tolerance)
        & (z_values <= maximum_height + tolerance)
    )
    normal_band = in_range & in_height

    self_box = (
        finite
        & (x_values >= float(config['self_min_x_m']) - tolerance)
        & (x_values <= float(config['self_max_x_m']) + tolerance)
        & (y_values >= float(config['self_min_y_m']) - tolerance)
        & (y_values <= float(config['self_max_y_m']) + tolerance)
        & (z_values >= float(config['self_min_z_m']) - tolerance)
        & (z_values <= float(config['self_max_z_m']) + tolerance)
    )

    return {
        'clear': normal_band & ~self_box,
        'front': (
            normal_band
            & (x_values >= 0.40)
            & (x_values <= 1.00)
            & (np.abs(y_values) <= 0.40)
        ),
        'left': (
            normal_band
            & (x_values >= 0.20)
            & (x_values <= 1.50)
            & (y_values >= 0.15)
            & (y_values <= 1.50)
        ),
        'right': (
            normal_band
            & (x_values >= 0.20)
            & (x_values <= 1.50)
            & (y_values <= -0.15)
            & (y_values >= -1.50)
        ),
        'low': (
            in_range
            & (z_values < minimum_height - tolerance)
        ),
        'normal': normal_band & ~self_box,
        'high': (
            in_range
            & (z_values > maximum_height + tolerance)
        ),
        'near': (
            finite
            & (ranges < minimum_range - tolerance)
        ),
        'mid': normal_band & ~self_box,
        'far': (
            finite
            & (ranges > maximum_range + tolerance)
        ),
        'self': self_box,
        'self_body': self_box,
        'self_nearby': (
            normal_band
            & ~self_box
            & (
                x_values
                >= float(config['self_min_x_m']) - 0.15
            )
            & (
                x_values
                <= float(config['self_max_x_m']) + 0.35
            )
            & (
                y_values
                >= float(config['self_min_y_m']) - 0.20
            )
            & (
                y_values
                <= float(config['self_max_y_m']) + 0.20
            )
            & (
                z_values
                >= max(
                    minimum_height - tolerance,
                    float(config['self_min_z_m']) - 0.10,
                )
            )
            & (
                z_values
                <= min(
                    maximum_height + tolerance,
                    float(config['self_max_z_m']) + 0.25,
                )
            )
        ),
    }


class HardwareObserver(Node):
    """Subscribe to real hardware streams and perform bounded validation."""

    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        """Prohibit every application or ROS graph publisher endpoint."""
        if (
            str(topic) == '/parameter_events'
            and not getattr(self, '_subscriber_only_initialized', False)
        ):
            return _NoOpParameterEventPublisher()

        raise SubscriberOnlyViolation(
            f'subscriber-only observer refused publisher creation on {topic}'
        )

    def __init__(self, arguments, configuration):
        """Construct only subscriptions, graph readers, and a TF listener."""
        self._subscriber_only_initialized = False

        super().__init__(
            'phase8a5_d435_hardware_observer',
            use_global_arguments=False,
            enable_rosout=False,
            start_parameter_services=False,
            enable_logger_service=False,
        )

        self._subscriber_only_initialized = True
        self.arguments = arguments
        self.config = dict(configuration)
        self.failures = []
        self.failure_keys = set()

        self.receive_times = defaultdict(list)
        self.message_stamps = defaultdict(list)
        self.point_counts = defaultdict(list)
        self.frames = defaultdict(set)
        self.raw_receive_by_stamp = {}
        self.output_receive_by_stamp = {}
        self.raw_message_counter = 0
        self.output_message_counter = 0
        self.output_messages_validated = 0
        self.raw_messages_analyzed = 0
        self.stamp_ages = defaultdict(list)

        self.health_values = []
        self.status_documents = []
        self.heartbeat_documents = []
        self.heartbeat_counters = []

        self.scene_counts = {
            'raw': defaultdict(list),
            'output': defaultdict(list),
        }
        self.scene_observations = {
            'raw': defaultdict(dict),
            'output': defaultdict(dict),
        }
        self.raw_self_extent_min = None
        self.raw_self_extent_max = None
        self.raw_self_extent_points = 0
        self.raw_body_candidate_extent_min = None
        self.raw_body_candidate_extent_max = None
        self.raw_body_candidate_extent_points = 0

        self.tf_attempts = 0
        self.tf_successes = 0
        self.tf_initial_misses = 0
        self.tf_post_success_failures = 0
        self.tf_consecutive_failures = 0
        self.tf_max_consecutive_failures = 0
        self.tf_seen_success = False
        self.tf_failure_messages = []
        self.tf_samples = []
        self.static_transform_signatures = defaultdict(set)
        self.static_transform_publisher_gids = defaultdict(set)

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cloud_qos = QoSProfile(
            history=qos_profile_sensor_data.history,
            depth=50,
            reliability=qos_profile_sensor_data.reliability,
            durability=qos_profile_sensor_data.durability,
        )

        self.create_subscription(
            PointCloud2,
            self.config['input_topic'],
            self._on_raw_cloud,
            cloud_qos,
        )
        self.create_subscription(
            PointCloud2,
            self.config['output_topic'],
            self._on_output_cloud,
            cloud_qos,
        )
        self.create_subscription(
            Bool,
            self.config['health_topic'],
            self._on_health,
            status_qos,
        )
        self.create_subscription(
            String,
            self.config['status_topic'],
            self._on_status,
            status_qos,
        )
        self.create_subscription(
            String,
            self.config['heartbeat_topic'],
            self._on_heartbeat,
            status_qos,
        )

        self.tf_buffer = Buffer(
            cache_time=Duration(
                seconds=max(30.0, float(arguments.duration) + 10.0)
            ),
            node=self,
        )
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self.create_subscription(
            TFMessage,
            '/tf_static',
            self._on_static_tf,
            static_tf_qos,
        )

    def record_failure(self, code, detail):
        """Record a bounded, de-duplicated explicit validation failure."""
        key = (str(code), str(detail))
        if key in self.failure_keys:
            return

        self.failure_keys.add(key)
        if len(self.failures) < int(self.arguments.maximum_failures):
            self.failures.append(
                {
                    'code': str(code),
                    'detail': str(detail),
                }
            )

    def _record_stamp_age(self, stream_name, stamp):
        """Measure message timestamp age using the node's ROS clock."""
        stamp_ns = stamp_nanoseconds(stamp)
        now_ns = int(self.get_clock().now().nanoseconds)

        if stamp_ns <= 0 or now_ns <= 0:
            return

        age_seconds = (now_ns - stamp_ns) / 1_000_000_000.0
        if math.isfinite(age_seconds):
            self.stamp_ages[stream_name].append(float(age_seconds))

    def _record_scene_counts(self, stream_name, stamp_ns, points):
        """Accumulate per-frame counts for every physical-scene region."""
        masks = scene_region_masks(
            points,
            self.config,
            float(self.arguments.tolerance),
        )

        for name, mask in masks.items():
            count = int(np.count_nonzero(mask))
            self.scene_counts[stream_name][name].append(count)
            selected_points = points[mask]
            observation = {
                'count': count,
                'minimum_xyz': None,
                'maximum_xyz': None,
                'centroid_xyz': None,
            }

            if count:
                observation['minimum_xyz'] = [
                    float(value)
                    for value in np.min(selected_points, axis=0)
                ]
                observation['maximum_xyz'] = [
                    float(value)
                    for value in np.max(selected_points, axis=0)
                ]
                observation['centroid_xyz'] = [
                    float(value)
                    for value in np.mean(selected_points, axis=0)
                ]

            self.scene_observations[stream_name][name][
                int(stamp_ns)
            ] = observation

        if stream_name == 'raw' and np.any(masks['self']):
            self_points = points[masks['self']]
            frame_minimum = np.min(self_points, axis=0)
            frame_maximum = np.max(self_points, axis=0)

            if self.raw_self_extent_min is None:
                self.raw_self_extent_min = frame_minimum
                self.raw_self_extent_max = frame_maximum
            else:
                self.raw_self_extent_min = np.minimum(
                    self.raw_self_extent_min,
                    frame_minimum,
                )
                self.raw_self_extent_max = np.maximum(
                    self.raw_self_extent_max,
                    frame_maximum,
                )

            self.raw_self_extent_points += int(self_points.shape[0])

        if stream_name == 'raw':
            finite = np.isfinite(points).all(axis=1)
            body_candidate_mask = (
                finite
                & (
                    points[:, 0]
                    >= BODY_CANDIDATE_ENVELOPE['x'][0]
                )
                & (
                    points[:, 0]
                    <= BODY_CANDIDATE_ENVELOPE['x'][1]
                )
                & (
                    points[:, 1]
                    >= BODY_CANDIDATE_ENVELOPE['y'][0]
                )
                & (
                    points[:, 1]
                    <= BODY_CANDIDATE_ENVELOPE['y'][1]
                )
                & (
                    points[:, 2]
                    >= BODY_CANDIDATE_ENVELOPE['z'][0]
                )
                & (
                    points[:, 2]
                    <= BODY_CANDIDATE_ENVELOPE['z'][1]
                )
            )

            if np.any(body_candidate_mask):
                candidate_points = points[body_candidate_mask]
                frame_minimum = np.min(candidate_points, axis=0)
                frame_maximum = np.max(candidate_points, axis=0)

                if self.raw_body_candidate_extent_min is None:
                    self.raw_body_candidate_extent_min = frame_minimum
                    self.raw_body_candidate_extent_max = frame_maximum
                else:
                    self.raw_body_candidate_extent_min = np.minimum(
                        self.raw_body_candidate_extent_min,
                        frame_minimum,
                    )
                    self.raw_body_candidate_extent_max = np.maximum(
                        self.raw_body_candidate_extent_max,
                        frame_maximum,
                    )

                self.raw_body_candidate_extent_points += int(
                    candidate_points.shape[0]
                )

    def _inspect_exact_transform(self, message):
        """Look up and validate raw-frame TF at the real cloud timestamp."""
        source_frame = normalized_frame(message.header.frame_id)
        target_frame = normalized_frame(self.config['output_frame'])
        self.tf_attempts += 1

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.0),
            )
            translation, quaternion = transform_components(transform)
        except (TransformException, ValidationFailure) as error:
            self.tf_consecutive_failures += 1
            self.tf_max_consecutive_failures = max(
                self.tf_max_consecutive_failures,
                self.tf_consecutive_failures,
            )

            if self.tf_seen_success:
                self.tf_post_success_failures += 1
            else:
                self.tf_initial_misses += 1

            if len(self.tf_failure_messages) < 20:
                self.tf_failure_messages.append(str(error))
            return None

        self.tf_seen_success = True
        self.tf_successes += 1
        self.tf_consecutive_failures = 0

        if len(self.tf_samples) < 20:
            self.tf_samples.append(
                {
                    'stamp_ns': stamp_nanoseconds(message.header.stamp),
                    'source_frame': source_frame,
                    'target_frame': target_frame,
                    'translation': [
                        float(value)
                        for value in translation
                    ],
                    'quaternion_xyzw': [
                        float(value)
                        for value in quaternion
                    ],
                }
            )

        return transform

    def _on_raw_cloud(self, message):
        """Validate real raw-cloud metadata and sample transformed scene points."""
        received = time.monotonic()
        self.receive_times['raw'].append(received)
        self.raw_message_counter += 1

        stamp_ns = stamp_nanoseconds(message.header.stamp)
        self.message_stamps['raw'].append(stamp_ns)
        self.point_counts['raw'].append(
            int(message.width) * int(message.height)
        )
        self.raw_receive_by_stamp.setdefault(stamp_ns, received)
        self._record_stamp_age('raw', message.header.stamp)

        frame = normalized_frame(message.header.frame_id)
        if not frame:
            self.record_failure('raw_frame', 'raw PointCloud2 frame_id is empty')
        else:
            self.frames['raw'].add(frame)

        try:
            xyz_fields = cloud_layout(
                message,
                'raw PointCloud2',
                allow_empty=False,
                xyz_only=False,
            )
        except ValidationFailure as error:
            self.record_failure('raw_metadata', error)
            return

        transform = self._inspect_exact_transform(message)

        if (
            transform is None
            or self.raw_message_counter
            % int(self.arguments.raw_analysis_stride) != 0
        ):
            return

        try:
            points = decode_xyz(message, xyz_fields)
            finite_points = points[np.isfinite(points).all(axis=1)]
            transformed = transform_points(finite_points, transform)
        except (ValidationFailure, TypeError, ValueError) as error:
            self.record_failure('raw_decode', error)
            return

        self.raw_messages_analyzed += 1
        self._record_scene_counts('raw', stamp_ns, transformed)

    def _validate_filtered_points(self, points):
        """Validate every decoded filtered XYZ point and every filter bound."""
        point_count = int(points.shape[0])
        tolerance = float(self.arguments.tolerance)

        if point_count > int(self.config['max_output_points']):
            raise ValidationFailure(
                f'filtered output has {point_count} points, exceeding '
                f"{self.config['max_output_points']}"
            )

        if not np.isfinite(points).all():
            invalid_count = int(
                np.count_nonzero(~np.isfinite(points).all(axis=1))
            )
            raise ValidationFailure(
                f'filtered output contains {invalid_count} non-finite points'
            )

        if point_count == 0:
            return

        ranges = np.hypot(points[:, 0], points[:, 1])
        below_range = ranges < float(self.config['min_range_m']) - tolerance
        above_range = ranges > float(self.config['max_range_m']) + tolerance
        below_height = (
            points[:, 2]
            < float(self.config['min_height_m']) - tolerance
        )
        above_height = (
            points[:, 2]
            > float(self.config['max_height_m']) + tolerance
        )

        violations = {
            'below minimum range': int(np.count_nonzero(below_range)),
            'above maximum range': int(np.count_nonzero(above_range)),
            'below minimum height': int(np.count_nonzero(below_height)),
            'above maximum height': int(np.count_nonzero(above_height)),
        }
        nonzero = {
            name: count
            for name, count in violations.items()
            if count
        }
        if nonzero:
            raise ValidationFailure(
                f'filtered output violates configured bounds: {nonzero}'
            )

        if bool(self.config['self_filter_enabled']):
            self_mask = (
                (points[:, 0] >= float(self.config['self_min_x_m']) - tolerance)
                & (
                    points[:, 0]
                    <= float(self.config['self_max_x_m']) + tolerance
                )
                & (
                    points[:, 1]
                    >= float(self.config['self_min_y_m']) - tolerance
                )
                & (
                    points[:, 1]
                    <= float(self.config['self_max_y_m']) + tolerance
                )
                & (
                    points[:, 2]
                    >= float(self.config['self_min_z_m']) - tolerance
                )
                & (
                    points[:, 2]
                    <= float(self.config['self_max_z_m']) + tolerance
                )
            )
            self_count = int(np.count_nonzero(self_mask))
            if self_count:
                raise ValidationFailure(
                    f'filtered output contains {self_count} self-box points'
                )

        voxel_size = float(self.config['voxel_size_m'])
        voxel_keys = np.floor(points / voxel_size).astype(np.int64)
        unique_count = int(
            np.unique(voxel_keys, axis=0).shape[0]
        )
        if unique_count != point_count:
            raise ValidationFailure(
                f'filtered output contains {point_count - unique_count} '
                'duplicate occupied voxels'
            )

    def _on_output_cloud(self, message):
        """Validate all metadata and all XYZ points in every filtered output."""
        received = time.monotonic()
        self.receive_times['output'].append(received)
        self.output_message_counter += 1

        stamp_ns = stamp_nanoseconds(message.header.stamp)
        self.message_stamps['output'].append(stamp_ns)
        self.output_receive_by_stamp.setdefault(stamp_ns, received)
        self._record_stamp_age('output', message.header.stamp)

        point_count = int(message.width) * int(message.height)
        self.point_counts['output'].append(point_count)
        if point_count > int(self.config['max_output_points']):
            self.record_failure(
                'output_point_limit',
                f'filtered output has {point_count} points, exceeding '
                f"{self.config['max_output_points']}",
            )

        frame = normalized_frame(message.header.frame_id)
        if frame:
            self.frames['output'].add(frame)
        else:
            self.record_failure(
                'output_frame',
                'filtered PointCloud2 frame_id is empty',
            )

        try:
            xyz_fields = cloud_layout(
                message,
                'filtered PointCloud2',
                allow_empty=True,
                xyz_only=True,
            )

            raw_selected_stamp = (
                stamp_ns
                in self.scene_observations['raw']['clear']
            )
            periodic_sample = (
                self.output_message_counter
                % int(self.arguments.output_analysis_stride)
                == 0
            )
            if not raw_selected_stamp and not periodic_sample:
                return

            points = decode_xyz(message, xyz_fields)
            self._validate_filtered_points(points)
            self._record_scene_counts('output', stamp_ns, points)
        except (ValidationFailure, TypeError, ValueError) as error:
            self.record_failure('output_contract', error)
            return

        self.output_messages_validated += 1

    def _on_health(self, message):
        """Record production health without publishing a synthetic status."""
        self.receive_times['health'].append(time.monotonic())
        self.health_values.append(bool(message.data))

    def _on_status(self, message):
        """Decode and validate production status JSON."""
        self.receive_times['status'].append(time.monotonic())

        try:
            document = json.loads(str(message.data))
        except (json.JSONDecodeError, TypeError) as error:
            self.record_failure('status_json', f'invalid status JSON: {error}')
            return

        if not isinstance(document, dict):
            self.record_failure('status_json', 'status JSON is not an object')
            return

        if document.get('semantics') != 'obstacle_only':
            self.record_failure(
                'status_semantics',
                f"unexpected semantics: {document.get('semantics')!r}",
            )
        if document.get('clearing_supported') is not False:
            self.record_failure(
                'status_clearing',
                'clearing_supported is not false',
            )

        for name in (
            'input_points',
            'output_points',
            'finite_points',
            'range_rejected',
            'height_rejected',
            'self_rejected',
            'voxel_rejected',
            'transform_failures',
            'malformed_clouds',
            'clouds_received',
            'clouds_published',
        ):
            value = document.get(name)
            if not isinstance(value, int) or value < 0:
                self.record_failure(
                    'status_counter',
                    f'status counter {name} is invalid: {value!r}',
                )

        output_points = document.get('output_points')
        if (
            isinstance(output_points, int)
            and output_points > int(self.config['max_output_points'])
        ):
            self.record_failure(
                'status_counter',
                f'status output_points exceeds maximum: {output_points}',
            )

        self.status_documents.append(document)

    def _on_heartbeat(self, message):
        """Decode and validate the production heartbeat counter."""
        self.receive_times['heartbeat'].append(time.monotonic())

        try:
            document = json.loads(str(message.data))
        except (json.JSONDecodeError, TypeError) as error:
            self.record_failure(
                'heartbeat_json',
                f'invalid heartbeat JSON: {error}',
            )
            return

        if not isinstance(document, dict):
            self.record_failure(
                'heartbeat_json',
                'heartbeat JSON is not an object',
            )
            return

        counter = document.get('counter')
        if not isinstance(counter, int) or counter < 0:
            self.record_failure(
                'heartbeat_counter',
                f'invalid heartbeat counter: {counter!r}',
            )
        else:
            if (
                self.heartbeat_counters
                and counter <= self.heartbeat_counters[-1]
            ):
                self.record_failure(
                    'heartbeat_counter',
                    f'heartbeat did not advance: '
                    f'{self.heartbeat_counters[-1]} -> {counter}',
                )
            self.heartbeat_counters.append(counter)

        self.heartbeat_documents.append(document)

    def _on_static_tf(self, message, message_info):
        """Record distinct static transforms to detect chain conflicts."""
        try:
            publisher_gid = bytes(message_info.publisher_gid).hex()
        except (AttributeError, TypeError, ValueError):
            publisher_gid = 'unknown'

        for transform in message.transforms:
            parent = normalized_frame(transform.header.frame_id)
            child = normalized_frame(transform.child_frame_id)

            if not parent or not child:
                self.record_failure(
                    'tf_static_frame',
                    'static transform contains an empty frame name',
                )
                continue

            try:
                translation, quaternion = transform_components(transform)
            except ValidationFailure as error:
                self.record_failure('tf_static_value', error)
                continue

            signature = tuple(
                round(float(value), 8)
                for value in np.concatenate((translation, quaternion))
            )
            self.static_transform_signatures[(parent, child)].add(signature)
            self.static_transform_publisher_gids[
                (parent, child, signature)
            ].add(publisher_gid)

    def run_for_duration(self):
        """Spin the subscriber-only observer for the requested interval."""
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self)
        deadline = time.monotonic() + float(self.arguments.duration)

        try:
            while rclpy.ok(context=self.context) and time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                executor.spin_once(timeout_sec=min(0.10, remaining))
        finally:
            executor.remove_node(self)
            executor.shutdown(timeout_sec=1.0)

    def _topic_endpoint_report(self, topic):
        """Return sorted publisher endpoint data for a topic."""
        endpoints = self.get_publishers_info_by_topic(topic)
        report = [endpoint_to_dict(endpoint) for endpoint in endpoints]
        return sorted(
            report,
            key=lambda item: (
                item['node'],
                item['topic_type'],
                item['endpoint_gid'],
            ),
        )

    def _validate_topic_endpoints(
        self,
        graph_report,
        key,
        topic,
        expected_count,
        expected_type,
    ):
        """Validate exact publisher count and message type."""
        endpoints = graph_report[key]['publishers']

        if len(endpoints) != expected_count:
            self.record_failure(
                'publisher_count',
                f'{topic}: expected {expected_count} publishers, '
                f'observed {len(endpoints)}',
            )

        unexpected_types = sorted(
            {
                endpoint['topic_type']
                for endpoint in endpoints
                if endpoint['topic_type'] != expected_type
            }
        )
        if unexpected_types:
            self.record_failure(
                'publisher_type',
                f'{topic}: unexpected publisher types {unexpected_types!r}',
            )

    def _graph_report(self):
        """Inspect production endpoints and prove observer authority absence."""
        topic_definitions = {
            'raw': (
                self.config['input_topic'],
                'sensor_msgs/msg/PointCloud2',
            ),
            'output': (
                self.config['output_topic'],
                'sensor_msgs/msg/PointCloud2',
            ),
            'health': (
                self.config['health_topic'],
                'std_msgs/msg/Bool',
            ),
            'status': (
                self.config['status_topic'],
                'std_msgs/msg/String',
            ),
            'heartbeat': (
                self.config['heartbeat_topic'],
                'std_msgs/msg/String',
            ),
        }

        graph_report = {}
        for key, (topic, _topic_type) in topic_definitions.items():
            graph_report[key] = {
                'topic': topic,
                'publishers': self._topic_endpoint_report(topic),
            }

        outage = (
            self.arguments.mode == 'stale'
            and self.arguments.stale_phase == 'outage'
        )
        expected_raw_publishers = 0 if outage else 1

        for key, (topic, topic_type) in topic_definitions.items():
            expected_count = (
                expected_raw_publishers
                if key == 'raw'
                else 1
            )
            self._validate_topic_endpoints(
                graph_report,
                key,
                topic,
                expected_count,
                topic_type,
            )

        filter_endpoint_keys = ('output', 'health', 'status', 'heartbeat')
        filter_nodes = {
            endpoint['node']
            for key in filter_endpoint_keys
            for endpoint in graph_report[key]['publishers']
        }

        if len(filter_nodes) != 1:
            self.record_failure(
                'filter_endpoint_identity',
                'filter output/status topics do not share one publisher node: '
                f'{sorted(filter_nodes)!r}',
            )

        for key in filter_endpoint_keys:
            for endpoint in graph_report[key]['publishers']:
                expected_filter_node = str(
                    self.arguments.expected_filter_node
                )
                if (
                    str(endpoint['node_name'])
                    != expected_filter_node
                ):
                    self.record_failure(
                        'filter_endpoint_identity',
                        f"{graph_report[key]['topic']}: publisher "
                        f"{endpoint['node']!r} does not have expected node name "
                        f'{expected_filter_node!r}',
                    )

        if not outage and len(graph_report['raw']['publishers']) == 1:
            raw_endpoint = graph_report['raw']['publishers'][0]
            expected_raw_node = str(self.arguments.expected_raw_node)
            if raw_endpoint['node'] != expected_raw_node:
                self.record_failure(
                    'raw_endpoint_identity',
                    f"{self.config['input_topic']}: sole publisher is "
                    f"{raw_endpoint['node']!r}, expected "
                    f'{expected_raw_node!r}',
                )

        observer_publishers = self.get_publisher_names_and_types_by_node(
            self.get_name(),
            self.get_namespace(),
        )
        graph_report['observer_publishers'] = [
            {
                'topic': topic,
                'types': list(types),
            }
            for topic, types in observer_publishers
        ]
        if observer_publishers:
            self.record_failure(
                'observer_authority',
                f'observer unexpectedly owns publishers: '
                f'{observer_publishers!r}',
            )

        observer_name = (
            self.get_namespace().rstrip('/')
            + '/'
            + self.get_name()
        )
        if not observer_name.startswith('/'):
            observer_name = '/' + observer_name

        forbidden_report = {}
        implicated_nodes = set(filter_nodes)
        implicated_nodes.add(observer_name)

        for topic in self.arguments.forbidden_topic:
            endpoints = self._topic_endpoint_report(topic)
            forbidden_report[topic] = endpoints
            implicated = [
                endpoint
                for endpoint in endpoints
                if endpoint['node'] in implicated_nodes
            ]
            if implicated:
                self.record_failure(
                    'forbidden_authority',
                    f'{topic}: forbidden publisher owned by filter/observer: '
                    f"{[item['node'] for item in implicated]!r}",
                )

        graph_report['forbidden_topics'] = forbidden_report
        tf_static_publishers = self._topic_endpoint_report('/tf_static')
        graph_report['tf_static'] = {
            'topic': '/tf_static',
            'publishers': tf_static_publishers,
        }
        observed_tf_static_nodes = {
            endpoint['node']
            for endpoint in tf_static_publishers
        }
        expected_tf_static_nodes = set(
            self.arguments.expected_tf_static_node
        )
        if outage:
            expected_tf_static_nodes.discard(
                self.arguments.expected_raw_node
            )
        missing_tf_static_nodes = sorted(
            expected_tf_static_nodes - observed_tf_static_nodes
        )
        unexpected_tf_static_nodes = sorted(
            observed_tf_static_nodes - expected_tf_static_nodes
        )
        graph_report['tf_static']['expected_nodes'] = sorted(
            expected_tf_static_nodes
        )
        graph_report['tf_static']['additional_nodes'] = (
            unexpected_tf_static_nodes
        )

        if missing_tf_static_nodes:
            self.record_failure(
                'tf_static_authority',
                f'missing expected /tf_static publishers: '
                f'{missing_tf_static_nodes!r}',
            )

        return graph_report

    def _tf_chain_report(self):
        """Resolve the exact discovered source-to-base TF parent chain."""
        source_frames = sorted(self.frames['raw'])
        target_frame = normalized_frame(self.config['output_frame'])
        report = {
            'source_frame': source_frames[0] if len(source_frames) == 1 else None,
            'target_frame': target_frame,
            'source_to_target': [],
            'target_to_source': [],
            'frame_metadata': {},
            'conflicting_static_transforms': [],
            'multiple_static_parents': [],
            'duplicate_static_publishers': [],
        }

        if len(source_frames) != 1:
            return report

        source_frame = source_frames[0]

        try:
            raw_frames = self.tf_buffer.all_frames_as_yaml()
            frame_metadata = yaml.safe_load(raw_frames) or {}
        except (TransformException, yaml.YAMLError, TypeError) as error:
            self.record_failure(
                'tf_tree',
                f'could not decode TF frame graph: {error}',
            )
            return report

        if not isinstance(frame_metadata, dict):
            self.record_failure('tf_tree', 'TF frame graph is not a mapping')
            return report

        normalized_metadata = {}
        for frame, metadata in frame_metadata.items():
            frame_name = normalized_frame(frame)
            if not isinstance(metadata, dict):
                continue
            normalized_metadata[frame_name] = {
                str(key): value
                for key, value in metadata.items()
            }

        current = source_frame
        source_to_target = [current]
        visited = set()

        while current != target_frame:
            if current in visited:
                self.record_failure(
                    'tf_tree',
                    f'TF parent cycle encountered at {current!r}',
                )
                break
            visited.add(current)

            metadata = normalized_metadata.get(current)
            if metadata is None:
                self.record_failure(
                    'tf_tree',
                    f'TF metadata has no frame {current!r}',
                )
                break

            parent = normalized_frame(metadata.get('parent', ''))
            if not parent or parent == 'NO_PARENT':
                self.record_failure(
                    'tf_tree',
                    f'TF chain from {source_frame!r} ended before '
                    f'{target_frame!r} at {current!r}',
                )
                break

            source_to_target.append(parent)
            current = parent

            if len(source_to_target) > 100:
                self.record_failure(
                    'tf_tree',
                    'TF chain exceeded 100 frames',
                )
                break

        if source_to_target[-1] != target_frame:
            self.record_failure(
                'tf_tree',
                f'no complete TF parent chain from {source_frame!r} '
                f'to {target_frame!r}',
            )

        report['source_to_target'] = source_to_target
        report['target_to_source'] = list(reversed(source_to_target))
        report['frame_metadata'] = {
            frame: normalized_metadata.get(frame, {})
            for frame in source_to_target
        }

        conflicts = []
        multiple_parents = []
        duplicate_publishers = []
        for child, parent in zip(
            source_to_target,
            source_to_target[1:],
        ):
            signatures = self.static_transform_signatures.get(
                (parent, child),
                set(),
            )
            if len(signatures) > 1:
                conflict = {
                    'parent': parent,
                    'child': child,
                    'distinct_transforms': len(signatures),
                }
                conflicts.append(conflict)
                self.record_failure(
                    'tf_static_conflict',
                    f'{parent} -> {child} has '
                    f'{len(signatures)} distinct static transforms',
                )

            publisher_gids = sorted(
                {
                    publisher_gid
                    for signature in signatures
                    for publisher_gid
                    in self.static_transform_publisher_gids.get(
                        (parent, child, signature),
                        set(),
                    )
                }
            )
            if len(publisher_gids) > 1:
                publisher_conflict = {
                    'parent': parent,
                    'child': child,
                    'publisher_gids': publisher_gids,
                }
                duplicate_publishers.append(publisher_conflict)

            observed_parents = sorted(
                {
                    observed_parent
                    for observed_parent, observed_child
                    in self.static_transform_signatures
                    if observed_child == child
                    and self.static_transform_signatures[
                        (observed_parent, observed_child)
                    ]
                }
            )
            if len(observed_parents) > 1:
                parent_conflict = {
                    'child': child,
                    'parents': observed_parents,
                }
                multiple_parents.append(parent_conflict)
                self.record_failure(
                    'tf_static_parent_conflict',
                    f'{child} has multiple static parents: '
                    f'{observed_parents!r}',
                )

        report['conflicting_static_transforms'] = conflicts
        report['multiple_static_parents'] = multiple_parents
        report['duplicate_static_publishers'] = duplicate_publishers
        return report

    def _validate_normal_streams(self, stream_report):
        """Validate message volume, rates, frames, stamps, and live status."""
        minimum_counts = {
            'raw': int(self.arguments.minimum_raw_messages),
            'output': int(self.arguments.minimum_output_messages),
            'health': int(self.arguments.minimum_status_messages),
            'status': int(self.arguments.minimum_status_messages),
            'heartbeat': int(self.arguments.minimum_heartbeat_messages),
        }

        for stream_name, minimum_count in minimum_counts.items():
            count = len(self.receive_times[stream_name])
            if count < minimum_count:
                self.record_failure(
                    'message_count',
                    f'{stream_name}: expected at least {minimum_count} '
                    f'messages, observed {count}',
                )

        rate_thresholds = {
            'raw': float(self.arguments.minimum_raw_rate_hz),
            'output': float(self.arguments.minimum_output_rate_hz),
            'status': float(self.arguments.minimum_status_rate_hz),
            'heartbeat': float(self.arguments.minimum_heartbeat_rate_hz),
        }

        for stream_name, minimum_rate in rate_thresholds.items():
            summary = stream_report[stream_name]
            average_rate = summary['average_rate_hz']
            maximum_gap = summary['maximum_gap_s']
            interval_cv = summary['interval_cv']
            maximum_gap_limit = float(self.arguments.maximum_gap_s)

            if stream_name == 'status':
                maximum_gap_limit = max(
                    maximum_gap_limit,
                    1.75 / float(self.config['status_publish_hz']),
                )
            elif stream_name == 'heartbeat':
                maximum_gap_limit = max(
                    maximum_gap_limit,
                    1.75 / float(self.config['heartbeat_hz']),
                )

            if average_rate is None or average_rate < minimum_rate:
                self.record_failure(
                    'stream_rate',
                    f'{stream_name}: average rate {average_rate!r} Hz is '
                    f'below {minimum_rate} Hz',
                )
            if (
                maximum_gap is None
                or maximum_gap > maximum_gap_limit
            ):
                self.record_failure(
                    'stream_gap',
                    f'{stream_name}: maximum gap {maximum_gap!r} s exceeds '
                    f'{maximum_gap_limit} s',
                )
            if (
                interval_cv is None
                or interval_cv
                > float(self.arguments.maximum_interval_cv)
            ):
                self.record_failure(
                    'stream_stability',
                    f'{stream_name}: interval CV {interval_cv!r} exceeds '
                    f'{self.arguments.maximum_interval_cv}',
                )

        for stream_name in ('raw', 'output'):
            try:
                validate_increasing_stamps(
                    self.message_stamps[stream_name],
                    stream_name,
                )
            except ValidationFailure as error:
                self.record_failure('timestamp_progression', error)

        raw_frames = sorted(self.frames['raw'])
        if len(raw_frames) != 1:
            self.record_failure(
                'raw_frame',
                f'expected exactly one nonempty raw frame, observed '
                f'{raw_frames!r}',
            )

        output_frames = sorted(self.frames['output'])
        expected_output_frame = normalized_frame(self.config['output_frame'])
        if output_frames != [expected_output_frame]:
            self.record_failure(
                'output_frame',
                f'expected only {expected_output_frame!r}, observed '
                f'{output_frames!r}',
            )

        if not self.health_values or self.health_values[-1] is not True:
            self.record_failure(
                'health_state',
                f'latest health is not ready/true: '
                f'{self.health_values[-1] if self.health_values else None!r}',
            )

        if not self.status_documents:
            self.record_failure('status_state', 'no valid status was decoded')
        elif self.status_documents[-1].get('state') != 'ready':
            latest_state = self.status_documents[-1].get('state')
            self.record_failure(
                'status_state',
                f'latest status state is {latest_state!r}, not ready',
            )

        if len(self.status_documents) >= 2:
            first_status = self.status_documents[0]
            latest_status = self.status_documents[-1]

            for error_counter in (
                'transform_failures',
                'malformed_clouds',
            ):
                first_value = first_status.get(error_counter)
                latest_value = latest_status.get(error_counter)
                if (
                    isinstance(first_value, int)
                    and isinstance(latest_value, int)
                    and latest_value != first_value
                ):
                    self.record_failure(
                        f'status_{error_counter}',
                        f'{error_counter} increased during steady '
                        f'observation: {first_value} -> {latest_value}',
                    )

            latest_malformed = latest_status.get('malformed_clouds')
            if isinstance(latest_malformed, int) and latest_malformed != 0:
                self.record_failure(
                    'status_malformed_clouds',
                    'malformed_clouds must remain zero during real D435 '
                    f'validation; latest value is {latest_malformed}',
                )

            for counter_name in ('clouds_received', 'clouds_published'):
                first_counter = first_status.get(counter_name)
                latest_counter = latest_status.get(counter_name)
                if (
                    not isinstance(first_counter, int)
                    or not isinstance(latest_counter, int)
                    or latest_counter <= first_counter
                ):
                    self.record_failure(
                        'status_progress',
                        f'{counter_name} did not progress during observation: '
                        f'{first_counter!r} -> {latest_counter!r}',
                    )

            latest_received = latest_status.get('clouds_received')
            latest_published = latest_status.get('clouds_published')
            if (
                isinstance(latest_received, int)
                and isinstance(latest_published, int)
                and latest_published > latest_received
            ):
                self.record_failure(
                    'status_progress',
                    f'clouds_published={latest_published} exceeds '
                    f'clouds_received={latest_received}',
                )

        if len(self.heartbeat_counters) < 2:
            self.record_failure(
                'heartbeat_counter',
                'fewer than two valid heartbeat counters were observed',
            )

        if (
            self.output_messages_validated
            < int(self.arguments.minimum_output_analysis_messages)
        ):
            self.record_failure(
                'output_validation',
                f'fully validated {self.output_messages_validated} output '
                f'messages, expected at least '
                f'{self.arguments.minimum_output_analysis_messages}',
            )

        if self.tf_successes <= 0:
            self.record_failure(
                'tf_lookup',
                'no exact-timestamp TF lookup succeeded',
            )
        if (
            self.tf_initial_misses
            > int(self.arguments.maximum_initial_tf_misses)
        ):
            self.record_failure(
                'tf_lookup',
                f'initial TF misses {self.tf_initial_misses} exceed '
                f'{self.arguments.maximum_initial_tf_misses}',
            )
        if (
            self.tf_post_success_failures
            > int(self.arguments.maximum_post_ready_tf_failures)
        ):
            self.record_failure(
                'tf_lookup',
                f'post-ready TF failures {self.tf_post_success_failures} '
                f'exceed {self.arguments.maximum_post_ready_tf_failures}',
            )
        if (
            self.tf_max_consecutive_failures
            > int(self.arguments.maximum_consecutive_tf_failures)
        ):
            self.record_failure(
                'tf_lookup',
                f'maximum consecutive TF failures '
                f'{self.tf_max_consecutive_failures} exceed '
                f'{self.arguments.maximum_consecutive_tf_failures}',
            )

        for stream_name in ('raw', 'output'):
            ages = self.stamp_ages[stream_name]
            maximum_absolute_age = (
                max(abs(value) for value in ages)
                if ages
                else None
            )
            if (
                maximum_absolute_age is None
                or maximum_absolute_age
                > float(self.arguments.maximum_stamp_age_s)
            ):
                self.record_failure(
                    'timestamp_age',
                    f'{stream_name}: maximum absolute stamp age '
                    f'{maximum_absolute_age!r} s exceeds '
                    f'{self.arguments.maximum_stamp_age_s} s',
                )

        raw_stamps = set(self.message_stamps['raw'])
        first_raw_stamp = (
            self.message_stamps['raw'][0]
            if self.message_stamps['raw']
            else None
        )
        output_stamps = set(self.message_stamps['output'])
        eligible_output = {
            stamp
            for stamp in output_stamps
            if first_raw_stamp is None or stamp >= first_raw_stamp
        }
        matched_output = eligible_output & raw_stamps
        match_ratio = (
            len(matched_output) / len(eligible_output)
            if eligible_output
            else 0.0
        )
        minimum_matched = int(
            self.arguments.minimum_matched_output_stamps
        )
        minimum_ratio = float(
            self.arguments.minimum_timestamp_match_ratio
        )
        if len(matched_output) < minimum_matched:
            self.record_failure(
                'timestamp_match',
                f'only {len(matched_output)} filtered stamps matched an '
                f'observed raw input stamp; {minimum_matched} required',
            )
        if match_ratio < minimum_ratio:
            self.record_failure(
                'timestamp_match',
                f'filtered/raw timestamp match ratio {match_ratio:.6f} is '
                f'below {minimum_ratio:.6f}',
            )

    def _validate_configured_auxiliary_rates(self, stream_report):
        """Require status and heartbeat rates near their configured values."""
        tolerance = float(
            self.arguments.auxiliary_rate_relative_tolerance
        )
        expectations = {
            'status': float(self.config['status_publish_hz']),
            'heartbeat': float(self.config['heartbeat_hz']),
        }

        for stream_name, expected_rate in expectations.items():
            measured_rate = stream_report[stream_name]['average_rate_hz']
            minimum_rate = expected_rate * (1.0 - tolerance)
            maximum_rate = expected_rate * (1.0 + tolerance)

            if (
                measured_rate is None
                or measured_rate < minimum_rate
                or (
                    stream_name == 'heartbeat'
                    and measured_rate > maximum_rate
                )
            ):
                expected_description = (
                    f'{expected_rate} Hz within '
                    f'{tolerance * 100.0:.1f}%'
                    if stream_name == 'heartbeat'
                    else f'at least {minimum_rate} Hz'
                )
                self.record_failure(
                    'configured_stream_rate',
                    f'{stream_name}: measured {measured_rate!r} Hz, '
                    f'expected {expected_description}',
                )

    def _validate_outage(self):
        """Validate the no-input stale phase while the filter stays alive."""
        if self.receive_times['raw']:
            self.record_failure(
                'stale_raw_quiet',
                f"observed {len(self.receive_times['raw'])} raw clouds "
                'during the outage window',
            )
        if self.receive_times['output']:
            self.record_failure(
                'stale_output_quiet',
                f"observed {len(self.receive_times['output'])} filtered "
                'clouds during the outage window',
            )

        for stream_name, minimum_count in (
            ('health', int(self.arguments.minimum_status_messages)),
            ('status', int(self.arguments.minimum_status_messages)),
            ('heartbeat', int(self.arguments.minimum_heartbeat_messages)),
        ):
            observed = len(self.receive_times[stream_name])
            if observed < minimum_count:
                self.record_failure(
                    'stale_message_count',
                    f'{stream_name}: expected at least {minimum_count}, '
                    f'observed {observed}',
                )

        if not self.health_values or self.health_values[-1] is not False:
            self.record_failure(
                'stale_health',
                f'latest outage health is not false: '
                f'{self.health_values[-1] if self.health_values else None!r}',
            )

        if not self.status_documents:
            self.record_failure(
                'stale_status',
                'no valid status was observed during the outage',
            )
        else:
            latest = self.status_documents[-1]
            if latest.get('state') != 'stale':
                self.record_failure(
                    'stale_status',
                    f"latest outage state is {latest.get('state')!r}",
                )

            age_seconds = latest.get('age_seconds')
            if (
                not isinstance(age_seconds, (int, float))
                or not math.isfinite(float(age_seconds))
                or float(age_seconds) <= float(self.config['stale_timeout_s'])
            ):
                self.record_failure(
                    'stale_status',
                    f'outage age_seconds is not beyond stale timeout: '
                    f'{age_seconds!r}',
                )

        if len(self.heartbeat_counters) < 2:
            self.record_failure(
                'stale_heartbeat',
                'heartbeat did not continue through the outage window',
            )

    def _scene_report(self):
        """Summarize and validate the selected controlled physical scene."""
        def aggregate_observations(observations):
            """Aggregate per-stamp ROI geometry without retaining point arrays."""
            values = list(observations.values())
            populated = [
                value
                for value in values
                if int(value['count']) > 0
            ]
            total_points = sum(int(value['count']) for value in values)

            if not populated:
                return {
                    'samples': len(values),
                    'populated_samples': 0,
                    'total_points': total_points,
                    'minimum_xyz': None,
                    'maximum_xyz': None,
                    'weighted_centroid_xyz': None,
                }

            minimum_xyz = [
                min(float(value['minimum_xyz'][axis]) for value in populated)
                for axis in range(3)
            ]
            maximum_xyz = [
                max(float(value['maximum_xyz'][axis]) for value in populated)
                for axis in range(3)
            ]
            weighted_centroid_xyz = [
                (
                    sum(
                        float(value['centroid_xyz'][axis])
                        * int(value['count'])
                        for value in populated
                    )
                    / total_points
                )
                for axis in range(3)
            ]
            return {
                'samples': len(values),
                'populated_samples': len(populated),
                'total_points': total_points,
                'minimum_xyz': minimum_xyz,
                'maximum_xyz': maximum_xyz,
                'weighted_centroid_xyz': weighted_centroid_xyz,
            }

        report = {
            'selected': self.arguments.scene,
            'minimum_required_points': int(
                self.arguments.minimum_scene_points
            ),
            'minimum_required_pairs': int(
                self.arguments.minimum_scene_pairs
            ),
            'regions': {
                source: {
                    name: summarize_integer_values(
                        self.scene_counts[source][name]
                    )
                    for name in SCENE_NAMES
                }
                for source in ('raw', 'output')
            },
            'automated_interpretation': None,
            'operator_confirmation_required': True,
            'raw_self_body_extents_m': {
                'measurement': 'points inside configured self-filter box',
                'contributing_points': self.raw_self_extent_points,
                'minimum_xyz': (
                    [
                        float(value)
                        for value in self.raw_self_extent_min
                    ]
                    if self.raw_self_extent_min is not None
                    else None
                ),
                'maximum_xyz': (
                    [
                        float(value)
                        for value in self.raw_self_extent_max
                    ]
                    if self.raw_self_extent_max is not None
                    else None
                ),
            },
            'raw_body_candidate_extents_m': {
                'measurement': (
                    'operator-cleared broad chassis candidate envelope'
                ),
                'envelope': {
                    axis: [float(bounds[0]), float(bounds[1])]
                    for axis, bounds in BODY_CANDIDATE_ENVELOPE.items()
                },
                'contributing_points': (
                    self.raw_body_candidate_extent_points
                ),
                'minimum_xyz': (
                    [
                        float(value)
                        for value in self.raw_body_candidate_extent_min
                    ]
                    if self.raw_body_candidate_extent_min is not None
                    else None
                ),
                'maximum_xyz': (
                    [
                        float(value)
                        for value in self.raw_body_candidate_extent_max
                    ]
                    if self.raw_body_candidate_extent_max is not None
                    else None
                ),
            },
        }

        if self.arguments.mode != 'scene':
            return report

        scene = str(self.arguments.scene)
        minimum_points = int(self.arguments.minimum_scene_points)
        minimum_pairs = int(self.arguments.minimum_scene_pairs)
        raw_summary = report['regions']['raw'][scene]
        raw_observations = self.scene_observations['raw'][scene]
        output_observations = self.scene_observations['output'][scene]
        paired_stamps = sorted(
            set(raw_observations) & set(output_observations)
        )
        paired_evidence = [
            {
                'stamp_ns': stamp_ns,
                'raw': raw_observations[stamp_ns],
                'output': output_observations[stamp_ns],
                'output_to_raw_count_ratio': (
                    float(output_observations[stamp_ns]['count'])
                    / float(raw_observations[stamp_ns]['count'])
                    if int(raw_observations[stamp_ns]['count']) > 0
                    else None
                ),
            }
            for stamp_ns in paired_stamps
        ]
        report['selected_roi'] = {
            'raw': aggregate_observations(raw_observations),
            'output': aggregate_observations(output_observations),
        }
        report['paired_roi_evidence'] = {
            'pairs': len(paired_stamps),
            'observations': paired_evidence,
        }

        if len(paired_stamps) < minimum_pairs:
            self.record_failure(
                'scene_paired_evidence',
                f'{scene}: expected at least {minimum_pairs} same-stamp '
                f'raw/output ROI pairs, observed {len(paired_stamps)}',
            )

        if raw_summary['samples'] <= 0:
            self.record_failure(
                'scene_raw_evidence',
                f'{scene}: no transformed raw frames were analyzed',
            )
            return report

        retained_scenes = {
            'front',
            'left',
            'right',
            'normal',
            'mid',
            'self_nearby',
        }
        rejected_scenes = {
            'low',
            'high',
            'near',
            'far',
            'self',
            'self_body',
        }

        if scene in retained_scenes:
            qualifying_pairs = sum(
                int(pair['raw']['count']) >= minimum_points
                and int(pair['output']['count']) >= minimum_points
                for pair in paired_evidence
            )
            report['paired_roi_evidence']['qualifying_pairs'] = (
                qualifying_pairs
            )
            if qualifying_pairs < minimum_pairs:
                self.record_failure(
                    'scene_retention',
                    f'{scene}: only {qualifying_pairs} same-stamp pairs had '
                    f'at least {minimum_points} raw and filtered ROI points; '
                    f'{minimum_pairs} required',
                )
            report['automated_interpretation'] = (
                'selected region retention was measured in same-stamp raw '
                'and filtered clouds'
            )
        elif scene in rejected_scenes:
            qualifying_pairs = sum(
                int(pair['raw']['count']) >= minimum_points
                and int(pair['output']['count']) == 0
                for pair in paired_evidence
            )
            report['paired_roi_evidence']['qualifying_pairs'] = (
                qualifying_pairs
            )
            if qualifying_pairs < minimum_pairs:
                self.record_failure(
                    'scene_rejection',
                    f'{scene}: only {qualifying_pairs} same-stamp pairs had '
                    f'at least {minimum_points} raw ROI points and zero '
                    f'filtered ROI points; {minimum_pairs} required',
                )
            report['automated_interpretation'] = (
                'selected rejected region removal was measured in same-stamp '
                'raw and filtered clouds'
            )

            if scene == 'self':
                nearby_raw = self.scene_observations['raw'][
                    'self_nearby'
                ]
                nearby_output = self.scene_observations['output'][
                    'self_nearby'
                ]
                nearby_stamps = sorted(
                    set(nearby_raw) & set(nearby_output)
                )
                nearby_evidence = [
                    {
                        'stamp_ns': stamp_ns,
                        'raw': nearby_raw[stamp_ns],
                        'output': nearby_output[stamp_ns],
                    }
                    for stamp_ns in nearby_stamps
                ]
                nearby_qualifying = sum(
                    int(pair['raw']['count']) >= minimum_points
                    and int(pair['output']['count']) >= minimum_points
                    for pair in nearby_evidence
                )
                report['self_nearby_obstacle_retention'] = {
                    'region': (
                        'configured self box expanded by '
                        'x=(-0.15,+0.35), y=0.20, z=(-0.10,+0.25) m'
                    ),
                    'raw': aggregate_observations(nearby_raw),
                    'output': aggregate_observations(nearby_output),
                    'same_stamp_pairs': len(nearby_stamps),
                    'qualifying_pairs': nearby_qualifying,
                    'minimum_required_pairs': minimum_pairs,
                    'minimum_required_points': minimum_points,
                    'observations': nearby_evidence,
                }

                if nearby_qualifying < minimum_pairs:
                    self.record_failure(
                        'self_nearby_retention',
                        f'self: only {nearby_qualifying} same-stamp pairs '
                        'retained at least '
                        f'{minimum_points} points from the controlled '
                        'legitimate obstacle immediately outside the '
                        f'self-filter box; {minimum_pairs} required',
                    )
        elif scene == 'clear':
            output_counts = self.point_counts['output']
            if output_counts:
                average = statistics.fmean(output_counts)
                coefficient = (
                    statistics.pstdev(output_counts) / average
                    if len(output_counts) > 1 and average > 0.0
                    else 0.0
                )
                report['clear_output_count_cv'] = float(coefficient)
                if coefficient > float(self.arguments.maximum_clear_count_cv):
                    self.record_failure(
                        'clear_scene_stability',
                        f'clear-scene output count CV {coefficient} exceeds '
                        f'{self.arguments.maximum_clear_count_cv}',
                    )

                maximum_allowed = int(
                    self.arguments.maximum_clear_output_points
                )
                if (
                    maximum_allowed >= 0
                    and max(output_counts) > maximum_allowed
                ):
                    self.record_failure(
                        'clear_scene_density',
                        f'clear-scene output maximum {max(output_counts)} '
                        f'exceeds {maximum_allowed}',
                    )

            report['automated_interpretation'] = (
                'filtered stream remained contract-valid and point-count '
                'stability was measured'
            )

        return report

    def build_report(self, started_at, elapsed_seconds):
        """Finalize validation and return a complete JSON-safe report."""
        stream_names = (
            'raw',
            'output',
            'health',
            'status',
            'heartbeat',
        )
        stream_report = {
            name: summarize_stream(self.receive_times[name])
            for name in stream_names
        }

        outage = (
            self.arguments.mode == 'stale'
            and self.arguments.stale_phase == 'outage'
        )

        if outage:
            self._validate_outage()
        else:
            self._validate_normal_streams(stream_report)

        self._validate_configured_auxiliary_rates(stream_report)
        graph_report = self._graph_report()
        tf_chain_report = self._tf_chain_report() if not outage else {
            'source_frame': None,
            'target_frame': normalized_frame(self.config['output_frame']),
            'source_to_target': [],
            'target_to_source': [],
            'frame_metadata': {},
            'conflicting_static_transforms': [],
            'multiple_static_parents': [],
            'duplicate_static_publishers': [],
            'not_evaluated_reason': 'raw publisher intentionally stopped',
        }
        scene_report = self._scene_report()

        latency_values = []
        for stamp_ns, output_received in self.output_receive_by_stamp.items():
            raw_received = self.raw_receive_by_stamp.get(stamp_ns)
            if raw_received is not None:
                latency_values.append(
                    (output_received - raw_received) * 1000.0
                )

        first_raw_stamp = (
            self.message_stamps['raw'][0]
            if self.message_stamps['raw']
            else None
        )
        raw_stamp_set = set(self.message_stamps['raw'])
        output_stamp_set = set(self.message_stamps['output'])
        ignored_pre_observation_output_stamps = {
            stamp
            for stamp in output_stamp_set
            if first_raw_stamp is not None and stamp < first_raw_stamp
        }
        unmatched_observation_output_stamps = {
            stamp
            for stamp in output_stamp_set - raw_stamp_set
            if first_raw_stamp is None or stamp >= first_raw_stamp
        }
        eligible_observation_output_stamps = {
            stamp
            for stamp in output_stamp_set
            if first_raw_stamp is None or stamp >= first_raw_stamp
        }
        matched_observation_output_stamps = (
            eligible_observation_output_stamps & raw_stamp_set
        )
        timestamp_match_ratio = (
            len(matched_observation_output_stamps)
            / len(eligible_observation_output_stamps)
            if eligible_observation_output_stamps
            else 0.0
        )

        stamp_age_report = {
            name: {
                'samples': len(self.stamp_ages[name]),
                'minimum_s': safe_min(self.stamp_ages[name]),
                'maximum_s': safe_max(self.stamp_ages[name]),
                'average_s': safe_mean(self.stamp_ages[name]),
                'maximum_absolute_s': (
                    safe_max(
                        [
                            abs(value)
                            for value in self.stamp_ages[name]
                        ]
                    )
                ),
            }
            for name in ('raw', 'output')
        }

        tf_report = {
            'attempts': self.tf_attempts,
            'successes': self.tf_successes,
            'initial_misses': self.tf_initial_misses,
            'post_success_failures': self.tf_post_success_failures,
            'maximum_consecutive_failures': (
                self.tf_max_consecutive_failures
            ),
            'failure_messages': list(self.tf_failure_messages),
            'samples': list(self.tf_samples),
            'chain': tf_chain_report,
        }

        latest_status = (
            self.status_documents[-1]
            if self.status_documents
            else None
        )
        latest_heartbeat = (
            self.heartbeat_documents[-1]
            if self.heartbeat_documents
            else None
        )
        average_raw_points = (
            safe_mean(self.point_counts['raw'])
        )
        average_output_points = (
            safe_mean(self.point_counts['output'])
        )
        reduction_percentage = None
        if (
            average_raw_points is not None
            and average_raw_points > 0.0
            and average_output_points is not None
        ):
            reduction_percentage = (
                100.0
                * (average_raw_points - average_output_points)
                / average_raw_points
            )

        report = {
            'schema': 'robot_savo.phase8a5.d435_hardware_fixture.v1',
            'fixture': {
                'node_name': '/' + self.get_name(),
                'subscriber_only': True,
                'application_publishers_created': 0,
                'mode': self.arguments.mode,
                'scene': self.arguments.scene,
                'stale_phase': self.arguments.stale_phase,
                'started_utc': started_at,
                'elapsed_s': float(elapsed_seconds),
            },
            'configuration': {
                'path': str(self.arguments.config),
                'parameters': dict(self.config),
            },
            'streams': stream_report,
            'point_counts': {
                'raw': summarize_integer_values(self.point_counts['raw']),
                'output': summarize_integer_values(
                    self.point_counts['output']
                ),
                'raw_frames_analyzed': self.raw_messages_analyzed,
                'output_messages_fully_validated': (
                    self.output_messages_validated
                ),
                'reduction_percentage': reduction_percentage,
            },
            'frames': {
                'raw_discovered': sorted(self.frames['raw']),
                'output_observed': sorted(self.frames['output']),
                'configured_output': normalized_frame(
                    self.config['output_frame']
                ),
            },
            'timestamps': {
                'raw_unique': len(set(self.message_stamps['raw'])),
                'output_unique': len(set(self.message_stamps['output'])),
                'eligible_output_stamps': len(
                    eligible_observation_output_stamps
                ),
                'matched_output_stamps': len(
                    matched_observation_output_stamps
                ),
                'unmatched_output_stamps': len(
                    unmatched_observation_output_stamps
                ),
                'match_ratio': float(timestamp_match_ratio),
                'minimum_required_matches': int(
                    self.arguments.minimum_matched_output_stamps
                ),
                'minimum_required_match_ratio': float(
                    self.arguments.minimum_timestamp_match_ratio
                ),
                'ignored_pre_observation_output_stamps': len(
                    ignored_pre_observation_output_stamps
                ),
                'receipt_latency_ms': {
                    'samples': len(latency_values),
                    'minimum': safe_min(latency_values),
                    'maximum': safe_max(latency_values),
                    'average': safe_mean(latency_values),
                    'p95': percentile(latency_values, 95.0),
                },
                'stamp_age': stamp_age_report,
            },
            'tf': tf_report,
            'status': {
                'health_samples': len(self.health_values),
                'latest_health': (
                    self.health_values[-1]
                    if self.health_values
                    else None
                ),
                'status_samples': len(self.status_documents),
                'latest_status': latest_status,
                'heartbeat_samples': len(self.heartbeat_documents),
                'latest_heartbeat': latest_heartbeat,
                'first_heartbeat_counter': (
                    self.heartbeat_counters[0]
                    if self.heartbeat_counters
                    else None
                ),
                'last_heartbeat_counter': (
                    self.heartbeat_counters[-1]
                    if self.heartbeat_counters
                    else None
                ),
            },
            'scene': scene_report,
            'graph': graph_report,
            'validation': {
                'result': 'passed' if not self.failures else 'failed',
                'failure_count': len(self.failures),
                'failures': list(self.failures),
            },
        }

        report['raw_frame'] = (
            sorted(self.frames['raw'])[0]
            if len(self.frames['raw']) == 1
            else None
        )
        report['tf_chain'] = list(
            tf_chain_report.get('target_to_source', [])
        )
        report['rate_metrics'] = stream_report
        report['count_metrics'] = report['point_counts']
        report['latency_metrics'] = report['timestamps'][
            'receipt_latency_ms'
        ]
        report['status_metrics'] = report['status']
        report['scene_metrics'] = scene_report
        return report


def parse_arguments():
    """Parse the bounded hardware-observer mode and validation thresholds."""
    parser = argparse.ArgumentParser(
        description=(
            'Observe and validate real D435 and production obstacle-cloud '
            'messages without creating any ROS publisher.'
        )
    )
    parser.add_argument(
        'mode',
        choices=('observe', 'scene', 'stale'),
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=DEFAULT_CONFIG,
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=10.0,
    )
    parser.add_argument(
        '--scene',
        choices=SCENE_NAMES,
    )
    parser.add_argument(
        '--stale-phase',
        choices=('baseline', 'outage', 'recovery'),
        default='baseline',
    )
    parser.add_argument(
        '--report',
        '--report-file',
        dest='report_file',
        type=Path,
    )
    parser.add_argument(
        '--raw-topic',
        default=DEFAULT_RAW_TOPIC,
    )
    parser.add_argument(
        '--output-topic',
        default=DEFAULT_OUTPUT_TOPIC,
    )
    parser.add_argument(
        '--health-topic',
        default=DEFAULT_HEALTH_TOPIC,
    )
    parser.add_argument(
        '--status-topic',
        default=DEFAULT_STATUS_TOPIC,
    )
    parser.add_argument(
        '--heartbeat-topic',
        default=DEFAULT_HEARTBEAT_TOPIC,
    )
    parser.add_argument(
        '--output-frame',
        default=DEFAULT_OUTPUT_FRAME,
    )
    parser.add_argument(
        '--expected-filter-node',
        default='obstacle_cloud_filter_node',
    )
    parser.add_argument(
        '--expected-raw-node',
        default='/camera/camera',
    )
    parser.add_argument(
        '--expected-tf-static-node',
        action='append',
        default=['/robot_state_publisher', '/camera/camera'],
    )
    parser.add_argument(
        '--raw-analysis-stride',
        type=int,
        default=10,
    )
    parser.add_argument(
        '--output-analysis-stride',
        type=int,
        default=5,
    )
    parser.add_argument(
        '--minimum-output-analysis-messages',
        type=int,
        default=5,
    )
    parser.add_argument(
        '--minimum-raw-messages',
        type=int,
        default=20,
    )
    parser.add_argument(
        '--minimum-output-messages',
        type=int,
        default=20,
    )
    parser.add_argument(
        '--minimum-matched-output-stamps',
        type=int,
        default=10,
    )
    parser.add_argument(
        '--minimum-timestamp-match-ratio',
        type=float,
        default=0.90,
    )
    parser.add_argument(
        '--minimum-status-messages',
        type=int,
        default=5,
    )
    parser.add_argument(
        '--minimum-heartbeat-messages',
        type=int,
        default=3,
    )
    parser.add_argument(
        '--minimum-raw-rate-hz',
        type=float,
        default=20.0,
    )
    parser.add_argument(
        '--minimum-output-rate-hz',
        type=float,
        default=20.0,
    )
    parser.add_argument(
        '--minimum-status-rate-hz',
        type=float,
        default=1.0,
    )
    parser.add_argument(
        '--minimum-heartbeat-rate-hz',
        type=float,
        default=0.5,
    )
    parser.add_argument(
        '--maximum-gap-s',
        type=float,
        default=0.50,
    )
    parser.add_argument(
        '--maximum-interval-cv',
        type=float,
        default=1.0,
    )
    parser.add_argument(
        '--auxiliary-rate-relative-tolerance',
        type=float,
        default=0.40,
        help='Relative tolerance for configured status and heartbeat rates.',
    )
    parser.add_argument(
        '--maximum-stamp-age-s',
        type=float,
        default=2.0,
    )
    parser.add_argument(
        '--maximum-initial-tf-misses',
        type=int,
        default=60,
    )
    parser.add_argument(
        '--maximum-post-ready-tf-failures',
        type=int,
        default=0,
    )
    parser.add_argument(
        '--maximum-consecutive-tf-failures',
        type=int,
        default=60,
    )
    parser.add_argument(
        '--minimum-scene-points',
        type=int,
        default=20,
    )
    parser.add_argument(
        '--minimum-scene-pairs',
        type=int,
        default=3,
    )
    parser.add_argument(
        '--maximum-clear-count-cv',
        type=float,
        default=1.0,
    )
    parser.add_argument(
        '--maximum-clear-output-points',
        type=int,
        default=-1,
        help='Disabled when negative; set only from a measured clear baseline.',
    )
    parser.add_argument(
        '--tolerance',
        type=float,
        default=1.0e-5,
    )
    parser.add_argument(
        '--maximum-failures',
        type=int,
        default=100,
    )
    parser.add_argument(
        '--forbidden-topic',
        action='append',
        default=list(DEFAULT_FORBIDDEN_TOPICS),
    )

    arguments = parser.parse_args()

    if arguments.duration <= 0.0:
        parser.error('--duration must be positive')
    if arguments.raw_analysis_stride <= 0:
        parser.error('--raw-analysis-stride must be positive')
    if arguments.output_analysis_stride <= 0:
        parser.error('--output-analysis-stride must be positive')
    if arguments.maximum_failures <= 0:
        parser.error('--maximum-failures must be positive')
    if arguments.tolerance < 0.0:
        parser.error('--tolerance must be nonnegative')
    if arguments.mode == 'scene' and arguments.scene is None:
        parser.error('--scene is required when mode is scene')
    if arguments.mode != 'scene' and arguments.scene is not None:
        parser.error('--scene is valid only when mode is scene')
    if (
        arguments.mode != 'stale'
        and arguments.stale_phase != 'baseline'
    ):
        parser.error('--stale-phase is valid only when mode is stale')

    for name in (
        'minimum_raw_messages',
        'minimum_output_messages',
        'minimum_matched_output_stamps',
        'minimum_output_analysis_messages',
        'minimum_status_messages',
        'minimum_heartbeat_messages',
        'minimum_scene_points',
        'minimum_scene_pairs',
        'maximum_initial_tf_misses',
        'maximum_post_ready_tf_failures',
        'maximum_consecutive_tf_failures',
    ):
        if int(getattr(arguments, name)) < 0:
            parser.error(f'--{name.replace("_", "-")} must be nonnegative')

    for name in (
        'minimum_raw_rate_hz',
        'minimum_output_rate_hz',
        'minimum_status_rate_hz',
        'minimum_heartbeat_rate_hz',
        'maximum_gap_s',
        'maximum_interval_cv',
        'auxiliary_rate_relative_tolerance',
        'maximum_stamp_age_s',
        'maximum_clear_count_cv',
        'minimum_timestamp_match_ratio',
    ):
        value = float(getattr(arguments, name))
        if not math.isfinite(value) or value < 0.0:
            parser.error(
                f'--{name.replace("_", "-")} must be finite and nonnegative'
            )

    if arguments.auxiliary_rate_relative_tolerance >= 1.0:
        parser.error(
            '--auxiliary-rate-relative-tolerance must be less than one'
        )
    if arguments.minimum_timestamp_match_ratio > 1.0:
        parser.error(
            '--minimum-timestamp-match-ratio must not exceed one'
        )

    arguments.config = arguments.config.expanduser().resolve()
    if arguments.report_file is not None:
        arguments.report_file = arguments.report_file.expanduser().resolve()

    arguments.forbidden_topic = sorted(
        {
            str(topic).strip()
            for topic in arguments.forbidden_topic
            if str(topic).strip()
        }
    )

    for name in (
        'raw_topic',
        'output_topic',
        'health_topic',
        'status_topic',
        'heartbeat_topic',
        'output_frame',
        'expected_raw_node',
    ):
        value = str(getattr(arguments, name)).strip()
        if not value:
            parser.error(f'--{name.replace("_", "-")} must not be empty')
        setattr(arguments, name, value)

    if not arguments.expected_raw_node.startswith('/'):
        arguments.expected_raw_node = '/' + arguments.expected_raw_node

    arguments.expected_tf_static_node = sorted(
        {
            node if str(node).startswith('/') else '/' + str(node)
            for node in arguments.expected_tf_static_node
            if str(node).strip()
        }
    )

    return arguments


def emit_report(report, report_file):
    """Print JSON and optionally write the same evidence document."""
    serialized = json.dumps(
        report,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    )
    print(serialized)

    if report_file is not None:
        report_file.parent.mkdir(parents=True, exist_ok=True)
        report_file.write_text(serialized + '\n', encoding='utf-8')


def main():
    """Run one bounded subscriber-only hardware observation."""
    arguments = parse_arguments()
    started_at = datetime.now(timezone.utc).isoformat()
    started_monotonic = time.monotonic()
    observer = None
    report = None

    try:
        configuration = load_filter_configuration(arguments.config)
        configuration['input_topic'] = arguments.raw_topic
        configuration['output_topic'] = arguments.output_topic
        configuration['health_topic'] = arguments.health_topic
        configuration['status_topic'] = arguments.status_topic
        configuration['heartbeat_topic'] = arguments.heartbeat_topic
        configuration['output_frame'] = arguments.output_frame
        rclpy.init(args=[])
        observer = HardwareObserver(arguments, configuration)
        observer.run_for_duration()
        report = observer.build_report(
            started_at,
            time.monotonic() - started_monotonic,
        )
    except (ValidationFailure, SubscriberOnlyViolation) as error:
        report = {
            'schema': 'robot_savo.phase8a5.d435_hardware_fixture.v1',
            'fixture': {
                'subscriber_only': True,
                'mode': arguments.mode,
                'scene': arguments.scene,
                'stale_phase': arguments.stale_phase,
                'started_utc': started_at,
                'elapsed_s': time.monotonic() - started_monotonic,
            },
            'validation': {
                'result': 'failed',
                'failure_count': 1,
                'failures': [
                    {
                        'code': 'fixture_setup',
                        'detail': str(error),
                    }
                ],
            },
        }
    except Exception as error:
        report = {
            'schema': 'robot_savo.phase8a5.d435_hardware_fixture.v1',
            'fixture': {
                'subscriber_only': True,
                'mode': arguments.mode,
                'scene': arguments.scene,
                'stale_phase': arguments.stale_phase,
                'started_utc': started_at,
                'elapsed_s': time.monotonic() - started_monotonic,
            },
            'validation': {
                'result': 'failed',
                'failure_count': 1,
                'failures': [
                    {
                        'code': 'fixture_runtime',
                        'detail': (
                            f'{type(error).__name__}: {error}'
                        ),
                    }
                ],
            },
        }
    except KeyboardInterrupt:
        report = {
            'schema': 'robot_savo.phase8a5.d435_hardware_fixture.v1',
            'fixture': {
                'subscriber_only': True,
                'mode': arguments.mode,
                'scene': arguments.scene,
                'stale_phase': arguments.stale_phase,
                'started_utc': started_at,
                'elapsed_s': time.monotonic() - started_monotonic,
            },
            'validation': {
                'result': 'failed',
                'failure_count': 1,
                'failures': [
                    {
                        'code': 'interrupted',
                        'detail': 'hardware observation was interrupted',
                    }
                ],
            },
        }
    finally:
        if observer is not None:
            observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    emit_report(report, arguments.report_file)
    return 0 if report['validation']['result'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
