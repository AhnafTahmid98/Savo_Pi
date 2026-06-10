"""Convert VO status models into ROS diagnostic messages."""

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from savo_vo.models.vo_status import VOState, VOStatus


def diagnostic_level_from_state(state: VOState) -> int:
    if state == VOState.OK:
        return DiagnosticStatus.OK

    if state == VOState.DEGRADED:
        return DiagnosticStatus.WARN

    return DiagnosticStatus.ERROR


def status_to_diagnostic_msg(
    status: VOStatus,
    name: str = "savo_vo",
    hardware_id: str = "savo-edge/realsense",
) -> DiagnosticStatus:
    msg = DiagnosticStatus()
    msg.level = diagnostic_level_from_state(status.state)
    msg.name = name
    msg.hardware_id = hardware_id
    msg.message = status.message or status.state.value

    msg.values = [
        KeyValue(key="state", value=status.state.value),
        KeyValue(key="tracking_quality", value=f"{status.tracking_quality:.3f}"),
        KeyValue(key="feature_count", value=str(status.feature_count)),
        KeyValue(key="age_s", value=f"{status.age_s:.3f}"),
        KeyValue(key="usable", value=str(status.is_usable).lower()),
        KeyValue(key="healthy", value=str(status.is_healthy).lower()),
    ]

    return msg