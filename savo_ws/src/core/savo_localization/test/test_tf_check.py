#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for TF health checks used by Robot Savo localization."""

from __future__ import annotations

import pytest

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_IMU,
    FRAME_MAP,
    FRAME_ODOM,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
)
from savo_localization.diagnostics.tf_check import (
    TfChainCheckResult,
    TfEdgeCheckResult,
    check_tf_chain,
    check_tf_edge,
    make_tf_edge_result,
    tf_chain_summary,
    tf_edge_label,
)


def test_tf_edge_label() -> None:
    assert tf_edge_label(FRAME_ODOM, FRAME_BASE_LINK) == "odom -> base_link"
    assert tf_edge_label(FRAME_BASE_LINK, FRAME_IMU) == "base_link -> imu_link"
    assert tf_edge_label("/odom", "/base_link") == "odom -> base_link"


def test_make_tf_edge_result_available_and_fresh() -> None:
    result = make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=True,
        age_s=0.05,
        max_age_s=0.5,
    )

    assert result.parent_frame == FRAME_ODOM
    assert result.child_frame == FRAME_BASE_LINK
    assert result.label == "odom -> base_link"
    assert result.available is True
    assert result.fresh is True
    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.age_s == pytest.approx(0.05)
    assert result.reasons == []


def test_make_tf_edge_result_available_but_stale() -> None:
    result = make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=True,
        age_s=1.0,
        max_age_s=0.5,
    )

    assert result.available is True
    assert result.fresh is False
    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.reasons
    assert any("stale" in reason.lower() for reason in result.reasons)


def test_make_tf_edge_result_missing() -> None:
    result = make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=False,
        age_s=None,
        max_age_s=0.5,
        message="transform not available",
    )

    assert result.available is False
    assert result.fresh is False
    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.message == "transform not available"
    assert result.reasons


def test_make_tf_edge_result_rejects_bad_age_limit() -> None:
    with pytest.raises(ValueError):
        make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=0.1,
            max_age_s=0.0,
        )


def test_make_tf_edge_result_rejects_bad_frame_pair() -> None:
    with pytest.raises(ValueError):
        make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=0.1,
            max_age_s=0.5,
        )


def test_tf_edge_check_result_to_dict() -> None:
    result = TfEdgeCheckResult(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=True,
        fresh=True,
        age_s=0.05,
        status=STATUS_OK,
        message="TF healthy",
        reasons=[],
    )

    data = result.to_dict()

    assert data["parent_frame"] == FRAME_ODOM
    assert data["child_frame"] == FRAME_BASE_LINK
    assert data["label"] == "odom -> base_link"
    assert data["available"] is True
    assert data["fresh"] is True
    assert data["ok"] is True
    assert data["age_s"] == pytest.approx(0.05)
    assert data["status"] == STATUS_OK
    assert data["message"] == "TF healthy"
    assert data["reasons"] == []


def test_check_tf_edge_healthy() -> None:
    edge = make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=True,
        age_s=0.05,
        max_age_s=0.5,
    )

    result = check_tf_edge(edge)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.available is True
    assert result.fresh is True
    assert result.reasons == []


def test_check_tf_edge_stale() -> None:
    edge = make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=True,
        age_s=1.0,
        max_age_s=0.5,
    )

    result = check_tf_edge(edge)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.available is True
    assert result.fresh is False
    assert result.reasons


def test_check_tf_edge_missing() -> None:
    edge = make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=False,
        age_s=None,
        max_age_s=0.5,
    )

    result = check_tf_edge(edge)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.available is False
    assert result.fresh is False
    assert result.reasons


def test_tf_chain_check_result_to_dict() -> None:
    odom_to_base = make_tf_edge_result(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
        available=True,
        age_s=0.05,
        max_age_s=0.5,
    )

    base_to_imu = make_tf_edge_result(
        parent_frame=FRAME_BASE_LINK,
        child_frame=FRAME_IMU,
        available=True,
        age_s=0.05,
        max_age_s=0.5,
    )

    result = TfChainCheckResult(
        status=STATUS_OK,
        ok=True,
        message="TF chain healthy",
        reasons=[],
        edges={
            "odom_to_base": odom_to_base,
            "base_to_imu": base_to_imu,
        },
    )

    data = result.to_dict()

    assert data["status"] == STATUS_OK
    assert data["ok"] is True
    assert data["message"] == "TF chain healthy"
    assert data["reasons"] == []
    assert data["checked_count"] == 2
    assert data["healthy_count"] == 2
    assert data["available_count"] == 2
    assert data["edges"]["odom_to_base"]["ok"] is True
    assert data["edges"]["base_to_imu"]["ok"] is True


def test_check_tf_chain_healthy_baseline() -> None:
    edges = {
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
    }

    result = check_tf_chain(edges)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.checked_count == 2
    assert result.healthy_count == 2
    assert result.available_count == 2
    assert result.reasons == []


def test_check_tf_chain_with_optional_map_to_odom() -> None:
    edges = {
        "map_to_odom": make_tf_edge_result(
            parent_frame=FRAME_MAP,
            child_frame=FRAME_ODOM,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
    }

    result = check_tf_chain(edges)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.checked_count == 3
    assert result.healthy_count == 3
    assert result.available_count == 3


def test_check_tf_chain_missing_required_edge() -> None:
    edges = {
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=False,
            age_s=None,
            max_age_s=0.5,
            message="transform not available",
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
    }

    result = check_tf_chain(edges)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.checked_count == 2
    assert result.healthy_count == 1
    assert result.available_count == 1
    assert result.reasons


def test_check_tf_chain_stale_required_edge() -> None:
    edges = {
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=1.0,
            max_age_s=0.5,
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
    }

    result = check_tf_chain(edges)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.checked_count == 2
    assert result.healthy_count == 1
    assert result.available_count == 2
    assert any("stale" in reason.lower() for reason in result.reasons)


def test_check_tf_chain_empty_edges() -> None:
    result = check_tf_chain({})

    assert result.ok is False
    assert result.status == STATUS_WARN
    assert result.checked_count == 0
    assert result.healthy_count == 0
    assert result.available_count == 0
    assert result.reasons


def test_tf_chain_summary() -> None:
    edges = {
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=True,
            age_s=0.05,
            max_age_s=0.5,
        ),
    }

    summary = tf_chain_summary(edges)

    assert summary["status"] == STATUS_OK
    assert summary["ok"] is True
    assert summary["checked_count"] == 2
    assert summary["healthy_count"] == 2
    assert summary["available_count"] == 2
    assert "edges" in summary


def test_robot_savo_localization_tf_chain_is_healthy() -> None:
    edges = {
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=0.03,
            max_age_s=0.5,
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=True,
            age_s=0.03,
            max_age_s=0.5,
        ),
    }

    result = check_tf_chain(edges)

    assert result.ok is True
    assert result.status == STATUS_OK
    assert result.healthy_count == 2
    assert result.available_count == 2


def test_robot_savo_missing_odom_to_base_tf_is_not_usable() -> None:
    edges = {
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=False,
            age_s=None,
            max_age_s=0.5,
            message="EKF is not publishing odom -> base_link",
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=True,
            age_s=0.03,
            max_age_s=0.5,
        ),
    }

    result = check_tf_chain(edges)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.healthy_count == 1
    assert result.available_count == 1
    assert any("odom -> base_link" in reason for reason in result.reasons)


def test_robot_savo_missing_base_to_imu_tf_is_not_usable() -> None:
    edges = {
        "odom_to_base": make_tf_edge_result(
            parent_frame=FRAME_ODOM,
            child_frame=FRAME_BASE_LINK,
            available=True,
            age_s=0.03,
            max_age_s=0.5,
        ),
        "base_to_imu": make_tf_edge_result(
            parent_frame=FRAME_BASE_LINK,
            child_frame=FRAME_IMU,
            available=False,
            age_s=None,
            max_age_s=0.5,
            message="robot_state_publisher is not publishing base_link -> imu_link",
        ),
    }

    result = check_tf_chain(edges)

    assert result.ok is False
    assert result.status == STATUS_STALE
    assert result.healthy_count == 1
    assert result.available_count == 1
    assert any("base_link -> imu_link" in reason for reason in result.reasons)


def test_default_edge_result_values() -> None:
    result = TfEdgeCheckResult(
        parent_frame=FRAME_ODOM,
        child_frame=FRAME_BASE_LINK,
    )

    assert result.parent_frame == FRAME_ODOM
    assert result.child_frame == FRAME_BASE_LINK
    assert result.label == "odom -> base_link"
    assert result.available is False
    assert result.fresh is False
    assert result.ok is False
    assert result.status == STATUS_UNKNOWN


def test_default_chain_result_values() -> None:
    result = TfChainCheckResult()

    assert result.status == STATUS_UNKNOWN
    assert result.ok is False
    assert result.checked_count == 0
    assert result.healthy_count == 0
    assert result.available_count == 0
    assert result.edges == {}
