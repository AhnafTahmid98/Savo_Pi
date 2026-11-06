"""Ensure STARTUP-ONLY recovery cannot weaken motion or localization limits."""

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_transient_ekf_grace_is_fail_closed_and_staged_only():
    """Verify bounded startup recovery remains fail-closed."""
    node = (ROOT / 'src/nodes/bringup_readiness_node.cpp').read_text()
    config = yaml.safe_load((ROOT / 'config/startup_stages.yaml').read_text())
    assert 'EligibleTransientEkfStartupLoss' in node
    assert 'control_stop' in node
    assert 'safety_clear' in node
    assert 'IsTransientEkfRateLoss' in node
    assert 'startup.core.transient_ekf_rate_recovery_s' in node
    assert 'input.dependencies_ready = false' in node
    assert 'input.unrecoverable_failure = recovery.expired' in node
    assert config['bringup_readiness_node']['ros__parameters'][
        'startup.core.transient_ekf_rate_recovery_s'] == 12.0
    assert 'transient_ekf_recovery_.Update(' in node


def test_localization_minimums_are_unchanged():
    """Verify the EKF minimum rate is unchanged."""
    health = (ROOT.parents[1] / 'core/savo_localization/config/localization_health.yaml')
    if health.exists():
        settings = yaml.safe_load(health.read_text())
        assert settings['localization_health_node']['ros__parameters'][
            'ekf_min_rate_hz'] == 10.0
