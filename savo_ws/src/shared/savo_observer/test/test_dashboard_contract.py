"""Validate the responsive read-only browser dashboard."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_dashboard_assets_and_mobile_layout_exist():
    web = ROOT / 'dashboard/web'
    for name in ('index.html', 'styles.css', 'app.js'):
        assert (web / name).stat().st_size > 0
    mobile = yaml.safe_load(
        (ROOT / 'dashboard/layouts/mobile.yaml').read_text(encoding='utf-8')
    )['layout']
    assert mobile['responsive'] is True
    assert mobile['raw_images'] is False
    assert mobile['raw_pointclouds'] is False


def test_dashboard_is_bounded_and_has_clear_stale_state():
    script = (ROOT / 'dashboard/web/app.js').read_text(encoding='utf-8')
    styles = (ROOT / 'dashboard/web/styles.css').read_text(encoding='utf-8')
    assert 'historyCapacity = 120' in script
    assert 'values.shift()' in script
    assert "fetch('/api/config'" in script
    assert 'renderMetricGraph' in script
    assert 'JSON.parse(dependency.detail)' in script
    assert 'Connection lost' in script
    assert '.card.stale' in styles


def test_dashboard_allowlist_is_read_only():
    dashboard = yaml.safe_load(
        (ROOT / 'config/dashboard.yaml').read_text(encoding='utf-8')
    )['dashboard']
    assert dashboard['read_only'] is True
    assert all(route.startswith('/') for route in dashboard['allowed_routes'])
    assert '/api/telemetry' in dashboard['allowed_routes']
    assert '/api/config' in dashboard['allowed_routes']
    assert all('cmd' not in route and 'goal' not in route for route in dashboard['allowed_routes'])


def test_profiles_bound_dashboard_rate_and_history():
    profiles = ROOT / 'config/profiles'
    for name in ('low_bandwidth', 'standard', 'full_debug', 'mobile'):
        document = yaml.safe_load(
            (profiles / f'{name}.yaml').read_text(encoding='utf-8')
        )
        parameters = document['/observer_dashboard_node']['ros__parameters']
        assert 250 <= parameters['polling_interval_ms'] <= 10000
        assert 10 <= parameters['history_capacity'] <= 1000
