"""File-based configuration tests (no ROS 2 import required)."""
from pathlib import Path

import pytest
import yaml


def test_example_yaml_has_consistent_active_agents():
    config = Path(__file__).resolve().parents[1] / 'config' / 'cyclic.yaml'
    data = yaml.safe_load(config.read_text(encoding='utf-8'))
    params = data['params']['ros__parameters']
    names, ids = params['cfs_active_names'], params['cfs_active_id']
    assert len(names) == len(ids) == params['n']
    assert len(set(names)) == len(names)
    assert len(set(ids)) == len(ids)
