#!/usr/bin/env python3
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))

from waypoint_plan import load_waypoints, speed_limit_for, split_legs  # noqa: E402


def write_yaml(tmp_path, data):
    path = tmp_path / 'waypoints.yaml'
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)
    return path


# --- load_waypoints ---

def test_load_waypoints_normal(tmp_path):
    path = write_yaml(tmp_path, {
        'loop': True,
        'waypoints': [
            {'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'checkpoint': True, 'speed_limit': 50.0},
            {'x': 3.0, 'y': 4.0, 'yaw': 1.0, 'checkpoint': False, 'speed_limit': 80.0},
        ],
    })
    waypoints, loop = load_waypoints(path)
    assert loop is True
    assert waypoints == [
        {'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'checkpoint': True, 'speed_limit': 50.0},
        {'x': 3.0, 'y': 4.0, 'yaw': 1.0, 'checkpoint': False, 'speed_limit': 80.0},
    ]


def test_load_waypoints_fills_defaults(tmp_path):
    path = write_yaml(tmp_path, {
        'waypoints': [
            {'x': 1.0, 'y': 2.0},
        ],
    })
    waypoints, loop = load_waypoints(path)
    assert loop is False
    assert waypoints == [
        {'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'checkpoint': False, 'speed_limit': 0.0},
    ]


def test_load_waypoints_coerces_ints_to_float(tmp_path):
    # yaml の int がそのまま PoseStamped 代入に渡ると型エラーになる回帰を防ぐ
    path = write_yaml(tmp_path, {
        'waypoints': [
            {'x': 3, 'y': 4, 'yaw': 0, 'speed_limit': 50},
        ],
    })
    waypoints, _ = load_waypoints(path)
    wp = waypoints[0]
    assert isinstance(wp['x'], float) and wp['x'] == 3.0
    assert isinstance(wp['y'], float) and wp['y'] == 4.0
    assert isinstance(wp['yaw'], float) and wp['yaw'] == 0.0
    assert isinstance(wp['speed_limit'], float) and wp['speed_limit'] == 50.0


def test_load_waypoints_missing_x_raises(tmp_path):
    path = write_yaml(tmp_path, {
        'waypoints': [
            {'y': 2.0},
        ],
    })
    with pytest.raises(ValueError, match=r'waypoint 0') as exc_info:
        load_waypoints(path)
    assert "'x'" in str(exc_info.value)


def test_load_waypoints_missing_y_raises(tmp_path):
    path = write_yaml(tmp_path, {
        'waypoints': [
            {'x': 1.0, 'y': 2.0},
            {'x': 3.0},
        ],
    })
    with pytest.raises(ValueError, match=r'waypoint 1') as exc_info:
        load_waypoints(path)
    assert "'y'" in str(exc_info.value)


def test_load_waypoints_empty_list_raises(tmp_path):
    path = write_yaml(tmp_path, {'waypoints': []})
    with pytest.raises(ValueError):
        load_waypoints(path)


def test_load_waypoints_missing_key_raises(tmp_path):
    path = write_yaml(tmp_path, {'loop': False})
    with pytest.raises(ValueError):
        load_waypoints(path)


def test_load_waypoints_non_dict_entry_raises(tmp_path):
    path = write_yaml(tmp_path, {
        'waypoints': [
            {'x': 0.0, 'y': 0.0},
            3.0,
        ],
    })
    with pytest.raises(ValueError, match=r'waypoint 1'):
        load_waypoints(path)


# --- split_legs ---

def make_wp(i, checkpoint=False):
    # x を index 由来の値にして、レグ分割のアサーションが順序・対応を実際に検証できるようにする
    return {'x': float(i), 'y': 0.0, 'yaw': 0.0, 'checkpoint': checkpoint, 'speed_limit': 0.0}


def test_split_legs_no_checkpoint():
    waypoints = [make_wp(0), make_wp(1), make_wp(2)]
    legs = split_legs(waypoints)
    assert len(legs) == 1
    assert legs[0].start_index == 0
    assert legs[0].waypoints == waypoints
    assert legs[0].ends_with_checkpoint is False


def test_split_legs_middle_checkpoint():
    a, b, c, d, e = make_wp(0), make_wp(1, True), make_wp(2), make_wp(3, True), make_wp(4)
    legs = split_legs([a, b, c, d, e])
    assert len(legs) == 3
    assert legs[0].start_index == 0
    assert legs[0].waypoints == [a, b]
    assert legs[0].ends_with_checkpoint is True
    assert legs[1].start_index == 2
    assert legs[1].waypoints == [c, d]
    assert legs[1].ends_with_checkpoint is True
    assert legs[2].start_index == 4
    assert legs[2].waypoints == [e]
    assert legs[2].ends_with_checkpoint is False


def test_split_legs_leading_checkpoint():
    a, b, c = make_wp(0, True), make_wp(1), make_wp(2)
    legs = split_legs([a, b, c])
    assert len(legs) == 2
    assert legs[0].start_index == 0
    assert legs[0].waypoints == [a]
    assert legs[0].ends_with_checkpoint is True
    assert legs[1].start_index == 1
    assert legs[1].waypoints == [b, c]
    assert legs[1].ends_with_checkpoint is False


def test_split_legs_trailing_checkpoint():
    a, b, c = make_wp(0), make_wp(1), make_wp(2, True)
    legs = split_legs([a, b, c])
    assert len(legs) == 1
    assert legs[0].start_index == 0
    assert legs[0].waypoints == [a, b, c]
    assert legs[0].ends_with_checkpoint is True


def test_split_legs_consecutive_checkpoints():
    a, b, c = make_wp(0, True), make_wp(1, True), make_wp(2)
    legs = split_legs([a, b, c])
    assert len(legs) == 3
    assert legs[0].start_index == 0
    assert legs[0].waypoints == [a]
    assert legs[0].ends_with_checkpoint is True
    assert legs[1].start_index == 1
    assert legs[1].waypoints == [b]
    assert legs[1].ends_with_checkpoint is True
    assert legs[2].start_index == 2
    assert legs[2].waypoints == [c]
    assert legs[2].ends_with_checkpoint is False


def test_split_legs_all_checkpoints():
    a, b = make_wp(0, True), make_wp(1, True)
    legs = split_legs([a, b])
    assert len(legs) == 2
    assert legs[0].start_index == 0
    assert legs[0].waypoints == [a]
    assert legs[0].ends_with_checkpoint is True
    assert legs[1].start_index == 1
    assert legs[1].waypoints == [b]
    assert legs[1].ends_with_checkpoint is True


# --- speed_limit_for ---

def test_speed_limit_for_in_range():
    waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'checkpoint': False, 'speed_limit': 30.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'checkpoint': False, 'speed_limit': 60.0},
    ]
    leg = split_legs(waypoints)[0]
    assert speed_limit_for(leg, 0) == 30.0
    assert speed_limit_for(leg, 1) == 60.0


def test_speed_limit_for_out_of_range_returns_last():
    waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'checkpoint': False, 'speed_limit': 30.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'checkpoint': False, 'speed_limit': 60.0},
    ]
    leg = split_legs(waypoints)[0]
    assert speed_limit_for(leg, 5) == 60.0


def test_speed_limit_for_negative_index_returns_last():
    waypoints = [
        {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'checkpoint': False, 'speed_limit': 30.0},
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'checkpoint': False, 'speed_limit': 60.0},
    ]
    leg = split_legs(waypoints)[0]
    assert speed_limit_for(leg, -1) == 60.0
