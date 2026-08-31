#!/usr/bin/env python3
"""Waypoint yaml parsing / leg splitting. ROS 非依存（pytest でロジックだけ検証するため）。"""
from dataclasses import dataclass

import yaml


@dataclass
class Leg:
    start_index: int
    waypoints: list
    ends_with_checkpoint: bool


def load_waypoints(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}

    raw_waypoints = data.get('waypoints')
    if not raw_waypoints:
        raise ValueError(f'no waypoints found in {path}')

    waypoints = []
    for i, wp in enumerate(raw_waypoints):
        missing = [key for key in ('x', 'y') if key not in wp]
        if missing:
            raise ValueError(f'waypoint {i} is missing required key(s) {missing}: {wp}')
        waypoints.append({
            'x': float(wp['x']),
            'y': float(wp['y']),
            'yaw': float(wp.get('yaw', 0.0)),
            'checkpoint': bool(wp.get('checkpoint', False)),
            'speed_limit': float(wp.get('speed_limit', 0.0)),
        })

    loop = bool(data.get('loop', False))
    return waypoints, loop


def split_legs(waypoints):
    legs = []
    current = []
    start_index = 0
    for i, wp in enumerate(waypoints):
        if not current:
            start_index = i
        current.append(wp)
        if wp['checkpoint']:
            legs.append(Leg(start_index=start_index, waypoints=current, ends_with_checkpoint=True))
            current = []
    if current:
        legs.append(Leg(start_index=start_index, waypoints=current, ends_with_checkpoint=False))
    return legs


def speed_limit_for(leg, feedback_index):
    # feedback はレグ長を超えて報告される瞬間があるため、範囲外は最終点の値を使う
    if 0 <= feedback_index < len(leg.waypoints):
        return leg.waypoints[feedback_index]['speed_limit']
    return leg.waypoints[-1]['speed_limit']
