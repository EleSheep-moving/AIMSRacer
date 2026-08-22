#!/usr/bin/env python3
"""Offline verifier for longitudinal current-to-acceleration models.

The script keeps temperature out of the first-pass fit and focuses on measured
phase/q-axis current, battery voltage, duty, and odometry-derived acceleration.
It compares these deployable feed-forward model shapes:

    Model A: a = k * I_net
    Model B: a = I_net * (k0 + k1 * (v_mid - v_ref))
    Model C: a = I_net * (k0 + k1 * (v_mid - v_ref)
                          + k2 * (V_mean - V_ref))

where I_net = I_q_mean - I_base(v0).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


def _nan() -> float:
    return float("nan")


def _mean(values: Sequence[float]) -> float:
    data = [float(v) for v in values if _finite(float(v))]
    return float(statistics.fmean(data)) if data else _nan()


def _std(values: Sequence[float]) -> float:
    data = [float(v) for v in values if _finite(float(v))]
    return float(statistics.pstdev(data)) if len(data) >= 2 else 0.0 if len(data) == 1 else _nan()


def _min(values: Sequence[float]) -> float:
    data = [float(v) for v in values if _finite(float(v))]
    return float(min(data)) if data else _nan()


def _max(values: Sequence[float]) -> float:
    data = [float(v) for v in values if _finite(float(v))]
    return float(max(data)) if data else _nan()


def _json_number(value: float) -> Optional[float]:
    value = float(value)
    return value if math.isfinite(value) else None


def _load_base_currents(path: Path) -> Dict[float, float]:
    mapping: Dict[float, float] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.replace(",", " ").split()
        if len(parts) < 2:
            continue
        try:
            speed = float(parts[0])
            current = float(parts[1])
        except Exception:
            continue
        mapping[round(speed, 6)] = current
    return mapping


def _interp_base_current(base: Dict[float, float], speed: float) -> float:
    if not base:
        return _nan()
    key = round(float(speed), 6)
    if key in base:
        return float(base[key])
    xs = sorted(base)
    if speed <= xs[0]:
        return float(base[xs[0]])
    if speed >= xs[-1]:
        return float(base[xs[-1]])
    for lo, hi in zip(xs[:-1], xs[1:]):
        if lo <= speed <= hi:
            ratio = (speed - lo) / max(hi - lo, 1e-9)
            return float(base[lo] + ratio * (base[hi] - base[lo]))
    return _nan()


@dataclass
class TrialRow:
    source: str
    bag_id: str
    bag_path: str
    v0: float
    v1: float
    v_mid_mps: float
    current_cmd_a: float
    t_sec: float
    reached: bool
    accel_endpoint_mps2: float
    accel_fit_mps2: float
    accel_fit_r2: float
    iq_mean_a: float
    iq_std_a: float
    i_base_a: float
    i_eff_a: float
    i_net_a: float
    voltage_mean_v: float
    voltage_min_v: float
    duty_mean: float
    duty_max: float
    speed_start_mps: float
    speed_end_mps: float
    speed_max_mps: float
    valid: bool
    reject_reason: str


def _parse_boolish(value: object) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "ok", "reached"}


def _read_csv_trials(
    samples_path: Path,
    base_currents: Dict[float, float],
    valid_v_end: float,
    valid_v_start: float,
) -> List[TrialRow]:
    rows: List[TrialRow] = []
    with samples_path.open(newline="") as handle:
        reader = csv.DictReader(line for line in handle if not line.lstrip().startswith("#"))
        for raw in reader:
            try:
                v0 = float(raw["v0"])
                v1 = float(raw["v1"])
                current = float(raw.get("current_A", raw.get("current_cmd_A", "nan")))
                t_sec = float(raw["t_sec"])
                reached = _parse_boolish(raw["reached"])
            except Exception:
                continue

            accel_endpoint = _nan()
            if reached and t_sec > 0.0:
                accel_endpoint = float(raw.get("accel_mps2", (v1 - v0) / t_sec))
            elif "accel_mps2" in raw:
                try:
                    accel_endpoint = float(raw["accel_mps2"])
                except Exception:
                    accel_endpoint = _nan()

            i_base = _interp_base_current(base_currents, v0)
            iq_mean = current
            i_eff = iq_mean - i_base if _finite(i_base) else _nan()

            valid = True
            reasons: List[str] = []
            if not reached:
                valid = False
                reasons.append("timeout_or_not_reached")
            if v0 < valid_v_start - 1e-9 or v1 > valid_v_end + 1e-9:
                valid = False
                reasons.append("speed_range_excluded")
            if not _finite(i_eff):
                valid = False
                reasons.append("missing_base_current")
            if not _finite(accel_endpoint):
                valid = False
                reasons.append("missing_accel")

            rows.append(
                TrialRow(
                    source="csv_cmd_current_proxy",
                    bag_id="csv",
                    bag_path=str(samples_path),
                    v0=v0,
                    v1=v1,
                    v_mid_mps=0.5 * (v0 + v1),
                    current_cmd_a=current,
                    t_sec=t_sec,
                    reached=reached,
                    accel_endpoint_mps2=accel_endpoint,
                    accel_fit_mps2=accel_endpoint,
                    accel_fit_r2=_nan(),
                    iq_mean_a=iq_mean,
                    iq_std_a=_nan(),
                    i_base_a=i_base,
                    i_eff_a=i_eff,
                    i_net_a=i_eff,
                    voltage_mean_v=_nan(),
                    voltage_min_v=_nan(),
                    duty_mean=_nan(),
                    duty_max=_nan(),
                    speed_start_mps=v0,
                    speed_end_mps=v1 if reached else _nan(),
                    speed_max_mps=v1 if reached else _nan(),
                    valid=valid,
                    reject_reason=";".join(reasons),
                )
            )
    return rows


@dataclass
class _BagSegment:
    t0_ns: int
    t_last_ns: int
    t_end_ns: int
    v0: float
    v1: float
    current_cmd_a: float
    outcome: str
    status_speed_start: float
    status_speed_end: float
    status_speed_max: float
    odom_t_ns: List[int]
    odom_speed: List[float]
    iq_values: List[float]
    voltage_values: List[float]
    duty_values: List[float]


def _get_msg_class(topic_types: Dict[str, str], topic: str):
    from rosidl_runtime_py.utilities import get_message

    if topic not in topic_types:
        return None
    return get_message(topic_types[topic])


def _open_bag_reader(bag_path: Path):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    return reader


def _read_bag_status_segments(
    bag_path: Path,
    status_topic: str,
    dv: float,
) -> Tuple[List[_BagSegment], Dict[str, str]]:
    from rclpy.serialization import deserialize_message

    reader = _open_bag_reader(bag_path)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    marker_cls = _get_msg_class(topic_types, status_topic)
    if marker_cls is None:
        raise RuntimeError(f"Bag does not contain status topic {status_topic!r}")

    line_re = re.compile(r"^(\S+) \| (\S+) \| (\S+)")
    vt_re = re.compile(r"v=([-+0-9.]+)m/s -> target=([-+0-9.]+)m/s \| ([A-Z_]+)")
    icmd_re = re.compile(r"I_cmd=([-+0-9.]+)A")

    segments: List[_BagSegment] = []
    current_segment: Optional[_BagSegment] = None
    last_start_target: Optional[float] = None
    last_start_speed: Optional[float] = None

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic != status_topic:
            continue
        msg = deserialize_message(data, marker_cls)
        text = getattr(msg, "text", "")
        first_line = text.split("\n", 1)[0]
        line_match = line_re.search(first_line)
        vt_match = vt_re.search(text)
        icmd_match = icmd_re.search(text)
        if not (line_match and vt_match and icmd_match):
            continue

        stage = line_match.group(2)
        speed = float(vt_match.group(1))
        target = float(vt_match.group(2))
        current_cmd = float(icmd_match.group(1))

        if stage == "PP_B_START_TRIAL":
            last_start_target = target
            last_start_speed = speed

        if stage == "PP_B_CURRENT_TRIAL":
            if last_start_target is None:
                inferred = round(speed / max(dv, 1e-9)) * dv
                last_start_target = inferred
                last_start_speed = speed
            if current_segment is None:
                v0 = float(last_start_target)
                current_segment = _BagSegment(
                    t0_ns=stamp_ns,
                    t_last_ns=stamp_ns,
                    t_end_ns=stamp_ns,
                    v0=v0,
                    v1=v0 + dv,
                    current_cmd_a=current_cmd,
                    outcome="",
                    status_speed_start=float(last_start_speed if last_start_speed is not None else speed),
                    status_speed_end=speed,
                    status_speed_max=speed,
                    odom_t_ns=[],
                    odom_speed=[],
                    iq_values=[],
                    voltage_values=[],
                    duty_values=[],
                )
            else:
                current_segment.t_last_ns = stamp_ns
                current_segment.t_end_ns = stamp_ns
                current_segment.status_speed_end = speed
                current_segment.status_speed_max = max(current_segment.status_speed_max, speed)
        else:
            if current_segment is not None:
                current_segment.t_end_ns = stamp_ns
                current_segment.outcome = stage
                current_segment.status_speed_end = speed
                current_segment.status_speed_max = max(current_segment.status_speed_max, speed)
                segments.append(current_segment)
                current_segment = None

    if current_segment is not None:
        current_segment.outcome = "BAG_END"
        segments.append(current_segment)

    return segments, topic_types


def _extract_vesc_value(msg, preferred: Sequence[str]) -> float:
    state = getattr(msg, "state", msg)
    for name in preferred:
        if not hasattr(state, name):
            continue
        try:
            value = float(getattr(state, name))
        except Exception:
            continue
        if math.isfinite(value):
            return value
    return _nan()


def _fit_speed_slope(
    t_ns: Sequence[int],
    speeds: Sequence[float],
    t0_ns: int,
    skip_sec: float,
) -> Tuple[float, float]:
    samples = [
        ((float(t) - float(t0_ns)) / 1e9, float(v))
        for t, v in zip(t_ns, speeds)
        if (float(t) - float(t0_ns)) / 1e9 >= skip_sec and math.isfinite(float(v))
    ]
    if len(samples) < 4:
        return _nan(), _nan()
    x = np.array([item[0] for item in samples], dtype=float)
    y = np.array([item[1] for item in samples], dtype=float)
    if float(np.ptp(x)) <= 1e-6:
        return _nan(), _nan()
    slope, intercept = np.polyfit(x, y, 1)
    pred = slope * x + intercept
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else _nan()
    return float(slope), float(r2)


def _read_bag_trials(
    bag_path: Path,
    bag_id: str,
    base_currents: Dict[float, float],
    valid_v_end: float,
    valid_v_start: float,
    status_topic: str,
    odom_topic: str,
    vesc_topic: str,
    dv: float,
    fit_skip_sec: float,
    min_fit_r2: float,
    reach_tolerance: float,
    speed_source: str,
) -> List[TrialRow]:
    from rclpy.serialization import deserialize_message

    segments, topic_types = _read_bag_status_segments(bag_path, status_topic, dv)
    if not segments:
        raise RuntimeError("No PP_B_CURRENT_TRIAL segments found in bag status text")

    odom_cls = _get_msg_class(topic_types, odom_topic)
    vesc_cls = _get_msg_class(topic_types, vesc_topic)
    if odom_cls is None:
        raise RuntimeError(f"Bag does not contain odom topic {odom_topic!r}")
    if vesc_cls is None:
        raise RuntimeError(f"Bag does not contain VESC topic {vesc_topic!r}")

    reader = _open_bag_reader(bag_path)
    seg_idx = 0
    segments = sorted(segments, key=lambda item: item.t0_ns)

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        while seg_idx < len(segments) and stamp_ns > segments[seg_idx].t_end_ns:
            seg_idx += 1
        if seg_idx >= len(segments):
            break
        segment = segments[seg_idx]
        if stamp_ns < segment.t0_ns or stamp_ns > segment.t_end_ns:
            continue

        if topic == odom_topic:
            msg = deserialize_message(data, odom_cls)
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            speed = vx if speed_source == "linear_x" else math.hypot(vx, vy)
            segment.odom_t_ns.append(stamp_ns)
            segment.odom_speed.append(float(speed))
        elif topic == vesc_topic:
            msg = deserialize_message(data, vesc_cls)
            iq = _extract_vesc_value(msg, ["avg_iq", "current_motor"])
            voltage = _extract_vesc_value(msg, ["voltage_input"])
            duty = _extract_vesc_value(msg, ["duty_cycle"])
            if _finite(iq):
                segment.iq_values.append(iq)
            if _finite(voltage):
                segment.voltage_values.append(voltage)
            if _finite(duty):
                segment.duty_values.append(abs(duty))

    rows: List[TrialRow] = []
    for segment in segments:
        t_sec = max(0.0, (segment.t_end_ns - segment.t0_ns) / 1e9)
        speed_start = segment.odom_speed[0] if segment.odom_speed else segment.status_speed_start
        speed_end = segment.odom_speed[-1] if segment.odom_speed else segment.status_speed_end
        speed_max = _max(segment.odom_speed) if segment.odom_speed else segment.status_speed_max
        reached = bool(speed_max >= segment.v1 - reach_tolerance and segment.outcome == "PP_B_RECORD_TRIAL")
        endpoint_accel = (segment.v1 - segment.v0) / max(t_sec, 1e-3) if reached else _nan()
        fit_accel, fit_r2 = _fit_speed_slope(
            segment.odom_t_ns,
            segment.odom_speed,
            segment.t0_ns,
            fit_skip_sec,
        )
        iq_mean = _mean(segment.iq_values)
        i_base = _interp_base_current(base_currents, segment.v0)
        i_eff = iq_mean - i_base if _finite(iq_mean) and _finite(i_base) else _nan()

        valid = True
        reasons: List[str] = []
        if not reached:
            valid = False
            reasons.append("timeout_or_not_reached")
        if segment.outcome != "PP_B_RECORD_TRIAL":
            valid = False
            reasons.append(f"outcome_{segment.outcome}")
        if segment.v0 < valid_v_start - 1e-9 or segment.v1 > valid_v_end + 1e-9:
            valid = False
            reasons.append("speed_range_excluded")
        if not _finite(i_eff):
            valid = False
            reasons.append("missing_iq_or_base")
        if not _finite(fit_accel):
            valid = False
            reasons.append("missing_accel_fit")
        if _finite(fit_r2) and fit_r2 < min_fit_r2:
            valid = False
            reasons.append("low_accel_fit_r2")

        rows.append(
            TrialRow(
                source="bag_avg_iq",
                bag_id=bag_id,
                bag_path=str(bag_path),
                v0=segment.v0,
                v1=segment.v1,
                v_mid_mps=0.5 * (segment.v0 + segment.v1),
                current_cmd_a=segment.current_cmd_a,
                t_sec=t_sec,
                reached=reached,
                accel_endpoint_mps2=endpoint_accel,
                accel_fit_mps2=fit_accel,
                accel_fit_r2=fit_r2,
                iq_mean_a=iq_mean,
                iq_std_a=_std(segment.iq_values),
                i_base_a=i_base,
                i_eff_a=i_eff,
                i_net_a=i_eff,
                voltage_mean_v=_mean(segment.voltage_values),
                voltage_min_v=_min(segment.voltage_values),
                duty_mean=_mean(segment.duty_values),
                duty_max=_max(segment.duty_values),
                speed_start_mps=float(speed_start),
                speed_end_mps=float(speed_end),
                speed_max_mps=float(speed_max),
                valid=valid,
                reject_reason=";".join(reasons),
            )
        )

    return rows


def _linear_fit(x_values: Sequence[float], y_values: Sequence[float]) -> Dict[str, object]:
    x = np.array(x_values, dtype=float)
    y = np.array(y_values, dtype=float)
    if len(x) < 2 or float(np.ptp(x)) <= 1e-9:
        return {
            "n": int(len(x)),
            "slope_k": None,
            "intercept_b": None,
            "r2": None,
            "rmse": None,
        }
    slope, intercept = np.polyfit(x, y, 1)
    pred = slope * x + intercept
    residual = y - pred
    ss_res = float(np.sum(residual ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else _nan()
    rmse = math.sqrt(ss_res / max(len(x), 1))
    return {
        "n": int(len(x)),
        "slope_k": _json_number(float(slope)),
        "intercept_b": _json_number(float(intercept)),
        "r2": _json_number(r2),
        "rmse": _json_number(rmse),
    }


def _zero_intercept_fit(x_values: Sequence[float], y_values: Sequence[float]) -> Dict[str, object]:
    x = np.array(x_values, dtype=float)
    y = np.array(y_values, dtype=float)
    if len(x) < 1 or float(np.sum(x * x)) <= 1e-12:
        return {
            "n": int(len(x)),
            "slope_k0": None,
            "r2_about_mean": None,
            "rmse": None,
        }
    slope = float(np.sum(x * y) / np.sum(x * x))
    pred = slope * x
    residual = y - pred
    ss_res = float(np.sum(residual ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else _nan()
    rmse = math.sqrt(ss_res / max(len(x), 1))
    return {
        "n": int(len(x)),
        "slope_k0": _json_number(slope),
        "r2_about_mean": _json_number(r2),
        "rmse": _json_number(rmse),
    }


def _range_summary(values: Sequence[float]) -> Dict[str, Optional[float]]:
    data = [float(value) for value in values if _finite(float(value))]
    if not data:
        return {"min": None, "mean": None, "max": None}
    return {
        "min": _json_number(min(data)),
        "mean": _json_number(_mean(data)),
        "max": _json_number(max(data)),
    }


def _linear_feature_fit(
    features: Sequence[Sequence[float]],
    targets: Sequence[float],
    coefficient_names: Sequence[str],
) -> Dict[str, object]:
    row_count = len(targets)
    coefficients = {name: None for name in coefficient_names}
    if row_count < max(1, len(coefficient_names)):
        return {
            "n": int(row_count),
            "coefficients": coefficients,
            "r2": None,
            "rmse": None,
            "reason": "insufficient_samples",
        }

    x = np.array(features, dtype=float)
    y = np.array(targets, dtype=float)
    if x.ndim != 2 or x.shape[0] != row_count or x.shape[1] != len(coefficient_names):
        return {
            "n": int(row_count),
            "coefficients": coefficients,
            "r2": None,
            "rmse": None,
            "reason": "invalid_feature_shape",
        }
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        return {
            "n": int(row_count),
            "coefficients": coefficients,
            "r2": None,
            "rmse": None,
            "reason": "non_finite_input",
        }

    beta, *_ = np.linalg.lstsq(x, y, rcond=None)
    pred = x @ beta
    residual = y - pred
    ss_res = float(np.sum(residual ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else _nan()
    rmse = math.sqrt(ss_res / max(row_count, 1))
    return {
        "n": int(row_count),
        "coefficients": {
            name: _json_number(float(value))
            for name, value in zip(coefficient_names, beta)
        },
        "r2": _json_number(r2),
        "rmse": _json_number(rmse),
    }


def _model_rows(
    rows: Sequence[TrialRow],
    model_accel_min: float,
    model_accel_max: float,
) -> List[TrialRow]:
    valid_rows = [
        row
        for row in rows
        if (
            row.valid
            and _finite(row.i_net_a)
            and _finite(row.accel_fit_mps2)
            and _finite(row.v_mid_mps)
        )
    ]
    if math.isfinite(model_accel_min):
        valid_rows = [row for row in valid_rows if row.accel_fit_mps2 >= model_accel_min]
    if math.isfinite(model_accel_max):
        valid_rows = [row for row in valid_rows if row.accel_fit_mps2 <= model_accel_max]
    return valid_rows


def _fit_aggregate_models(
    rows: Sequence[TrialRow],
    model_accel_min: float,
    model_accel_max: float,
    v_ref_override: float,
    voltage_ref_override: float,
) -> Dict[str, object]:
    valid_rows = _model_rows(rows, model_accel_min, model_accel_max)
    voltage_rows = [row for row in valid_rows if _finite(row.voltage_mean_v)]

    v_ref = (
        float(v_ref_override)
        if math.isfinite(v_ref_override)
        else _mean([row.v_mid_mps for row in valid_rows])
    )
    voltage_ref = (
        float(voltage_ref_override)
        if math.isfinite(voltage_ref_override)
        else _mean([row.voltage_mean_v for row in voltage_rows])
    )

    y_all = [row.accel_fit_mps2 for row in valid_rows]
    model_a_features = [[row.i_net_a] for row in valid_rows]
    model_b_features = [
        [row.i_net_a, row.i_net_a * (row.v_mid_mps - v_ref)]
        for row in valid_rows
    ]

    y_voltage = [row.accel_fit_mps2 for row in voltage_rows]
    model_c_features = [
        [
            row.i_net_a,
            row.i_net_a * (row.v_mid_mps - v_ref),
            row.i_net_a * (row.voltage_mean_v - voltage_ref),
        ]
        for row in voltage_rows
    ]

    model_a = _linear_feature_fit(model_a_features, y_all, ["k"])
    model_a["formula"] = "a = k * I_net"
    model_b = _linear_feature_fit(model_b_features, y_all, ["k0", "k1_speed"])
    model_b["formula"] = "a = I_net * (k0 + k1_speed*(v_mid-v_ref))"
    model_c = _linear_feature_fit(
        model_c_features,
        y_voltage,
        ["k0", "k1_speed", "k2_voltage"],
    )
    model_c["formula"] = (
        "a = I_net * (k0 + k1_speed*(v_mid-v_ref) "
        "+ k2_voltage*(V_mean-V_ref))"
    )

    def _rmse(model: Dict[str, object]) -> Optional[float]:
        value = model.get("rmse")
        return float(value) if value is not None else None

    def _delta(new_model: Dict[str, object], old_model: Dict[str, object]) -> Optional[float]:
        new = _rmse(new_model)
        old = _rmse(old_model)
        return _json_number(new - old) if new is not None and old is not None else None

    def _improvement_pct(new_model: Dict[str, object], old_model: Dict[str, object]) -> Optional[float]:
        new = _rmse(new_model)
        old = _rmse(old_model)
        if new is None or old is None or abs(old) <= 1e-12:
            return None
        return _json_number(100.0 * (old - new) / old)

    bag_fit_counts: Dict[str, int] = {}
    for row in valid_rows:
        bag_fit_counts[row.bag_id] = bag_fit_counts.get(row.bag_id, 0) + 1

    return {
        "fit_trial_count": len(valid_rows),
        "voltage_fit_trial_count": len(voltage_rows),
        "refs": {
            "v_ref_mps": _json_number(v_ref),
            "voltage_ref_v": _json_number(voltage_ref),
        },
        "data_ranges": {
            "v_mid_mps": _range_summary([row.v_mid_mps for row in valid_rows]),
            "i_net_a": _range_summary([row.i_net_a for row in valid_rows]),
            "accel_fit_mps2": _range_summary([row.accel_fit_mps2 for row in valid_rows]),
            "voltage_mean_v": _range_summary([row.voltage_mean_v for row in voltage_rows]),
            "duty_max": _range_summary([row.duty_max for row in valid_rows]),
        },
        "fit_trial_count_by_bag": dict(sorted(bag_fit_counts.items())),
        "model_a_net_current": model_a,
        "model_b_speed_modulated": model_b,
        "model_c_speed_voltage_modulated": model_c,
        "comparisons": {
            "model_b_rmse_delta_vs_a": _delta(model_b, model_a),
            "model_b_rmse_improvement_pct_vs_a": _improvement_pct(model_b, model_a),
            "model_c_rmse_delta_vs_b": _delta(model_c, model_b),
            "model_c_rmse_improvement_pct_vs_b": _improvement_pct(model_c, model_b),
        },
    }


def _fit_models(
    rows: Sequence[TrialRow],
    min_fit_samples: int,
    model_accel_min: float,
    model_accel_max: float,
    v_ref_override: float,
    voltage_ref_override: float,
) -> Dict[str, object]:
    valid_rows = _model_rows(rows, model_accel_min, model_accel_max)
    groups: Dict[str, List[TrialRow]] = {}
    for row in valid_rows:
        key = f"{row.v0:.3f}->{row.v1:.3f}"
        groups.setdefault(key, []).append(row)

    fits: Dict[str, object] = {}
    for key, items in sorted(groups.items()):
        if len(items) < min_fit_samples:
            continue
        x = [item.i_net_a for item in items]
        y = [item.accel_fit_mps2 for item in items]
        fits[key] = {
            "linear_with_intercept": _linear_fit(x, y),
            "zero_intercept": _zero_intercept_fit(x, y),
            "i_net_range_a": [_json_number(min(x)), _json_number(max(x))],
            "i_eff_range_a": [_json_number(min(x)), _json_number(max(x))],
            "accel_range_mps2": [_json_number(min(y)), _json_number(max(y))],
            "voltage_mean_v": _json_number(_mean([item.voltage_mean_v for item in items])),
            "duty_max": _json_number(_max([item.duty_max for item in items])),
        }

    if len(valid_rows) >= min_fit_samples:
        x_all = [item.i_net_a for item in valid_rows]
        y_all = [item.accel_fit_mps2 for item in valid_rows]
        fits["global"] = {
            "linear_with_intercept": _linear_fit(x_all, y_all),
            "zero_intercept": _zero_intercept_fit(x_all, y_all),
            "i_net_range_a": [_json_number(min(x_all)), _json_number(max(x_all))],
            "i_eff_range_a": [_json_number(min(x_all)), _json_number(max(x_all))],
            "accel_range_mps2": [_json_number(min(y_all)), _json_number(max(y_all))],
            "voltage_mean_v": _json_number(_mean([item.voltage_mean_v for item in valid_rows])),
            "duty_max": _json_number(_max([item.duty_max for item in valid_rows])),
        }
    return {
        "fits": fits,
        "valid_trial_count": len(valid_rows),
        "aggregate_models": _fit_aggregate_models(
            rows,
            model_accel_min=model_accel_min,
            model_accel_max=model_accel_max,
            v_ref_override=v_ref_override,
            voltage_ref_override=voltage_ref_override,
        ),
    }


def _write_trials_csv(path: Path, rows: Sequence[TrialRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(TrialRow.__dataclass_fields__.keys())
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            data = asdict(row)
            for key, value in list(data.items()):
                if isinstance(value, float):
                    data[key] = "" if not math.isfinite(value) else f"{value:.8g}"
            writer.writerow(data)


def _write_model_json(
    path: Path,
    base_currents: Dict[float, float],
    rows: Sequence[TrialRow],
    fits: Dict[str, object],
    args: argparse.Namespace,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "model": "Compare Model A/B/C where I_net = iq_mean - I_base(v0)",
        "temperature_policy": "not used in first-pass fit",
        "current_policy": "prefer state.avg_iq; fallback state.current_motor",
        "base_current": [
            {"speed_mps": speed, "current_a": current}
            for speed, current in sorted(base_currents.items())
        ],
        "filters": {
            "valid_v_start": args.valid_v_start,
            "valid_v_end": args.valid_v_end,
            "min_fit_r2": args.min_fit_r2,
            "fit_skip_sec": args.fit_skip_sec,
            "model_accel_min": _json_number(args.model_accel_min),
            "model_accel_max": _json_number(args.model_accel_max),
            "v_ref_override": _json_number(args.v_ref),
            "voltage_ref_override": _json_number(args.voltage_ref),
        },
        "trials": {
            "total": len(rows),
            "valid": sum(1 for row in rows if row.valid),
            "invalid": sum(1 for row in rows if not row.valid),
            "by_bag": {
                bag_id: sum(1 for row in rows if row.bag_id == bag_id)
                for bag_id in sorted({row.bag_id for row in rows})
            },
        },
        **fits,
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))


def _print_summary(rows: Sequence[TrialRow], fits: Dict[str, object]) -> None:
    total = len(rows)
    valid = [row for row in rows if row.valid]
    print(f"Trials: total={total}, valid={len(valid)}, rejected={total - len(valid)}")
    rejected: Dict[str, int] = {}
    for row in rows:
        if row.valid:
            continue
        for reason in row.reject_reason.split(";"):
            if reason:
                rejected[reason] = rejected.get(reason, 0) + 1
    if rejected:
        print("Reject reasons:")
        for key, count in sorted(rejected.items(), key=lambda item: (-item[1], item[0])):
            print(f"  {key}: {count}")

    fit_map = fits.get("fits", {})
    if not isinstance(fit_map, dict) or not fit_map:
        print("No fit produced. Check valid filters or input data.")
        return

    print("\nFit summary: a = k * I_eff + b; I_eff = iq_mean - I_base(v0)")
    for key, fit in fit_map.items():
        if not isinstance(fit, dict):
            continue
        lin = fit.get("linear_with_intercept", {})
        zero = fit.get("zero_intercept", {})
        if not isinstance(lin, dict) or not isinstance(zero, dict):
            continue
        print(
            f"  {key:13s} n={lin.get('n')} "
            f"k={lin.get('slope_k')} b={lin.get('intercept_b')} "
            f"R2={lin.get('r2')} RMSE={lin.get('rmse')} | "
            f"k0={zero.get('slope_k0')} RMSE0={zero.get('rmse')}"
        )

    aggregate = fits.get("aggregate_models", {})
    if not isinstance(aggregate, dict):
        return
    print("\nAggregate model comparison:")
    refs = aggregate.get("refs", {})
    if isinstance(refs, dict):
        print(f"  refs: v_ref={refs.get('v_ref_mps')} m/s, V_ref={refs.get('voltage_ref_v')} V")
    for label, key in [
        ("Model A", "model_a_net_current"),
        ("Model B", "model_b_speed_modulated"),
        ("Model C", "model_c_speed_voltage_modulated"),
    ]:
        model = aggregate.get(key, {})
        if not isinstance(model, dict):
            continue
        print(
            f"  {label}: n={model.get('n')} "
            f"coeff={model.get('coefficients')} "
            f"R2={model.get('r2')} RMSE={model.get('rmse')}"
        )
    comparisons = aggregate.get("comparisons", {})
    if isinstance(comparisons, dict):
        print(
            "  C vs B: "
            f"RMSE delta={comparisons.get('model_c_rmse_delta_vs_b')} "
            f"improvement={comparisons.get('model_c_rmse_improvement_pct_vs_b')}%"
        )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Verify the longitudinal current-to-acceleration model.",
    )
    parser.add_argument(
        "--base",
        type=Path,
        default=Path("speed_hold_current_results.txt"),
        help="Stage A speed-hold current results.",
    )
    parser.add_argument(
        "--samples",
        type=Path,
        default=Path("speed_interval_accel_samples.csv"),
        help="Stage B CSV. Used when --bag is not supplied.",
    )
    parser.add_argument(
        "--bag",
        type=Path,
        default=None,
        help="Optional rosbag directory. Reconstructs avg_iq/voltage/duty trial stats.",
    )
    parser.add_argument(
        "--bags",
        type=Path,
        nargs="+",
        default=None,
        help="Optional rosbag directories. Aggregates all bags into one fit.",
    )
    parser.add_argument("--out-prefix", default="longitudinal_model_verify")
    parser.add_argument("--valid-v-start", type=float, default=0.0)
    parser.add_argument("--valid-v-end", type=float, default=6.0)
    parser.add_argument("--dv", type=float, default=1.0)
    parser.add_argument("--status-topic", default="/calib/status_text")
    parser.add_argument("--odom-topic", default="/odometry/filtered")
    parser.add_argument("--vesc-topic", default="/sensors/core")
    parser.add_argument("--speed-source", choices=["norm", "linear_x"], default="norm")
    parser.add_argument("--fit-skip-sec", type=float, default=0.15)
    parser.add_argument("--min-fit-r2", type=float, default=0.80)
    parser.add_argument("--reach-tolerance", type=float, default=0.05)
    parser.add_argument("--min-fit-samples", type=int, default=3)
    parser.add_argument(
        "--model-accel-min",
        type=float,
        default=float("nan"),
        help="Optional lower acceleration bound for model fitting.",
    )
    parser.add_argument(
        "--model-accel-max",
        type=float,
        default=2.5,
        help="Optional upper acceleration bound for model fitting.",
    )
    parser.add_argument(
        "--v-ref",
        type=float,
        default=float("nan"),
        help="Override v_ref for speed-modulated aggregate models. Defaults to mean v_mid.",
    )
    parser.add_argument(
        "--voltage-ref",
        type=float,
        default=float("nan"),
        help="Override V_ref for voltage-modulated aggregate models. Defaults to mean voltage.",
    )
    return parser


def _unique_bag_id(path: Path, used: Dict[str, int]) -> str:
    base = path.name or "bag"
    count = used.get(base, 0)
    used[base] = count + 1
    return base if count == 0 else f"{base}_{count + 1}"


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    base_currents = _load_base_currents(args.base.expanduser())
    if not base_currents:
        raise RuntimeError(f"No base currents loaded from {args.base}")

    bag_paths: List[Path] = []
    if args.bag is not None:
        bag_paths.append(args.bag.expanduser())
    if args.bags:
        bag_paths.extend(path.expanduser() for path in args.bags)

    if bag_paths:
        rows = []
        used_bag_ids: Dict[str, int] = {}
        for bag_path in bag_paths:
            bag_id = _unique_bag_id(bag_path, used_bag_ids)
            rows.extend(
                _read_bag_trials(
                    bag_path=bag_path,
                    bag_id=bag_id,
                    base_currents=base_currents,
                    valid_v_end=args.valid_v_end,
                    valid_v_start=args.valid_v_start,
                    status_topic=args.status_topic,
                    odom_topic=args.odom_topic,
                    vesc_topic=args.vesc_topic,
                    dv=args.dv,
                    fit_skip_sec=args.fit_skip_sec,
                    min_fit_r2=args.min_fit_r2,
                    reach_tolerance=args.reach_tolerance,
                    speed_source=args.speed_source,
                )
            )
    else:
        rows = _read_csv_trials(
            samples_path=args.samples.expanduser(),
            base_currents=base_currents,
            valid_v_end=args.valid_v_end,
            valid_v_start=args.valid_v_start,
        )

    fits = _fit_models(
        rows,
        min_fit_samples=max(1, args.min_fit_samples),
        model_accel_min=args.model_accel_min,
        model_accel_max=args.model_accel_max,
        v_ref_override=args.v_ref,
        voltage_ref_override=args.voltage_ref,
    )

    out_prefix = Path(args.out_prefix).expanduser()
    trials_path = out_prefix.with_name(out_prefix.name + "_trials.csv")
    model_path = out_prefix.with_name(out_prefix.name + "_model.json")
    _write_trials_csv(trials_path, rows)
    _write_model_json(model_path, base_currents, rows, fits, args)
    _print_summary(rows, fits)
    print(f"\nWrote: {trials_path}")
    print(f"Wrote: {model_path}")


if __name__ == "__main__":
    main()
