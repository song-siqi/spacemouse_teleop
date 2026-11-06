"""Clock estimates retain unknown fixed delay; residuals are not accuracy bounds."""

import numpy as np


def affine(x, y):
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    if (
        len(x) < 2
        or not np.isfinite(x).all()
        or not np.isfinite(y).all()
        or np.any(np.diff(x) < 0)
    ):
        raise ValueError("Invalid clock evidence")
    origin = float(x[0])
    dx = x - origin
    mask = np.ones(len(x), bool)
    for _ in range(4):
        if np.ptp(dx[mask]) <= 0:
            raise ValueError("Degenerate clock evidence")
        a, b = np.polyfit(dx[mask], y[mask] - y[0], 1)
        residual = y - (y[0] + a * dx + b)
        median = np.median(residual)
        mad = np.median(abs(residual - median))
        mask = abs(residual - median) <= max(6 * mad, 0.002)
    if not 0.95 < a < 1.05:
        raise ValueError("Implausible clock drift")
    mapped = y[0] + a * dx + b
    return mapped, dict(
        slope=float(a),
        origin=origin,
        intercept=float(y[0] + b),
        residual_p95_s=float(np.percentile(abs(y - mapped), 95)),
        fixed_offset_uncertainty_s=None,
    )


def map_source(rows, kind, anchors, offset=0.0):
    if len({r.get("clock_epoch", 0) for r in rows}) != 1:
        raise ValueError("Source crosses clock epochs")
    host = np.array([r["host_received_at"] for r in rows])
    if kind == "shared":
        mapped = np.array([r["source_timestamp"] for r in rows])
        fit = dict(method="shared_source_clock", fixed_offset_uncertainty_s=0.0)
    elif kind == "camera":
        if any("global_time" not in r["values"]["timestamp_domain"] for r in rows):
            raise ValueError("Camera requires global-time mapping evidence")
        if len({a["clock_epoch"] for a in anchors}) != 1:
            raise ValueError("Host mapping changed within session")
        mono = np.array(
            [
                (a["monotonic_before_ns"] + a["monotonic_after_ns"]) / 2e9
                for a in anchors
            ]
        )
        wall = np.array([a["realtime_ns"] / 1e9 for a in anchors])
        _, fit = affine(wall, mono)
        mapped = fit["intercept"] + fit["slope"] * (
            np.array([r["source_timestamp"] for r in rows]) - fit["origin"]
        )
        adjustments = []
        for row in rows:
            md = row["values"].get("frame_metadata", {})
            sensor = md.get("sensor_timestamp", {})
            frame = md.get("frame_timestamp", {})
            if sensor.get("available") and frame.get("available"):
                d = (sensor["raw"] - frame["raw"]) * 1e-6
                if abs(d) > 0.1:
                    raise ValueError("Implausible exposure/readout metadata difference")
                adjustments.append(d)
            else:
                adjustments.append(0.0)
        mapped = mapped + np.array(adjustments) * fit["slope"]
        fit["method"] = "SDK_global_time_to_host_anchors"
        fit["exposure_metadata_frames"] = sum(x != 0 for x in adjustments)
        fit["fallback_event"] = "SDK_frame_time_where_exposure_metadata_unavailable"
    elif kind == "robot":
        mapped, fit = affine([r["source_timestamp"] for r in rows], host)
        fit["method"] = "robust_one_way_receive_fit"
    else:
        mapped = np.array(
            [
                (r["values"]["query_started_at"] + r["values"]["query_finished_at"]) / 2
                for r in rows
            ]
        )
        fit = dict(method="query_midpoint", fixed_offset_uncertainty_s=None)
    mapped = mapped + offset
    if not np.isfinite(mapped).all() or np.any(np.diff(mapped) <= 0):
        raise ValueError("Non-increasing mapped times")
    fit["explicit_offset_s"] = offset
    return mapped, fit
