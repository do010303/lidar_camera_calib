"""Geometry shared by capture, offline solver and projection (metres)."""
from pathlib import Path
import cv2
import numpy as np
import yaml


def load_camera(path):
    with Path(path).expanduser().open() as f:
        cfg = yaml.safe_load(f)
    K = np.asarray(cfg['K'], float).reshape(3, 3)
    D = np.asarray(cfg['D'], float).ravel()
    if not np.isfinite(K).all() or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    return cfg, K, D


def scan_points(scan):
    r = np.asarray(scan.ranges, float)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    a = np.arctan2(np.sin(a), np.cos(a))  # drivers like ldlidar publish 0..2pi; ROI gates assume -pi..pi
    good = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    ids = np.flatnonzero(good)
    return np.c_[r[good]*np.cos(a[good]), r[good]*np.sin(a[good]), np.zeros(len(ids))], ids, a[good], r[good]


def fit_line(points):
    center = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points[:, :2]-center[:2], full_matrices=False)
    direction = vh[0]
    errors = (points[:, :2]-center[:2]) @ vh[1]
    return float(np.sqrt(np.mean(errors**2))), direction


def select_segment(points, ids, mask, min_points=6, gap=0.04, line_rmse=0.008):
    """Require exactly one contiguous, straight cluster inside operator ROI."""
    p, ix = points[mask], ids[mask]
    if len(p) < min_points:
        raise ValueError('Too few points in ROI')
    breaks = np.flatnonzero((np.diff(ix) != 1) | (np.linalg.norm(np.diff(p, axis=0), axis=1) > gap))+1
    clusters = [c for c in np.split(np.arange(len(p)), breaks) if len(c) >= min_points]
    if len(clusters) != 1:
        raise ValueError(f'ROI has {len(clusters)} candidate segments; isolate the board')
    chosen = clusters[0]
    error, _ = fit_line(p[chosen])
    if error > line_rmse:
        raise ValueError(f'Segment not planar/straight: line RMSE {error:.4f} m')
    return p[chosen], ix[chosen], error


def candidate_segments(points, ids, mask, min_points=6, gap=0.04, line_rmse=0.008):
    """All contiguous clusters in the ROI that are long enough and straight (unlike select_segment, never raises)."""
    p, ix = points[mask], ids[mask]
    if len(p) < min_points:
        return []
    breaks = np.flatnonzero((np.diff(ix) != 1) | (np.linalg.norm(np.diff(p, axis=0), axis=1) > gap))+1
    out = []
    for c in np.split(np.arange(len(p)), breaks):
        if len(c) < min_points:
            continue
        error, _ = fit_line(p[c])
        out.append(dict(points=p[c], ids=ix[c], rmse=error, straight=error <= line_rmse))
    return out


def project(points, R, t, K, D, width, height):
    pc = (R @ points.T).T + t
    good = np.isfinite(pc).all(axis=1) & (pc[:, 2] > 0.01)
    source = np.flatnonzero(good)
    if not len(source):
        return np.empty((0, 2)), source
    uv = cv2.projectPoints(pc[good], np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    inside = np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return uv[inside], source[inside]
