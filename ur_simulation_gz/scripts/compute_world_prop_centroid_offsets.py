#!/usr/bin/env python3
"""Compute model-origin offsets (m) for world_props spawn correction.

Most models: vector from Gazebo model origin to **primary visual mesh centroid**.
**Table**: centroid of the named collision box (`top_plate`) so spawn aligns with the tabletop
collision used by physics/MoveIt — not the STL volumetric centroid.

The launch file uses only X and Y components; Z in YAML is informational for Table (spawn keeps
profile z as model-origin height). Regenerate after changing meshes or collision geometry.

Requires: trimesh, numpy (optional: pycollada for some DAE paths — trimesh uses it when present).

Print YAML for config/multi_ur/world_prop_centroid_offset_m.yaml
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from typing import Iterable

import numpy as np

try:
    import trimesh
except ImportError as exc:
    print("pip install trimesh numpy", file=sys.stderr)
    raise SystemExit(1) from exc

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def _parse_floats(text: str) -> list[float]:
    return [float(x) for x in re.split(r"\s+", text.strip()) if x]


def _dae_unit_to_meters(asset: ET.Element | None) -> float:
    if asset is None:
        return 1.0
    unit = asset.find("c:unit", NS)
    if unit is None:
        return 1.0
    m = unit.get("meter")
    if m is None:
        return 1.0
    return float(m)


def dae_mesh_centroid_meters(dae_path: str) -> np.ndarray:
    """Mean of all POSITION vertices in all meshes (collada), in meters."""
    tree = ET.parse(dae_path)
    root = tree.getroot()
    asset = root.find("c:asset", NS)
    unit_m = _dae_unit_to_meters(asset)
    id_to_floats: dict[str, list[float]] = {}
    for fa in root.findall(".//c:float_array", NS):
        fid = fa.get("id")
        if not fid or not fa.text:
            continue
        id_to_floats[fid] = _parse_floats(fa.text)

    positions: list[np.ndarray] = []
    for mesh in root.findall(".//c:mesh", NS):
        for vert in mesh.findall("c:vertices", NS):
            for inp in vert.findall("c:input", NS):
                if inp.get("semantic") != "POSITION":
                    continue
                src = inp.get("source", "")
                if not src.startswith("#"):
                    continue
                src_id = src[1:]
                src_el = root.find(f".//*[@id='{src_id}']", NS)
                if src_el is None:
                    continue
                src_fa = src_el.find("c:float_array", NS)
                if src_fa is None or not src_fa.text:
                    continue
                fid = src_fa.get("id")
                if fid is None:
                    continue
                data = id_to_floats.get(fid)
                if not data:
                    continue
                arr = np.array(data, dtype=np.float64).reshape(-1, 3) * unit_m
                positions.append(arr)

    if not positions:
        raise RuntimeError(f"No POSITION data in {dae_path}")
    all_pts = np.vstack(positions)
    return all_pts.mean(axis=0)


def parse_pose_matrix(parent: ET.Element | None) -> np.ndarray:
    """4x4 transform from SDF <pose> on parent (identity if missing)."""
    if parent is None:
        return np.eye(4)
    t = parent.find("pose")
    if t is None or not t.text:
        return np.eye(4)
    parts = t.text.strip().split()
    x, y, z, rr, pp, yy = map(float, parts[:6])
    cx, cy, cz = math.cos(rr), math.cos(pp), math.cos(yy)
    sx, sy, sz = math.sin(rr), math.sin(pp), math.sin(yy)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    r = rz @ ry @ rx
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = [x, y, z]
    return out


def mesh_centroid(path: str, scale: Iterable[float]) -> np.ndarray:
    m = trimesh.load(path, force="mesh")
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate([g for g in m.geometry.values()])
    sc = np.array(list(scale), dtype=np.float64)
    vs = np.asarray(m.vertices) * sc
    return vs.mean(axis=0)


def collision_box_center_and_footprint_xy(
    models_dir: str,
    model_name: str,
    collision_name: str,
) -> tuple[np.ndarray, tuple[float, float, float], tuple[float, float]]:
    """Center of a named box collision in model frame + XY axis-aligned footprint span (yaw=0).

    Returns:
        center_model (3,)
        box_size (sx, sy, sz) from SDF <size>
        (span_model_x, span_model_y) max-min of transformed corners projected on model XY.
    """
    sdf_path = os.path.join(models_dir, model_name, "model.sdf")
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model_el = root.find(".//model")
    if model_el is None:
        raise RuntimeError(f"No model in {sdf_path}")

    model_T = parse_pose_matrix(model_el)

    col_el = None
    for link in model_el.findall("link"):
        for col in link.findall("collision"):
            if col.get("name") == collision_name:
                col_el = col
                link_el = link
                break
        if col_el is not None:
            break
    if col_el is None:
        raise RuntimeError(f"No collision name={collision_name!r} in {sdf_path}")

    link_T = parse_pose_matrix(link_el)
    collision_T = parse_pose_matrix(col_el)

    geo = col_el.find("geometry/box")
    if geo is None:
        raise RuntimeError(f"Collision {collision_name} has no box geometry in {sdf_path}")
    size_el = geo.find("size")
    if size_el is None or not size_el.text:
        raise RuntimeError(f"Collision {collision_name} missing box size")
    sx, sy, sz = map(float, size_el.text.strip().split()[:3])
    half = np.array([sx / 2.0, sy / 2.0, sz / 2.0], dtype=np.float64)

    T_model_collision = model_T @ link_T @ collision_T
    center_model = (T_model_collision @ np.array([0.0, 0.0, 0.0, 1.0]))[:3]

    corners_model_xy: list[tuple[float, float]] = []
    for a in (-1.0, 1.0):
        for b in (-1.0, 1.0):
            for c in (-1.0, 1.0):
                p = np.array([a * half[0], b * half[1], c * half[2], 1.0])
                pm = T_model_collision @ p
                corners_model_xy.append((float(pm[0]), float(pm[1])))
    xs = [t[0] for t in corners_model_xy]
    ys = [t[1] for t in corners_model_xy]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    return center_model, (sx, sy, sz), (span_x, span_y)


def centroid_in_model_frame(
    models_dir: str, model_name: str, first_visual_only: bool = True
) -> tuple[np.ndarray, str]:
    sdf_path = os.path.join(models_dir, model_name, "model.sdf")
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model_el = root.find(".//model")
    if model_el is None:
        raise RuntimeError(f"No model in {sdf_path}")

    model_T = parse_pose_matrix(model_el)

    for link in model_el.findall("link"):
        L = parse_pose_matrix(link)
        visuals = link.findall("visual")
        if first_visual_only:
            visuals = visuals[:1]
        for vis in visuals:
            V = parse_pose_matrix(vis)
            geo = vis.find("geometry/mesh")
            if geo is None:
                continue
            u = geo.find("uri")
            if u is None or not u.text:
                continue
            uri = u.text.strip()
            sc_el = geo.find("scale")
            if sc_el is not None and sc_el.text:
                scale = [float(x) for x in sc_el.text.split()]
            else:
                scale = [1.0, 1.0, 1.0]

            if uri.startswith("model://"):
                rest = uri.replace("model://", "").split("/", 1)
                pkg = rest[0]
                sub = rest[1] if len(rest) > 1 else ""
                fpath = os.path.join(models_dir, pkg, sub)
            else:
                fpath = os.path.join(models_dir, model_name, uri)

            ext = os.path.splitext(fpath)[1].lower()
            if ext == ".dae":
                c_local = dae_mesh_centroid_meters(fpath)
                c_local = c_local * np.array(scale)
            elif ext in (".stl", ".obj"):
                c_local = mesh_centroid(fpath, scale)
            else:
                raise RuntimeError(f"Unsupported mesh: {fpath}")

            T = model_T @ L @ V
            ch = np.append(c_local, 1.0)
            c_model = (T @ ch)[:3]
            return c_model, uri
    raise RuntimeError(f"No visual mesh in {sdf_path}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--models-dir",
        default=os.path.join(os.path.dirname(__file__), "..", "models"),
        help="Path to .../ur_simulation_gz/models",
    )
    args = ap.parse_args()
    models_dir = os.path.abspath(args.models_dir)
    names = [
        "Table",
        "Coke",
        "GraspCube",
        "Banana",
        "Gear",
        "Wrench",
        "Drone",
        "Drill",
        "Hammer",
    ]
    print("# world_prop_centroid_offset_m.yaml — generated by compute_world_prop_centroid_offsets.py")
    print("# Model-origin -> anchor (visual centroid, except Table = top_plate collision center).")
    print("# Launch uses XY only for spawn; Table Z optional/MoveIt reference. Spawn: profile_xy - R*(ox,oy,0).")
    print("centroid_offset_m:")
    for name in names:
        if name == "GraspCube":
            print(f"  GraspCube: [0.0, 0.0, 0.0225]  # primitive box, COM half-height")
            continue
        if name == "Table":
            c, box_sz, fp_xy = collision_box_center_and_footprint_xy(
                models_dir, name, collision_name="top_plate"
            )
            print(
                f"  {name}: [{c[0]:.12f}, {c[1]:.12f}, {c[2]:.12f}]  "
                f"# collision top_plate box center; size {box_sz}; footprint_xy_span {fp_xy}"
            )
            continue
        try:
            c, uri = centroid_in_model_frame(models_dir, name, first_visual_only=True)
            print(f"  {name}: [{c[0]:.12f}, {c[1]:.12f}, {c[2]:.12f}]  # {uri}")
        except Exception as exc:
            print(f"  # {name}: FAILED {exc}", file=sys.stderr)
            raise


if __name__ == "__main__":
    main()
