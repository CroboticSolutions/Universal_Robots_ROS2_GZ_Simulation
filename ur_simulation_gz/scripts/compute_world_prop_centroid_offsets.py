#!/usr/bin/env python3
"""Compute model-origin -> visual mesh centroid offsets (m) for world_props spawn correction.

The launch file uses only the X and Y components of each offset; Z in the YAML is ignored for
spawn math (profile z is model origin height). Regenerate the YAML after changing meshes.

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
        "Banana",
        "Gear",
        "Wrench",
        "Drone",
        "Drill",
        "Hammer",
    ]
    print("# world_prop_centroid_offset_m.yaml — generated by compute_world_prop_centroid_offsets.py")
    print("# Vector from Gazebo model origin to primary visual mesh centroid (model frame, meters).")
    print("# Profile x,y,z = that centroid; spawn = profile - R(rpy) * offset (see launch file).")
    print("centroid_offset_m:")
    for name in names:
        try:
            c, uri = centroid_in_model_frame(models_dir, name, first_visual_only=True)
            print(f"  {name}: [{c[0]:.12f}, {c[1]:.12f}, {c[2]:.12f}]  # {uri}")
        except Exception as exc:
            print(f"  # {name}: FAILED {exc}", file=sys.stderr)
            raise


if __name__ == "__main__":
    main()
