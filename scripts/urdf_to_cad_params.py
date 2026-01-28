#!/usr/bin/env python3
"""
Extract geometry and joint data from Sensei URDF/xacro and merge into cad/sensei_params.yaml.
Also writes cad/urdf_geometry.json (for Fusion GenerateFromParams) and updates
cad/export_config.json with link-name → STL path for generated bodies.

Run from repo root:
  python scripts/urdf_to_cad_params.py
  python scripts/urdf_to_cad_params.py --urdf-dir simulation/urdf --output cad/sensei_params.yaml

URDF lengths are in meters; we store them as-is in urdf_links/urdf_joints (Fusion scripts
convert to mm when needed). sensei_params.yaml can be used by Fusion 360 scripts via
Design.modifyParameters or as a reference for parametric models.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _resolve_link_name(name: str) -> str:
    """Resolve xacro params that appear as '${var}' in unprocessed files."""
    if name == "${base_link}":
        return "base_link"
    return name


def _resolve_joint_ref(ref: str) -> str:
    """Resolve xacro ${parent} and similar in joint parent/child."""
    if ref == "${parent}":
        return "base_link"
    return _resolve_link_name(ref)


def _link_to_subdir(name: str) -> str:
    if name == "base_link":
        return "torso"
    if name in ("l_shoulder_link", "l_upper_arm_link", "l_forearm_link", "l_hand_link"):
        return "arm_left"
    if name in ("r_shoulder_link", "r_upper_arm_link", "r_forearm_link", "r_hand_link"):
        return "arm_right"
    if name in ("l_hip_link", "l_thigh_link", "l_shin_link", "l_foot_link"):
        return "leg_left"
    if name in ("r_hip_link", "r_thigh_link", "r_shin_link", "r_foot_link"):
        return "leg_right"
    if name in ("neck_link", "head_link"):
        return "head"
    return "torso"

try:
    import yaml
except ImportError:
    yaml = None  # pip install -r scripts/requirements.txt

try:
    import xml.etree.ElementTree as ET
except ImportError:
    ET = None


def _tag(e, local: str) -> bool:
    return e.tag == local or (e.tag and e.tag.endswith("}" + local))


def _find(elem, local: str):
    for c in elem:
        if _tag(c, local):
            return c
    return None


def _findall(elem, local: str):
    return [c for c in elem if _tag(c, local)]


def _parse_origin(elem) -> tuple[list[float], list[float]]:
    o = _find(elem, "origin")
    if o is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    s = (o.get("xyz") or "").split()
    if len(s) >= 3:
        xyz = [float(s[0]), float(s[1]), float(s[2])]
    s = (o.get("rpy") or "").split()
    if len(s) >= 3:
        rpy = [float(s[0]), float(s[1]), float(s[2])]
    return xyz, rpy


def _parse_geometry(elem) -> dict | None:
    g = _find(elem, "geometry")
    if g is None:
        return None
    box = _find(g, "box")
    if box is not None:
        size = (box.get("size") or "0 0 0").split()
        if len(size) >= 3:
            return {"type": "box", "size": [float(size[0]), float(size[1]), float(size[2])]}
    cy = _find(g, "cylinder")
    if cy is not None:
        r = float(cy.get("radius") or 0)
        l_ = float(cy.get("length") or 0)
        return {"type": "cylinder", "radius": r, "length": l_}
    sp = _find(g, "sphere")
    if sp is not None:
        return {"type": "sphere", "radius": float(sp.get("radius") or 0)}
    return None


def _parse_axis(elem) -> list[float]:
    a = _find(elem, "axis")
    if a is None:
        return [0.0, 0.0, 1.0]
    s = (a.get("xyz") or "0 0 1").split()
    if len(s) >= 3:
        return [float(s[0]), float(s[1]), float(s[2])]
    return [0.0, 0.0, 1.0]


def extract_link(link_elem) -> dict | None:
    name = link_elem.get("name")
    if not name:
        return None
    # Prefer visual; fallback to collision
    for kind in ("visual", "collision"):
        v = _find(link_elem, kind)
        if v is None:
            continue
        geo = _parse_geometry(v)
        if geo is None:
            continue
        xyz, rpy = _parse_origin(v)
        return {
            "geometry": geo,
            "origin_xyz": xyz,
            "origin_rpy": rpy,
        }
    return None


def extract_joint(joint_elem) -> dict | None:
    name = joint_elem.get("name")
    jtype = joint_elem.get("type", "revolute")
    parent = _find(joint_elem, "parent")
    child = _find(joint_elem, "child")
    if parent is None or child is None:
        return None
    parent_link = parent.get("link") or ""
    child_link = child.get("link") or ""
    xyz, rpy = _parse_origin(joint_elem)
    axis = _parse_axis(joint_elem)
    return {
        "type": jtype,
        "parent": parent_link,
        "child": child_link,
        "origin_xyz": xyz,
        "origin_rpy": rpy,
        "axis": axis,
    }


def parse_urdf_file(path: Path) -> tuple[dict, dict]:
    tree = ET.parse(path)  # type: ignore[union-attr]
    root = tree.getroot()
    links, joints = {}, {}
    for e in root.iter():
        if _tag(e, "link"):
            rec = extract_link(e)
            if rec and e.get("name"):
                links[e.get("name")] = rec
        elif _tag(e, "joint"):
            rec = extract_joint(e)
            if rec and e.get("name"):
                joints[e.get("name")] = rec
    return links, joints


def main() -> int:
    ap = argparse.ArgumentParser(description="URDF → cad/sensei_params.yaml")
    ap.add_argument(
        "--urdf-dir",
        type=Path,
        default=Path("simulation/urdf"),
        help="Directory containing .urdf.xacro / .urdf",
    )
    ap.add_argument(
        "--output",
        type=Path,
        default=Path("cad/sensei_params.yaml"),
        help="Output YAML path",
    )
    args = ap.parse_args()

    if ET is None:
        print("xml.etree.ElementTree not available", file=sys.stderr)
        return 1
    if yaml is None:
        print("PyYAML required. Run: pip install -r scripts/requirements.txt", file=sys.stderr)
        return 1

    urdf_dir = args.urdf_dir
    if not urdf_dir.is_dir():
        print(f"Not a directory: {urdf_dir}", file=sys.stderr)
        return 1

    # Parse all xacro and urdf (exclude sensei_full which only includes macros)
    all_links, all_joints = {}, {}
    for p in sorted(urdf_dir.iterdir()):
        if p.suffix not in (".xacro", ".urdf"):
            continue
        if "sensei_full" in p.name and "sensei_full" == p.stem.split(".")[0]:
            continue
        try:
            li, jo = parse_urdf_file(p)
            all_links.update(li)
            all_joints.update(jo)
        except Exception as e:
            print(f"Warning: {p}: {e}", file=sys.stderr)

    # Load existing params so we don’t overwrite robot/xl430/bracket/export_parts
    out = args.output
    if out.exists():
        with open(out) as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}

    data["urdf_links"] = all_links
    data["urdf_joints"] = all_joints

    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    print(f"Wrote urdf_links ({len(all_links)}) and urdf_joints ({len(all_joints)}) to {out}")

    # --- Write cad/urdf_geometry.json for Fusion GenerateFromParams ---
    cad_dir = out.parent
    geom = []
    for name, link in all_links.items():
        resolved = _resolve_link_name(name)
        g = link.get("geometry") or {}
        t = g.get("type")
        if not t:
            continue
        entry = {"name": resolved, "type": t, "origin_xyz": link.get("origin_xyz") or [0, 0, 0]}
        if t == "box":
            s = g.get("size")
            if s and len(s) >= 3:
                entry["size"] = s
            else:
                continue
        elif t == "cylinder":
            if "radius" in g and "length" in g:
                entry["radius"] = g["radius"]
                entry["length"] = g["length"]
            else:
                continue
        elif t == "sphere":
            if "radius" in g:
                entry["radius"] = g["radius"]
            else:
                continue
        else:
            continue
        geom.append(entry)

    joints_out = []
    for jname, j in all_joints.items():
        parent = _resolve_joint_ref((j.get("parent") or "").strip())
        child = _resolve_joint_ref((j.get("child") or "").strip())
        if not parent or not child:
            continue
        joints_out.append({
            "name": jname,
            "type": j.get("type") or "revolute",
            "parent": parent,
            "child": child,
            "origin_xyz": j.get("origin_xyz") or [0.0, 0.0, 0.0],
            "origin_rpy": j.get("origin_rpy") or [0.0, 0.0, 0.0],
            "axis": j.get("axis") or [0.0, 0.0, 1.0],
        })

    geom_path = cad_dir / "urdf_geometry.json"
    with open(geom_path, "w") as f:
        json.dump({"parts": geom, "joints": joints_out}, f, indent=2)
    print("Wrote {} parts and {} joints to {}".format(len(geom), len(joints_out), geom_path))

    # --- Update cad/export_config.json with link → STL path ---
    export_cfg_path = cad_dir / "export_config.json"
    if export_cfg_path.exists():
        with open(export_cfg_path) as f:
            export_cfg = json.load(f)
    else:
        export_cfg = {"assembly": {}, "parts": {}}
    parts = export_cfg.setdefault("parts", {})
    # drop xacro-style keys from previous runs
    for k in list(parts.keys()):
        if isinstance(k, str) and k.startswith("${") and "}" in k:
            del parts[k]
    for name in all_links:
        r = _resolve_link_name(name)
        if r not in parts:
            sub = _link_to_subdir(r)
            parts[r] = "parts/{}/{}.stl".format(sub, r)
    with open(export_cfg_path, "w") as f:
        json.dump(export_cfg, f, indent=2)
    print(f"Updated {export_cfg_path} with link export paths")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
