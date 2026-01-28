# -*- coding: utf-8 -*-
"""
Fusion 360 script: position components from URDF joint tree.

Reads cad/urdf_geometry.json "joints", computes each link's world transform
from the base_link, and sets occurrence.transform2. base_link stays at identity.
Requires BodiesToComponents first. Units: URDF meters -> mm.

Install: Scripts and Add-Ins > Add Script from your computer > select this folder.
"""

import adsk.core
import adsk.fusion
import json
import math
import os
from collections import deque

M_TO_MM = 1000.0


def _get_cad_root():
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
    except Exception:
        script_dir = os.getcwd()
    return os.path.normpath(os.path.join(script_dir, "..", ".."))


def _rpy_to_rotation_matrix(rpy):
    """Roll-pitch-yaw (rad) to 3x3 in row-major. R = Rz(yaw)*Ry(pitch)*Rx(roll)."""
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Rx, Ry, Rz
    # R = Rz * Ry * Rx
    R00 = cz * cy
    R01 = cz * sy * sx - sz * cx
    R02 = cz * sy * cx + sz * sx
    R10 = sz * cy
    R11 = sz * sy * sx + cz * cx
    R12 = sz * sy * cx - cz * sx
    R20 = -sy
    R21 = cy * sx
    R22 = cy * cx
    return [R00, R01, R02, R10, R11, R12, R20, R21, R22]


def _build_matrix4(origin_xyz, origin_rpy):
    """4x4 row-major: [R|t; 0 0 0 1]. xyz in m, converted to mm."""
    R = _rpy_to_rotation_matrix(origin_rpy)
    tx = origin_xyz[0] * M_TO_MM
    ty = origin_xyz[1] * M_TO_MM
    tz = origin_xyz[2] * M_TO_MM
    return [
        R[0], R[1], R[2], tx,
        R[3], R[4], R[5], ty,
        R[6], R[7], R[8], tz,
        0.0, 0.0, 0.0, 1.0,
    ]


def _multiply4x4(a, b):
    """a and b as 16-element row-major; return a*b."""
    out = [0.0] * 16
    for i in range(4):
        for j in range(4):
            out[i * 4 + j] = sum(a[i * 4 + k] * b[k * 4 + j] for k in range(4))
    return out


def _find_occurrence_by_component_name(root, name):
    for occ in root.allOccurrences:
        try:
            if occ.component.name == name:
                return occ
        except Exception:
            pass
    return None


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active design. Open a design and run again.")
            return

        cad_root = _get_cad_root()
        path = os.path.join(cad_root, "urdf_geometry.json")
        if not os.path.isfile(path):
            ui.messageBox("urdf_geometry.json not found. Run: python3 scripts/urdf_to_cad_params.py")
            return

        with open(path) as f:
            data = json.load(f)
        joints = data.get("joints") or []

        # world transform per link; base_link = identity
        link_transforms = {"base_link": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}

        # BFS from base_link: for each joint, child_world = parent_world * T_joint
        q = deque(["base_link"])
        seen = {"base_link"}
        while q:
            parent = q.popleft()
            pT = link_transforms.get(parent)
            if pT is None:
                continue
            for j in joints:
                if j.get("parent") != parent:
                    continue
                child = j.get("child") or ""
                if not child or child in seen:
                    continue
                seen.add(child)
                T = _build_matrix4(
                    j.get("origin_xyz") or [0, 0, 0],
                    j.get("origin_rpy") or [0, 0, 0],
                )
                link_transforms[child] = _multiply4x4(pT, T)
                q.append(child)

        root = design.rootComponent
        placed = 0
        for link_name, arr in link_transforms.items():
            if link_name == "base_link":
                continue
            occ = _find_occurrence_by_component_name(root, link_name)
            if occ is None:
                continue
            try:
                m = adsk.core.Matrix3D.create()
                m.setWithArray(arr)
                occ.transform2 = m
                placed += 1
            except Exception:
                pass

        ui.messageBox("PlaceFromUrdf: positioned %d components. Add Joints manually or use fusion360-urdf-ros2." % placed)

    except Exception as e:
        if ui:
            ui.messageBox("Error: %s" % e)
        raise
