# -*- coding: utf-8 -*-
"""
Fusion 360 script: limb tubes with open cable channel.

Reads cad/urdf_geometry.json, creates a cylinder (radius, length) with an open
C-channel (8x4mm) along the back (-X) for each limb link. Matches the
exposed-electronics reference: visible cable runs. Design units: mm.

Install: Scripts and Add-Ins > Add Script from your computer > select this folder.
Run: Scripts and Add-Ins > LimbTubeWithChannel.

Output: one body per link (l_upper_arm_link, l_forearm_link, ...), named as the link.
"""

import adsk.core
import adsk.fusion
import json
import os

M_TO_MM = 1000.0
CHANNEL_W = 8.0   # mm, half = 4
CHANNEL_D = 4.0   # mm

LIMB_TUBE_LINKS = [
    "l_upper_arm_link", "l_forearm_link",
    "r_upper_arm_link", "r_forearm_link",
    "l_thigh_link", "l_shin_link",
    "r_thigh_link", "r_shin_link",
]


def _vi(v):
    return adsk.core.ValueInput.createByReal(v)


def _get_cad_root():
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
    except Exception:
        script_dir = os.getcwd()
    return os.path.normpath(os.path.join(script_dir, "..", ".."))


def _create_cylinder(comp, radius_mm, length_mm, name):
    """Cylinder along Z: circle on XY, extrude +Z."""
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0), radius_mm
    )
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures.createInput(
        prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    ext.setOneSideExtent(
        adsk.fusion.DistanceExtentDefinition.create(_vi(length_mm)),
        adsk.fusion.ExtentDirections.PositiveExtentDirection,
    )
    bod = comp.features.extrudeFeatures.add(ext).bodies.item(0)
    bod.name = name
    return bod


def _add_channel(comp, body, radius_mm, length_mm):
    """Cut C-channel on -X side: 8mm wide (Y), 4mm deep (X), full length (Z)."""
    # Plane at X = -radius_mm (outer surface). Sketch: rect Y [-4,4], Z [0, L].
    # Extrude +X by 4mm (into the body).
    plane_in = comp.constructionPlanes.createInput()
    plane_in.setByOffset(comp.yZConstructionPlane, _vi(-radius_mm))
    pl = comp.constructionPlanes.add(plane_in)
    sk = comp.sketches.add(pl)
    # In YZ: Y is first param, Z second. Rect centered (0, length_mm/2), half (CHANNEL_W/2, length_mm/2)
    lines = sk.sketchCurves.sketchLines
    hw = CHANNEL_W / 2.0
    hl = length_mm / 2.0
    p0 = adsk.core.Point3D.create(-hw, 0, 0)
    p1 = adsk.core.Point3D.create(hw, 0, 0)
    p2 = adsk.core.Point3D.create(hw, length_mm, 0)
    p3 = adsk.core.Point3D.create(-hw, length_mm, 0)
    lines.addByTwoPoints(p0, p1)
    lines.addByTwoPoints(p1, p2)
    lines.addByTwoPoints(p2, p3)
    lines.addByTwoPoints(p3, p0)
    prof = sk.profiles.item(0)
    cut = comp.features.extrudeFeatures.createInput(
        prof, adsk.fusion.FeatureOperations.CutFeatureOperation
    )
    cut.setOneSideExtent(
        adsk.fusion.DistanceExtentDefinition.create(_vi(CHANNEL_D)),
        adsk.fusion.ExtentDirections.PositiveExtentDirection,
    )
    cut.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
    comp.features.extrudeFeatures.add(cut)


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active design. Open a design and run again.")
            return

        cad_root = _get_cad_root()
        geom_path = os.path.join(cad_root, "urdf_geometry.json")
        if not os.path.isfile(geom_path):
            ui.messageBox("urdf_geometry.json not found. Run: python3 scripts/urdf_to_cad_params.py")
            return

        with open(geom_path) as f:
            data = json.load(f)
        parts = {p.get("name"): p for p in (data.get("parts") or []) if p.get("name")}

        comp = design.rootComponent
        created = 0
        for name in LIMB_TUBE_LINKS:
            p = parts.get(name)
            if not p or (p.get("type") or "") != "cylinder":
                continue
            r = p.get("radius")
            L = p.get("length")
            if r is None or L is None:
                continue
            r_mm = r * M_TO_MM
            L_mm = L * M_TO_MM
            try:
                body = _create_cylinder(comp, r_mm, L_mm, name)
                _add_channel(comp, body, r_mm, L_mm)
                created += 1
            except Exception as e:
                ui.messageBox("LimbTubeWithChannel '{}' failed: {}".format(name, e))

        ui.messageBox("LimbTubeWithChannel: created {} bodies. Export via export_to_cad.".format(created))

    except Exception as e:
        if ui:
            ui.messageBox("Error: {}".format(e))
        raise
