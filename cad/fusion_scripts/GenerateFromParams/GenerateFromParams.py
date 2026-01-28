# -*- coding: utf-8 -*-
"""
Fusion 360 script: generate bodies from cad/urdf_geometry.json.

Reads cad/urdf_geometry.json (from scripts/urdf_to_cad_params.py), creates
box/cylinder/sphere (sphere as cylinder stand-in) bodies in the root component.
Design units must be mm. Run after urdf_to_cad_params. Export via export_to_cad.

Install: Scripts and Add-Ins > Add Script from your computer > select this folder.
"""

import adsk.core
import adsk.fusion
import json
import os

# URDF units are meters; design is mm
M_TO_MM = 1000.0


def _vi(v):
    return adsk.core.ValueInput.createByReal(v)


def _get_cad_root():
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
    except Exception:
        script_dir = os.getcwd()
    return os.path.normpath(os.path.join(script_dir, "..", ".."))


def _create_box(comp, size_m, name):
    """size_m: [x,y,z] in meters. Creates box centered at origin on XY, extruded +Z."""
    sx, sy, sz = size_m[0] * M_TO_MM, size_m[1] * M_TO_MM, size_m[2] * M_TO_MM
    hx, hy = sx / 2.0, sy / 2.0
    sk = comp.sketches.add(comp.xYConstructionPlane)
    lines = sk.sketchCurves.sketchLines
    lines.addByTwoPoints(adsk.core.Point3D.create(-hx, -hy, 0), adsk.core.Point3D.create(hx, -hy, 0))
    lines.addByTwoPoints(adsk.core.Point3D.create(hx, -hy, 0), adsk.core.Point3D.create(hx, hy, 0))
    lines.addByTwoPoints(adsk.core.Point3D.create(hx, hy, 0), adsk.core.Point3D.create(-hx, hy, 0))
    lines.addByTwoPoints(adsk.core.Point3D.create(-hx, hy, 0), adsk.core.Point3D.create(-hx, -hy, 0))
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.setOneSideExtent(
        adsk.fusion.DistanceExtentDefinition.create(_vi(sz)),
        adsk.fusion.ExtentDirections.PositiveExtentDirection,
    )
    bod = comp.features.extrudeFeatures.add(ext).bodies.item(0)
    bod.name = name
    return bod


def _create_cylinder(comp, radius_m, length_m, name):
    """Circle on XY, extrude +Z."""
    r = radius_m * M_TO_MM
    L = length_m * M_TO_MM
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), r)
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.setOneSideExtent(
        adsk.fusion.DistanceExtentDefinition.create(_vi(L)),
        adsk.fusion.ExtentDirections.PositiveExtentDirection,
    )
    bod = comp.features.extrudeFeatures.add(ext).bodies.item(0)
    bod.name = name
    return bod


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
        parts = data.get("parts") or []

        comp = design.rootComponent
        created = 0
        for p in parts:
            name = p.get("name") or ""
            typ = p.get("type") or ""
            if not name or not typ:
                continue
            try:
                if typ == "box":
                    sz = p.get("size")
                    if sz and len(sz) >= 3:
                        _create_box(comp, sz, name)
                        created += 1
                elif typ == "cylinder":
                    r = p.get("radius")
                    L = p.get("length")
                    if r is not None and L is not None:
                        _create_cylinder(comp, r, L, name)
                        created += 1
                elif typ == "sphere":
                    r = p.get("radius")
                    if r is not None:
                        _create_cylinder(comp, r, 2.0 * r, name)
                        created += 1
            except Exception as e:
                ui.messageBox("Part '{}' failed: {}".format(name, e))

        ui.messageBox("GenerateFromParams: created {} bodies. Export via export_to_cad.".format(created))

    except Exception as e:
        if ui:
            ui.messageBox("Error: {}".format(e))
        raise
