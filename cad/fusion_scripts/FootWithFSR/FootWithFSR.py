# -*- coding: utf-8 -*-
"""
Fusion 360 script: foot with FSR pockets on the sole.

Creates foot_with_fsr: 100x60x30 mm block with two circular FSR pockets (10mm dia,
2mm deep) on the UNDERSIDE (sole): ball 25mm from toe, heel 25mm from heel.
Matches docs/frame_cad_design.md and the reference image. Design units: mm.

Install: Scripts and Add-Ins > Add Script from your computer > select this folder.
Run: Scripts and Add-Ins > FootWithFSR.

Output: body "foot_with_fsr". Duplicate for l_foot_link / r_foot_link, or export
to parts/leg_left/foot_L.stl and parts/leg_right/foot_R.stl.
"""

import adsk.core
import adsk.fusion

# --- Defaults (match cad/detail_params.yaml) ---
Lx, Wy, Hz = 100.0, 60.0, 30.0
FSR_D = 10.0
FSR_POCKET_DEPTH = 2.0
BALL_OFFSET = 25.0   # from toe (+X)
HEEL_OFFSET = 25.0   # from heel (-X)


def _vi(v):
    return adsk.core.ValueInput.createByReal(v)


def _rect(sketch, cx, cy, w, h):
    lines = sketch.sketchCurves.sketchLines
    x0, x1 = cx - w, cx + w
    y0, y1 = cy - h, cy + h
    p0 = adsk.core.Point3D.create(x0, y0, 0)
    p1 = adsk.core.Point3D.create(x1, y0, 0)
    p2 = adsk.core.Point3D.create(x1, y1, 0)
    p3 = adsk.core.Point3D.create(x0, y1, 0)
    lines.addByTwoPoints(p0, p1)
    lines.addByTwoPoints(p1, p2)
    lines.addByTwoPoints(p2, p3)
    lines.addByTwoPoints(p3, p0)


def _circle(sketch, cx, cy, r):
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(cx, cy, 0), r
    )


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active design. Open a design and run again.")
            return

        comp = design.rootComponent
        hx, hy = Lx / 2.0, Wy / 2.0

        # 1) Base box: X [-hx,hx], Y [-hy,hy], Z [0, Hz]. Sketch XY, extrude +Z.
        sk0 = comp.sketches.add(comp.xYConstructionPlane)
        _rect(sk0, 0, 0, hx, hy)
        ext0 = comp.features.extrudeFeatures.createInput(
            sk0.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )
        ext0.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(Hz)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        body = comp.features.extrudeFeatures.add(ext0).bodies.item(0)

        # 2) FSR pockets on underside (Z=0): ball at X=+hx-BALL_OFFSET=+25, heel at X=-hx+HEEL_OFFSET=-25, Y=0.
        sk1 = comp.sketches.add(comp.xYConstructionPlane)
        _circle(sk1, hx - BALL_OFFSET, 0, FSR_D / 2.0)
        _circle(sk1, -hx + HEEL_OFFSET, 0, FSR_D / 2.0)
        for i in range(2):
            cut = comp.features.extrudeFeatures.createInput(
                sk1.profiles.item(i), adsk.fusion.FeatureOperations.CutFeatureOperation
            )
            cut.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(_vi(FSR_POCKET_DEPTH)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection,
            )
            cut.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
            comp.features.extrudeFeatures.add(cut)

        body.name = "foot_with_fsr"
        ui.messageBox("foot_with_fsr created. Duplicate for L/R or export via export_to_cad.")

    except Exception as e:
        if ui:
            ui.messageBox("Error: {}".format(e))
        raise
