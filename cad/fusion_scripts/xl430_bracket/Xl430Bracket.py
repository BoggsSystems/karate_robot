# -*- coding: utf-8 -*-
"""
Fusion 360 script: parametric XL430 servo bracket.

Creates a tray-style bracket: outer block with pocket for the servo body and
4x M2.5 mounting holes. Dimensions follow docs/frame_cad_design.md (wall, clearance,
cable channel optional). Design units should be mm.

Install: Scripts and Add-Ins > Add Script from your computer > select this folder.
Run: Scripts and Add-Ins > Xl430Bracket.

Output: body "xl430_bracket" in the root component. Export via export_to_cad.
"""

import adsk.core
import adsk.fusion


# --- Parameters (mm). Match cad/sensei_params.yaml / frame_cad_design.md ---
SERVO_W = 28.5
SERVO_D = 46.5
SERVO_H = 34
WALL = 3
CLEARANCE = 1
CABLE_D = 8
HORN_R = 12
M25_HOLE_D = 2.7
HOLE_HW = 10   # half-width for M2.5 pattern
HOLE_HD = 15   # half-depth for M2.5 pattern


def _vi(v):
    return adsk.core.ValueInput.createByReal(v)


def _rect(sketch, cx, cy, w, h):
    """Draw closed rectangle centered at (cx,cy) with half-width w, half-height h."""
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
            ui.messageBox('No active design. Open a design and run again.')
            return

        # Work in the root component (Part Design has no .components on root)
        comp = design.rootComponent

        # 1) Outer block
        ow = (SERVO_W + 2 * CLEARANCE + 2 * WALL) / 2.0
        od = (SERVO_D + 2 * CLEARANCE + 2 * WALL) / 2.0
        height = SERVO_H + 2 * WALL

        sk1 = comp.sketches.add(comp.xYConstructionPlane)
        _rect(sk1, 0, 0, ow, od)
        prof = sk1.profiles.item(0)
        ext_input = comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )
        ext_input.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(height)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection
        )
        ext1 = comp.features.extrudeFeatures.add(ext_input)
        body = ext1.bodies.item(0)
        body.name = 'xl430_bracket'

        # 2) Pocket (inner cut from top)
        iw = (SERVO_W + 2 * CLEARANCE) / 2.0
        id_ = (SERVO_D + 2 * CLEARANCE) / 2.0
        pocket_depth = SERVO_H

        plane_inp = comp.constructionPlanes.createInput()
        plane_inp.setByOffset(comp.xYConstructionPlane, _vi(height))
        plane = comp.constructionPlanes.add(plane_inp)
        sk2 = comp.sketches.add(plane)
        _rect(sk2, 0, 0, iw, id_)
        prof2 = sk2.profiles.item(0)
        cut_input = comp.features.extrudeFeatures.createInput(
            prof2, adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        cut_input.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(pocket_depth)),
            adsk.fusion.ExtentDirections.NegativeExtentDirection
        )
        cut_input.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(cut_input)

        # 3) M2.5 mounting holes (bottom face, through)
        sk3 = comp.sketches.add(comp.xYConstructionPlane)
        r = (M25_HOLE_D / 2.0)
        for dx in (-HOLE_HW, HOLE_HW):
            for dy in (-HOLE_HD, HOLE_HD):
                _circle(sk3, dx, dy, r)

        # One profile per circle; we need to cut all. Fusion can do multiple profiles.
        # If the 4 circles are separate, we may need 4 cut operations or a single
        # with multiple profiles. createInput(profile, Cut) takes one profile.
        # We can collect profiles: sk3.profiles gives all. For 4 circles we get 4.
        holes_input = comp.features.extrudeFeatures.createInput(
            sk3.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        holes_input.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(50)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection
        )
        holes_input.targetBodies = adsk.core.ObjectCollection.createWithArray([body])

        # Add one hole; then repeat for the other 3 profiles
        comp.features.extrudeFeatures.add(holes_input)
        for i in range(1, 4):
            hi = comp.features.extrudeFeatures.createInput(
                sk3.profiles.item(i), adsk.fusion.FeatureOperations.CutFeatureOperation
            )
            hi.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(_vi(50)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection
            )
            hi.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
            comp.features.extrudeFeatures.add(hi)

        ui.messageBox('xl430_bracket created. Export via export_to_cad.')

    except Exception as e:
        if ui:
            ui.messageBox(f'Error: {e}')
        raise
