# -*- coding: utf-8 -*-
"""
Fusion 360 script: parametric torso with exposed electronics bay.

Creates torso_main: open-front torso, electronics cavity, Pi boss (shelf + 4 standoffs),
battery tray, and cable passthroughs. Matches docs/frame_cad_design.md and the
exposed-electronics reference image. Design units: mm.

Install: Scripts and Add-Ins > Add Script from your computer > select this folder.
Run: Scripts and Add-Ins > TorsoMain.

Output: body "torso_main". Export via export_to_cad to parts/torso/torso_main.stl.
"""

import adsk.core
import adsk.fusion

# --- Defaults (match cad/detail_params.yaml) ---
OUTER = (200.0, 140.0, 100.0)   # W, D, H (X, Y, Z); origin at bottom-center
CAVITY = (100.0, 80.0, 40.0)    # internal bay
WALL = 3.0
# Pi: 85x56, hole spacing 58x49, standoff 10mm, M2.5
PI_HOLE_SPACING = (58.0, 49.0)
PI_STANDOFF = 10.0
PI_HOLE_D = 2.7
# Battery tray well
BATTERY_TRAY = (105.0, 35.0, 20.0)
CABLE_D = 8.0


def _vi(v):
    return adsk.core.ValueInput.createByReal(v)


def _rect(sketch, cx, cy, w, h):
    """Closed rectangle centered at (cx,cy), half-width w, half-height h."""
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
        Wx, Dy, Hz = OUTER
        Cx, Cy, Cz = CAVITY
        hW, hD, hZ = Wx / 2.0, Dy / 2.0, Hz

        # 1) Outer box: X [-hW,hW], Y [-hD,hD], Z [0, Hz]. Sketch XY, extrude +Z.
        sk0 = comp.sketches.add(comp.xYConstructionPlane)
        _rect(sk0, 0, 0, hW, hD)
        ext0 = comp.features.extrudeFeatures.createInput(
            sk0.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )
        ext0.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(Hz)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        body = comp.features.extrudeFeatures.add(ext0).bodies.item(0)

        # 2) Cavity cut: X [-Cx/2,Cx/2], Y [-Cy/2,Cy/2], Z [WALL, WALL+Cz].
        #    Sketch on XZ at Y=-Cy/2, rect Cx x Cz at Z=WALL; extrude +Y by Cy.
        plane_cav = comp.constructionPlanes.createInput()
        plane_cav.setByOffset(comp.xZConstructionPlane, _vi(-Cy / 2.0))
        pl = comp.constructionPlanes.add(plane_cav)
        sk_cav = comp.sketches.add(pl)
        _rect(sk_cav, 0, WALL + Cz / 2.0, Cx / 2.0, Cz / 2.0)
        ext_cav = comp.features.extrudeFeatures.createInput(
            sk_cav.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ext_cav.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(Cy)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        ext_cav.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(ext_cav)

        # 3) Open front: remove wall Y=[Cy/2, hD]. Sketch on XZ at Y=hD, full X and Z; extrude -Y (Cy/2 - (-hD)) = hD - Cy/2.
        plane_front = comp.constructionPlanes.createInput()
        plane_front.setByOffset(comp.xZConstructionPlane, _vi(hD))
        plf = comp.constructionPlanes.add(plane_front)
        sk_front = comp.sketches.add(plf)
        _rect(sk_front, 0, Hz / 2.0, hW, Hz / 2.0)
        ext_front = comp.features.extrudeFeatures.createInput(
            sk_front.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ext_front.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(hD - Cy / 2.0)),
            adsk.fusion.ExtentDirections.NegativeExtentDirection,
        )
        ext_front.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(ext_front)

        # 4) Pi shelf on cavity back (Y=-Cy/2): plate Y [-Cy/2, -Cy/2+2], X [-Cx/2,Cx/2], Z [15, 75]. New body then Join.
        plane_shelf = comp.constructionPlanes.createInput()
        plane_shelf.setByOffset(comp.xZConstructionPlane, _vi(-Cy / 2.0))
        pls = comp.constructionPlanes.add(plane_shelf)
        sk_shelf = comp.sketches.add(pls)
        _rect(sk_shelf, 0, 45.0, Cx / 2.0, 30.0)
        ext_shelf = comp.features.extrudeFeatures.createInput(
            sk_shelf.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )
        ext_shelf.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(2.0)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        shelf_body = comp.features.extrudeFeatures.add(ext_shelf).bodies.item(0)

        comb_in = comp.features.combineFeatures.createInput(body, adsk.core.ObjectCollection.createWithArray([shelf_body]))
        comb_in.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        comp.features.combineFeatures.add(comb_in)

        # 5) Pi standoffs: 4 cylinders on shelf front (Y=-Cy/2+2), 10mm tall. Pi hole pattern 58x49 -> +/-29, +/-24.5.
        #    Shelf top at Y=-Cy/2+2. Sketch on plane at that Y: 4 circles. Extrude +Y 10. Then Join.
        plane_so = comp.constructionPlanes.createInput()
        plane_so.setByOffset(comp.xZConstructionPlane, _vi(-Cy / 2.0 + 2.0))
        plo = comp.constructionPlanes.add(plane_so)
        sk_so = comp.sketches.add(plo)
        hx, hz = PI_HOLE_SPACING[0] / 2.0, PI_HOLE_SPACING[1] / 2.0
        for (dx, dz) in [(hx, hz), (-hx, hz), (-hx, -hz), (hx, -hz)]:
            _circle(sk_so, dx, 50.0 + dz, 3.0)
        ext_so = comp.features.extrudeFeatures.createInput(
            sk_so.profiles.item(0), adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )
        ext_so.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(PI_STANDOFF)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        so_body = comp.features.extrudeFeatures.add(ext_so).bodies.item(0)
        for i in range(1, 4):
            ex = comp.features.extrudeFeatures.createInput(
                sk_so.profiles.item(i), adsk.fusion.FeatureOperations.NewBodyFeatureOperation
            )
            ex.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(_vi(PI_STANDOFF)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection,
            )
            sob = comp.features.extrudeFeatures.add(ex).bodies.item(0)
            so_body = sob  # keep last; we need to join all 4
        # Simpler: one extrusion with 4 profiles. Fusion createInput(profile) takes one. We can try createInput with multiple?
        # Per Xl430Bracket, they did 4 separate extrusions for holes. For 4 standoffs we can do 4 separate NewBody and then 4 Joins.
        # Above I did 4 NewBody - that creates 4 bodies. We need to join all into body. Let me redo: do one extrusion with the first circle, get body; then for 1..3 do another extrusion NewBody and Join to body. So we have one standoff body after 4 joins. Then Join standoffs to main body.
        # Actually the loop above overwrites so_body each time and only the last is kept. And we're creating 4 separate bodies. Let me instead create 4 standoffs as one extrude: in Fusion, if we have 4 circles in one sketch, the profiles might be 4. We'd need to extrude all 4. The API might allow a single extrude with multiple profiles producing one body. I'll check: ExtrudeFeatures.createInput(Profile, operation) - Profile is one. So we need 4 extrusions. For Join: we need to join 4 bodies to the main. Let me create them and then do 4 combine/joins. I need to fix the loop - we shouldn't overwrite. And the first extrusion already created one body. So we have 4 bodies from 4 extrusions. We can do: main = body; for each of the 4 standoff bodies, combine(main, standoff, Join). The first combine merges standoff1 into main. The second combine: we need the next standoff body. The 4 extrusions produce 4 bodies. I need to collect them. The first: ext_so = add(ext_so); so_bodies = [ext_so.bodies.item(0)]. Then for i in 1..3: ext_i = add(createInput(profiles.item(i), NewBody)); so_bodies.append(ext_i.bodies.item(0)). Then for sb in so_bodies: combine(main, sb, Join). After first join, 'body' is still the same object and now includes the first standoff. Good.
        # Fix: create only one standoff body with 4 cylinders. We can do that by having 4 circles in one sketch and then one extrude - in some CAD APIs that creates one body with 4 lumps. In Fusion, one profile per extrude. So we need 4 extrude-add and 4 joins. Let me simplify: do one extrude with profile 0, get body A. Second with profile 1, get B. Join A and B -> A. Third, get C. Join A and C -> A. Fourth, get D. Join A and D -> A. Then Join A into main body. So we need to track the "current" standoff aggregate. I'll do: st_bodies = []; for i in 0..3: e=add(createInput(profiles.item(i), NewBody)); st_bodies.append(e.bodies.item(0)). Then for sb in st_bodies: cin=createInput(body, [sb]); cin.operation=Join; add(cin). That should work.
        st_bodies = [comp.features.extrudeFeatures.add(ext_so).bodies.item(0)]
        for i in range(1, 4):
            ei = comp.features.extrudeFeatures.createInput(
                sk_so.profiles.item(i), adsk.fusion.FeatureOperations.NewBodyFeatureOperation
            )
            ei.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(_vi(PI_STANDOFF)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection,
            )
            st_bodies.append(comp.features.extrudeFeatures.add(ei).bodies.item(0))
        for sb in st_bodies:
            ci = comp.features.combineFeatures.createInput(body, adsk.core.ObjectCollection.createWithArray([sb]))
            ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            comp.features.combineFeatures.add(ci)

        # 6) Pi standoff through-holes: 4 circles, M2.5. Sketch on standoff top (Y=-Cy/2+2+10). Extrude -Y 15 (Cut).
        plane_hol = comp.constructionPlanes.createInput()
        plane_hol.setByOffset(comp.xZConstructionPlane, _vi(-Cy / 2.0 + 2.0 + PI_STANDOFF))
        plh = comp.constructionPlanes.add(plane_hol)
        sk_hol = comp.sketches.add(plh)
        for (dx, dz) in [(hx, hz), (-hx, hz), (-hx, -hz), (hx, -hz)]:
            _circle(sk_hol, dx, 50.0 + dz, PI_HOLE_D / 2.0)
        for i in range(4):
            exh = comp.features.extrudeFeatures.createInput(
                sk_hol.profiles.item(i), adsk.fusion.FeatureOperations.CutFeatureOperation
            )
            exh.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(_vi(15.0)),
                adsk.fusion.ExtentDirections.NegativeExtentDirection,
            )
            exh.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
            comp.features.extrudeFeatures.add(exh)

        # 7) Battery tray: well 105x35x20 from cavity floor Z=WALL. Sketch on Z=WALL, rect 105x35 centered. Extrude +Z 20 (Cut).
        plane_bt = comp.constructionPlanes.createInput()
        plane_bt.setByOffset(comp.xYConstructionPlane, _vi(WALL))
        plb = comp.constructionPlanes.add(plane_bt)
        sk_bt = comp.sketches.add(plb)
        _rect(sk_bt, 0, 0, BATTERY_TRAY[0] / 2.0, BATTERY_TRAY[1] / 2.0)
        ext_bt = comp.features.extrudeFeatures.createInput(
            sk_bt.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ext_bt.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(BATTERY_TRAY[2])),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        ext_bt.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(ext_bt)

        # 8) Cable passthroughs: 8mm dia. Top (0,15,Hz), Left (-hW+5,0,50), Right (hW-5,0,50), Bottom (0,-15,5).
        r = CABLE_D / 2.0
        # Top: sketch XZ at Y=15, circle (0,Hz-5); extrude -Z 50 (down into cavity)
        pl_top = comp.constructionPlanes.createInput()
        pl_top.setByOffset(comp.xZConstructionPlane, _vi(15.0))
        pt = comp.constructionPlanes.add(pl_top)
        sk_t = comp.sketches.add(pt)
        _circle(sk_t, 0, Hz - 5.0, r)
        ex_t = comp.features.extrudeFeatures.createInput(
            sk_t.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ex_t.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(50.0)),
            adsk.fusion.ExtentDirections.NegativeExtentDirection,
        )
        ex_t.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(ex_t)
        # Left: sketch YZ at X=-hW+2, circle (0,50); extrude +X 20
        pl_le = comp.constructionPlanes.createInput()
        pl_le.setByOffset(comp.yZConstructionPlane, _vi(-hW + 2.0))
        ple = comp.constructionPlanes.add(pl_le)
        sk_le = comp.sketches.add(ple)
        _circle(sk_le, 0, 50.0, r)
        ex_le = comp.features.extrudeFeatures.createInput(
            sk_le.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ex_le.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(20.0)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        ex_le.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(ex_le)
        # Right: YZ at X=hW-2, extrude -X 20
        pl_ri = comp.constructionPlanes.createInput()
        pl_ri.setByOffset(comp.yZConstructionPlane, _vi(hW - 2.0))
        plr = comp.constructionPlanes.add(pl_ri)
        sk_ri = comp.sketches.add(plr)
        _circle(sk_ri, 0, 50.0, r)
        ex_ri = comp.features.extrudeFeatures.createInput(
            sk_ri.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ex_ri.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(20.0)),
            adsk.fusion.ExtentDirections.NegativeExtentDirection,
        )
        ex_ri.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(ex_ri)
        # Bottom: sketch XY at Z=0, circle (0,0); extrude +Z 20
        sk_bo = comp.sketches.add(comp.xYConstructionPlane)
        _circle(sk_bo, 0, 0, r)
        ex_bo = comp.features.extrudeFeatures.createInput(
            sk_bo.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ex_bo.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(_vi(20.0)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection,
        )
        ex_bo.targetBodies = adsk.core.ObjectCollection.createWithArray([body])
        comp.features.extrudeFeatures.add(ex_bo)

        body.name = "torso_main"
        ui.messageBox("torso_main created. Export via export_to_cad.")

    except Exception as e:
        if ui:
            ui.messageBox("Error: {}".format(e))
        raise
