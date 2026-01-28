# -*- coding: utf-8 -*-
"""
Fusion 360 script: convert all bodies in the root to components.

Each body becomes its own component; the body's name is applied to the new
component. Run after GenerateFromParams (and optional Xl430Bracket). Follow
with PlaceFromUrdf to position occurrences from URDF.

Install: Scripts and Add-Ins > Add Script from your computer > select this folder.
"""

import adsk.core
import adsk.fusion


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active design. Open a design and run again.")
            return

        root = design.rootComponent
        bodies = [b for b in root.bRepBodies]
        created = 0
        errors = []
        for body in bodies:
            try:
                # Only skip if explicitly transient (createComponent invalid then)
                is_trans = getattr(body, "isTransient", False) or getattr(body, "IsTransient", False)
                if is_trans:
                    continue
                name = body.name or ("Body_%d" % created)
                rb = body.createComponent()
                if rb and rb.parentComponent:
                    rb.parentComponent.name = name
                    created += 1
            except Exception as e:
                err = str(e)
                errors.append("%s: %s" % (getattr(body, "name", "?"), err))

        msg = "BodiesToComponents: created %d components. Run PlaceFromUrdf to position." % created
        is_part_design_error = errors and (
            "Part Design" in (errors[0] or "") and "one component" in (errors[0] or "")
        )
        if is_part_design_error:
            msg = (
                "Part Design allows only one component.\n\n"
                "Do this instead:\n"
                "1. File > New Design — choose 'Assembly' or 'Empty Assembly' if offered.\n"
                "2. In that new design, run GenerateFromParams (then optional Xl430Bracket).\n"
                "3. Run BodiesToComponents, then PlaceFromUrdf.\n\n"
                "You can keep your current Part as reference; run the pipeline in the Assembly."
            )
        elif errors:
            msg += "\n\nFailures: " + "; ".join(errors[:3])
            if len(errors) > 3:
                msg += " (+%d more)" % (len(errors) - 3)
        ui.messageBox(msg)

    except Exception as e:
        if ui:
            ui.messageBox("Error: %s" % e)
        raise
