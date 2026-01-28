# -*- coding: utf-8 -*-
"""
Fusion 360 script: export parts to STL and assembly to STEP.

Install: In Fusion 360, UTILITIES > Scripts and Add-Ins (Shift+S) > Add-Ins >
  Create > Add-in from your computer > select the folder containing this script
  (export_to_cad, or the parent cad/fusion_scripts). Then run "ExportToCad" from
  the list.

Alternatively, place this folder in:
  - Windows: %%APPDATA%%\\Autodesk\\Autodesk Fusion 360\\API\\Scripts\\
  - macOS: ~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/

Expects cad/export_config.json and writes under cad/ (assemblies/, parts/).
CAD root is assumed at: this_script/../../ (when run from cad/fusion_scripts/export_to_cad/).
"""

import adsk.core
import adsk.fusion
import json
import os
import sys


def get_cad_root():
    try:
        if '__file__' in dir():
            script_dir = os.path.dirname(os.path.abspath(__file__))
        else:
            script_dir = os.getcwd()
    except Exception:
        script_dir = os.getcwd()
    # from cad/fusion_scripts/export_to_cad/ -> cad/
    return os.path.normpath(os.path.join(script_dir, '..', '..'))


def find_occurrence_by_name(root_comp, name):
    for occ in root_comp.allOccurrences:
        try:
            if occ.component.name == name or occ.name == name:
                return occ
        except Exception:
            pass
    return None


def find_body_by_name(root_comp, name):
    """For Part Design: bodies in root, no child components."""
    for body in root_comp.bRepBodies:
        try:
            if body.name == name:
                return body
        except Exception:
            pass
    return None


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active design. Open a design and run again.')
            return

        cad_root = get_cad_root()
        config_path = os.path.join(cad_root, 'export_config.json')
        if not os.path.isfile(config_path):
            ui.messageBox(f'Config not found: {config_path}\nCAD root: {cad_root}')
            return

        with open(config_path) as f:
            cfg = json.load(f)

        root_comp = design.rootComponent
        export_mgr = design.exportManager
        parts_cfg = cfg.get('parts') or {}
        asm_cfg = (cfg.get('assembly') or {}).get('step')

        exported = []
        for comp_name, rel_path in parts_cfg.items():
            target = find_occurrence_by_name(root_comp, comp_name)
            if target is None:
                target = find_body_by_name(root_comp, comp_name)
            if target is None:
                continue
            full = os.path.normpath(os.path.join(cad_root, rel_path))
            d = os.path.dirname(full)
            if d and not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
            try:
                stl_opts = export_mgr.createSTLExportOptions(target, full)
                export_mgr.execute(stl_opts)
                exported.append(rel_path)
            except Exception as e:
                ui.messageBox('Export failed {}: {}'.format(comp_name, e))

        if asm_cfg:
            full = os.path.normpath(os.path.join(cad_root, asm_cfg))
            d = os.path.dirname(full)
            if d and not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
            try:
                step_opts = export_mgr.createSTEPExportOptions(full)
                export_mgr.execute(step_opts)
                exported.append(asm_cfg)
            except Exception as e:
                ui.messageBox(f'STEP export failed: {e}')

        ui.messageBox(f'Exported {len(exported)} item(s):\n' + '\n'.join(exported[:20])
                     + ('\n...' if len(exported) > 20 else ''))

    except Exception as e:
        if ui:
            ui.messageBox(f'Error: {e}')
        raise
