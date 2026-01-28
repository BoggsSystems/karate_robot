# Using Fusion 360 to Match the Reference Image

This document explains how to generate Sensei hardware in Fusion 360 so it matches the exposed-electronics reference image: open chest, visible cable runs on limbs, rectangular servos, Pi+Teensy, flat LiPo, and feet with FSR on the sole.

---

## 1. Use the Reference Image in Fusion

Use the image as a **visual reference** for proportions and layout; model from `frame_cad_design` dimensions for accuracy.

### 1.1 Insert and Calibrate the Image

1. **Insert → Canvas** (or **Attached Canvas**): attach the reference image to a plane (e.g. **XZ** for a front/side view, or **XY** for top).
2. **Calibrate scale:** In the Canvas dialog, set a **known distance**. Use **300 mm** = full robot height (top of head to bottom of feet). Pick two points on the image that correspond to that span and enter 300 mm.
3. **Position:** Place the canvas so the robot is centered and upright. You can use a second canvas on another plane (e.g. side view) if needed.

### 1.2 Use as a Sketch Guide

- Create a **New Sketch** on the same plane (or an offset) and use **Project / Include** to pull in the canvas, or **trace** the silhouette and key edges (torso opening, limb tubes, foot outline) as construction geometry.
- **Model from dimensions:** Use `docs/frame_cad_design.md` and `cad/detail_params.yaml` for all sizes. The sketch is only for proportions and the “look”; the parametric generators and specs are the source of truth.

---

## 2. Parametric Generators (Fusion Scripts)

Three scripts create **exposed-electronics** geometry that matches the image. They read from `cad/detail_params.yaml` (or in-script defaults) and produce **bodies** you can export or use in the assembly.

| Script | Output | Purpose |
|--------|--------|---------|
| **TorsoMain** | `torso_main` | Open-front torso, electronics cavity, Pi/Teensy bosses, battery tray, cable passthroughs |
| **LimbTubeWithChannel** | one body per link | Tube (from URDF radius/length) + **open cable channel** along the back |
| **FootWithFSR** | `foot_with_fsr` | Foot block with **two FSR pockets on the sole (underside)** |

### 2.1 TorsoMain

**Params** (in `cad/detail_params.yaml` or script defaults):

- **Outer:** 200×140×100 mm (W×D×H). **Cavity:** 100×80×40 mm internal.
- **Open front:** No front wall; front of the cavity is open. Optional skeletal struts.
- **Pi boss:** 4× M2.5 standoffs, 85×56 mm pattern (e.g. 58×49 hole spacing). Standoff 10 mm.
- **Teensy boss:** 4× M2 standoffs, 60×18 mm footprint. Standoff 6 mm.
- **Battery tray:** 105×35×20 mm well at the cavity floor.
- **Cable passthroughs:** 8 mm dia. at top (head), left/right (arms), bottom (pelvis/legs).

**Run:** In a **Part Design** or **Assembly**, run **TorsoMain**. It creates body `torso_main`. Export to `parts/torso/torso_main.stl`. In the main assembly, use **Insert → Insert into Current Design** (or **Replace Component**) to use it; you can **Replace** the `base_link` placeholder with `torso_main` if the assembly uses `base_link` for the central block.

### 2.2 LimbTubeWithChannel

**Params:**

- **Radius, length:** From `cad/urdf_geometry.json` per link (e.g. `l_upper_arm_link`: r=25 mm, L=120 mm).
- **Channel:** 8 mm wide, 4 mm deep, along the **posterior** side (“back” of the limb in a nominal pose).

**Links that get a channel:**  
`l_upper_arm_link`, `l_forearm_link`, `r_upper_arm_link`, `r_forearm_link`, `l_thigh_link`, `l_shin_link`, `r_thigh_link`, `r_shin_link`.

**Run:** **LimbTubeWithChannel** reads `urdf_geometry.json`, finds `type: cylinder` parts in the limb list above, and creates one **tube-with-channel** body per link, **name = link name** (e.g. `l_upper_arm_link`). You can then **Replace** the cylinder placeholders from GenerateFromParams with these bodies (or run **LimbTubeWithChannel** *instead* of generating those cylinders in GenerateFromParams, and feed the rest of the pipeline as-is).

### 2.3 FootWithFSR

**Params:**

- **Size:** 100×60×30 mm (L×W×H). Matches `frame_cad_design`.
- **FSR:** 10 mm dia. circular pockets on the **underside (sole)**, 2 mm deep. **Ball** 25 mm from toe; **heel** 25 mm from heel (toe = +X).

**Run:** **FootWithFSR** creates body `foot_with_fsr`. Duplicate for left/right and name `l_foot_link`, `r_foot_link`, or run with `left_right: both` to create both. Export to `parts/leg_left/foot_L.stl`, `parts/leg_right/foot_R.stl`. **Replace** the `l_foot_link` / `r_foot_link` placeholders in the assembly.

---

## 3. Detail Params File

**Path:** `cad/detail_params.yaml`

Defines dimensions for TorsoMain, LimbTubeWithChannel, and FootWithFSR so you can tweak without editing the scripts. The scripts load this file when present; otherwise they use built-in defaults.

```yaml
# cad/detail_params.yaml (example)
torso_main:
  outer_mm: [200, 140, 100]
  cavity_mm: [100, 80, 40]
  open_front: true
  wall_mm: 3
  pi_boss: { size_mm: [85, 56], hole_spacing_mm: [58, 49], standoff_mm: 10, hole_d_mm: 2.7 }
  teensy_boss: { size_mm: [60, 18], standoff_mm: 6, hole_d_mm: 2 }
  battery_tray_mm: [105, 35, 20]
  cable_passthrough_d_mm: 8

limb_channel:
  width_mm: 8
  depth_mm: 4
  # Links: from urdf_geometry type=cylinder; list in script or here
  links: [l_upper_arm_link, l_forearm_link, r_upper_arm_link, r_forearm_link, l_thigh_link, l_shin_link, r_thigh_link, r_shin_link]

foot_fsr:
  size_mm: [100, 60, 30]
  fsr_diameter_mm: 10
  fsr_pocket_depth_mm: 2
  ball_offset_from_toe_mm: 25
  heel_offset_from_heel_mm: 25
```

---

## 4. Workflow: From Image to Hardware

### Option A — Replace placeholders (recommended)

1. **URDF → geometry**
   - `python3 scripts/urdf_to_cad_params.py`

2. **Fusion Assembly:** Create an **Assembly** design.
   - **(Optional)** **Insert → Canvas**: attach the reference image, calibrate 300 mm height.
   - Run **GenerateFromParams** → **BodiesToComponents** → **PlaceFromUrdf** to get positioned placeholders.

3. **Detailed parts**
   - **TorsoMain:** In the same design (or a Part design), run **TorsoMain**. Insert `torso_main` if created elsewhere. **Replace** the `base_link` occurrence with `torso_main` (or hide `base_link` and position `torso_main` at the base_link transform).
   - **LimbTubeWithChannel:** Run **LimbTubeWithChannel** (in the assembly or in a Part with the bodies then Insert). It creates bodies named `l_upper_arm_link`, etc. **Replace** the corresponding placeholder occurrences.
   - **FootWithFSR:** Run **FootWithFSR**. Create `l_foot_link` and `r_foot_link` (duplicate or `left_right: both`). **Replace** `l_foot_link` and `r_foot_link`.

4. **Joints and export**
   - Add **Joints** (Rigid / Revolute) by hand from `urdf_joints` (see `fusion_assembly_roadmap`).
   - Run **ExportToCad** for STLs and `sensei_full.step`. Ensure `export_config.json` includes `torso_main`, `foot_with_fsr` or `l_foot_link`/`r_foot_link`, and the limb link names.

### Option B — Build from detailed parts first

1. Run **TorsoMain**, **LimbTubeWithChannel**, **FootWithFSR** in one or more Part designs.
2. **Export** each to STL (or keep as bodies).
3. **New Assembly:** **Insert → Insert into Current Design** (or Insert from file) for `torso_main`, each limb link, and both feet. Name components to match `base_link`, `l_upper_arm_link`, etc.
4. Run **PlaceFromUrdf** (it positions by **component name**; ensure names match `urdf_geometry` / `joints`).
5. Add **Joints** and run **ExportToCad**.

---

## 5. Export Config

Add to `cad/export_config.json` (or `export_config.yaml` before converting):

```json
"torso_main": "parts/torso/torso_main.stl",
"l_upper_arm_link": "parts/arm_left/l_upper_arm_link.stl",
"l_forearm_link": "parts/arm_left/l_forearm_link.stl",
"l_foot_link": "parts/leg_left/foot_L.stl",
...
```

`urdf_to_cad_params.py` already adds link-name → path for URDF links; add `torso_main` and adjust `l_foot_link`/`r_foot_link` to `foot_L`/`foot_R` if you use that naming.

---

## 6. Image ↔ CAD Checklist

| Image element | Fusion approach |
|---------------|-----------------|
| **Open chest, visible Pi/Teensy** | **TorsoMain**: `open_front: true`, Pi and Teensy bosses (standoffs) in cavity. |
| **Flat LiPo in tray** | **TorsoMain**: `battery_tray_mm: [105,35,20]`. |
| **Red/black cables along limbs** | **LimbTubeWithChannel**: open groove 8×4 mm; route wires in channel when assembling. |
| **Rectangular servos at joints** | **Xl430Bracket** for brackets; servos are purchased parts. Place them at joint positions; no need to model the servo body in CAD beyond the bracket. |
| **Feet with FSR on sole** | **FootWithFSR**: two pockets on the **underside**; `ball_offset_from_toe_mm`, `heel_offset_from_heel_mm`. |
| **Left fist blue, right fist red** | Hand/fist body from GenerateFromParams or a simple box; **color is finish** (paint or filament). Not geometry. |
| **Head: screen + camera** | Head from URDF (e.g. `head_link`); add a **camera_mount** and an LCD cutout in the head part if you model it in detail. |

---

## 7. References

- `docs/frame_cad_design.md` — torso 200×140×100, cavity 100×80×40, Pi 85×56, Teensy 60×18, LiPo 105×35, limb sizes, foot 100×60×30, FSR, cable 8 mm.
- `docs/hardware_architecture.md` — Pi5, Teensy, XL430, battery.
- `docs/sensei_image_prompt.md` — reference image prompt and refinements.
- `cad/README.md` — CAD layout, scripts, export.
- `docs/fusion_assembly_roadmap.md` — BodiesToComponents, PlaceFromUrdf, Joints, detailing.

---

*Last updated: January 2026*
