# Fusion 360 Assembly Roadmap — Sensei Robot

High-level plan to assemble the Sensei robot in Fusion 360: from separate bodies/components to a fully constrained, exportable assembly. Uses `cad/sensei_params.yaml` (urdf_links, urdf_joints) and `docs/frame_cad_design.md` as references.

**Important:** **Part Design** allows only one component. For **BodiesToComponents** and **PlaceFromUrdf**, use an **Assembly** (or **Empty Assembly**) design: **File → New Design** and choose Assembly if offered. Run **GenerateFromParams**, **BodiesToComponents**, and **PlaceFromUrdf** in that design.

---

## 1. Get to an Assembly Structure

**Goal:** One **component** per link/part so Fusion can treat them as separate, movable pieces.

**Automated:** Run **BodiesToComponents** (Scripts and Add-Ins). It turns each body into a component and names it from `body.name` (e.g. `base_link`, `l_shoulder_link`, …).

**Manual alternative:**

| Step | Action |
|------|--------|
| 1.1 | In the design with GenerateFromParams bodies (and optional xl430_bracket): **Bodies** → right‑click each body → **Create Components from Bodies** (or **Bodies to Components** for all). |
| 1.2 | Rename each new component to match the link. |
| 1.3 | Ensure **`base_link`** exists; it will be the root of the kinematic tree. |
| 1.4 | *(Optional)* Replace simple link bodies with detailed parts (e.g. **xl430_bracket**, or future torso_main, foot) by inserting/deriving from other designs. |

**Output:** Root component with multiple **occurrences** (one per link), each still unconstrained.

---

## 2. Position and Define the Kinematic Tree

**Position (automated):** Run **PlaceFromUrdf**. It reads `cad/urdf_geometry.json` → **joints**, computes each link’s world transform from `base_link`, and sets `occurrence.transform2`. `base_link` stays at identity; others are placed from the URDF joint tree. Run after **BodiesToComponents**.

**Joints (manual):** The Fusion Joint API needs **JointGeometry** (faces, edges, or points) on each part. There is no “create joint at (x,y,z) with axis” from numbers alone, so joint creation is not fully automated. Add **Joints** by hand as below.

**Source:** `cad/sensei_params.yaml` → **urdf_joints** (or `cad/urdf_geometry.json` → **joints**): parent, child, type, origin_xyz, axis.

| Step | Action |
|------|--------|
| 2.1 | **ASSEMBLE** → **Joint**. For each `urdf_joint`: choose **Rigid** (fixed) or **Revolute** (revolute). |
| 2.2 | Set **Component 1** = child link, **Component 2** = parent link (fusion360-urdf-ros2 expects parent as Component 2). |
| 2.3 | Use **origin_xyz** (m → mm) and **axis** to place the joint: pick faces/edges or **Origin** so the joint sits at the right location and the revolute axis matches the URDF axis. |
| 2.4 | For **Revolute** joints: in the Joint dialog, set the **Axis** from `urdf_joints.*.axis` (e.g. `0,0,1` = Z; `0,1,0` = Y). |
| 2.5 | Name each joint to match URDF (e.g. `l_shoulder_pan`, `l_shoulder_lift`, `l_elbow_flex`, `l_hand_fixed`, `l_hip_pitch`, `l_knee_pitch`, `l_ankle_pitch`, `l_foot_fixed`, and their `r_*` and `neck_*` counterparts). |

**Order (follow parent → child):**  
`base_link` first, then its children (`l_shoulder_pan`, `r_shoulder_pan`, `l_hip_pitch`, `r_hip_pitch`, `neck_pan`), then *their* children, and so on. The exact list is in **urdf_joints**; sensei’s tree is base → arms (L/R), legs (L/R), head.

**Output:** Fully constrained assembly: fixed links rigid, revolute links rotating about the correct axes.

---

## 3. Pose and Validate

**Goal:** Check reach, clearances, and that the CAD matches the URDF.

| Step | Action |
|------|--------|
| 3.1 | **Move** `base_link` (or the root) so the robot sits on the ground (e.g. bottom of feet at Z=0) if desired. |
| 3.2 | Use **Joint** limits or the timeline to rotate revolute joints; run a quick motion study to see ranges and avoid self‑intersection. |
| 3.3 | **Inspect** → **Interference** to find clashes. Refine part geometry or joint positions if needed. |
| 3.4 | *(Optional)* Export via **fusion360-urdf-ros2**: run the add‑in, export to a ROS 2 package, then compare in RViz/Gazebo with `simulation/urdf` or your control stack. |

**Output:** A validated, poseable assembly ready for drawings, BOM, or URDF export.

---

## 4. Add Physical Assembly Detail (Optional)

**Goal:** Turn the kinematic placeholder into a buildable design.

| Step | Action |
|------|--------|
| 4.1 | **Replace link placeholders** with real parts: e.g. **torso_main** (from **TorsoMain**: open front, cavity, Pi/Teensy bosses, battery tray), arm/leg links with **LimbTubeWithChannel** (tube + open cable channel) or **Xl430Bracket**, **feet** with **FootWithFSR** (FSR pockets on sole). Use **Insert** → **Replace Component** or in‑place edits. See **`docs/fusion_image_to_hardware.md`**. |
| 4.2 | **Insert** or model **fasteners** (M2.5, M3) and **heat‑set inserts** per `frame_cad_design` (e.g. bracket rules, hardware list). |
| 4.3 | **Cable runs**: sketch or sweep paths; leave clearance (e.g. 8 mm) per bracket rules. |
| 4.4 | **BOM / Drawings**: use Fusion’s **Drawing** or **Design** BOM and 2D views for manufacture and assembly docs. |

**Output:** Assembly suitable for manufacturing, printing, and physical build per `docs/frame_cad_design.md` (Phase 1–4: Torso → Arms → Legs → Head).

---

## 5. Suggested Sequence (Summary)

1. **Bodies → components**: run **BodiesToComponents** (or do it manually).  
2. **Position**: run **PlaceFromUrdf** so occurrences follow the URDF joint tree.  
3. **Add Joints** manually from **urdf_joints** (Rigid / Revolute, parent/child, axes).  
4. **Pose and validate** (interference, motion, optional URDF export).  
5. **Detail** (real parts, fasteners, cables, BOM, drawings) as needed.

---

## 6. Data Sources

| Source | Use |
|--------|-----|
| `cad/sensei_params.yaml` → **urdf_links** | Link geometry and `origin_xyz`; already used by GenerateFromParams. |
| `cad/sensei_params.yaml` → **urdf_joints** | Parent, child, type, `origin_xyz`, `axis` for each Joint. |
| `docs/frame_cad_design.md` | Physical build order, bracket rules, hardware, print settings, assembly phases. |
| **fusion360-urdf-ros2** | Export Fusion assembly → URDF + STL + ROS 2 pkg; requires `base_link`, one component per link, and named Joints (Rigid / Revolute / Slider). |
| **docs/fusion_image_to_hardware.md** | Use the reference image in Fusion (Canvas); **TorsoMain**, **LimbTubeWithChannel**, **FootWithFSR**; `detail_params.yaml`; workflow to match exposed-electronics look. |

---

## 7. API Automation (Implemented)

| Script | API / behavior | Notes |
|--------|----------------|-------|
| **BodiesToComponents** | `BRepBody.createComponent()`; set `component.name = body.name`. | Run after GenerateFromParams. One component per body. |
| **PlaceFromUrdf** | Reads `urdf_geometry.json` → **joints**. Builds world transform per link (BFS from `base_link`, `origin_xyz`/`origin_rpy` → 4×4, m→mm). Sets `occurrence.transform2` via `Matrix3D.setWithArray()`. | Run after BodiesToComponents. Positions only; no constraints. |
| **Joints** | Fusion `joints.createInput(geo0, geo1)` needs **JointGeometry** (e.g. `createByPlanarFace`, `createByCurve`, `createByPoint`). `setAsRigidJointMotion()` / `setAsRevoluteJointMotion(axis)`. | Not automated: requires picking faces/edges/points. A future script could add construction points at joint origins and use `createByPoint`, or use heuristics to pick faces. |

**Data:** `scripts/urdf_to_cad_params.py` writes **joints** into `cad/urdf_geometry.json` (name, type, parent, child, origin_xyz, origin_rpy, axis). Resolves `${base_link}` and `${parent}`.

**Export:** **ExportToCad** for STL; **fusion360-urdf-ros2** for URDF + ROS 2.
