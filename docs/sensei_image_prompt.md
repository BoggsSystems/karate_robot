# Sensei Robot — Image Generation Prompt for Gemini

Use the following prompt with Gemini (Imagen / image generation) to generate a reference image of what the Sensei karate robot should look like.

---

## Copy-paste block (single paragraph, for quick use)

```
Generate a photorealistic image of a small humanoid karate robot named Sensei, ~30 cm tall, with an exposed-electronics maker aesthetic. Open chest showing Raspberry Pi (green PCB), Teensy, LiPo battery, and red/black power wires; 3D-printed black PETG torso and pelvis with skeletal front frame. Shoulder, elbow, hip, knee, ankle servos (Dynamixel XL430, compact metal rectangles) visible at each joint. Arms: ~25 mm and ~22 mm tubes with open cable channels; left fist blue, right fist red (60×40×30 mm blocks). Legs: thigh and shin tubes with visible cables; black TPU feet ~100×60 mm with optional FSR sensors. Compact head ~60×60×50 mm with camera and two-DOF neck. Karate ready stance: feet apart, knees bent, fists in guard. Three-quarter view, studio lighting, neutral or dojo background. Technical, buildable, purposeful — not toy or glossy. No enclosed torso, no human face.
```

---

## Full prompt (detailed, for best control)

Generate a photorealistic or highly detailed 3D-render-style image of a small humanoid robot named Sensei, designed for karate and robotics research. The robot has an **exposed-electronics, maker-style aesthetic**: visible internals, 3D-printed frame, and visible wiring. It should look like a real prototype that could be built in a workshop, not a glossy consumer product.

### Scale and proportions
- **Height:** about 30 cm (12 inches) standing — small enough to fit on a desk or table.
- **Build:** Compact and dense. Torso is the largest mass; limbs are slender and mechanical. Shoulder span is roughly 280 mm; hip width about 120 mm. Overall proportions are humanoid but slightly top-heavy, with a wide torso relative to limb thickness.

### Torso (chest and belly)
- **Open-front design.** The chest has **no full front panel** — the electronics bay is visible from the front.
- **Visible inside the chest:**
  - A **Raspberry Pi** (green PCB, ~85×56 mm) mounted on standoffs, with USB ports and GPIO visible.
  - A **Teensy** or similar small MCU board (compact, with a double-row pin header).
  - A **LiPo battery** (rectangular, ~105×35 mm) in a simple tray or holder; red and black power wires clearly visible.
  - Optionally a **small LCD display** (e.g. 2–3 inches) in the upper chest as a “face” or status display.
- **Wiring:** Red and black power cables, and thinner signal wires (various colors), routed from the Pi and Teensy toward the shoulders and down toward the pelvis. Wires are neatly bundled where possible but visibly run along the frame — not hidden inside solid plastic.
- **Torso frame:** 3D-printed PETG shell forming the back, sides, and a **skeletal or partial front frame** (e.g. vertical struts or an open frame) that holds the electronics. Color: matte **black or dark gray**. The shell has a technical, FDM-printed look: slight layer lines OK, no glossy coating. **Shoulder servo mounts** on the left and right: each shoulder pan servo (Dynamixel XL430) is mounted at the top-outside of the torso; the servos are compact **rectangular metal blocks** (~28×46×34 mm) with cables coming out one end.
- **Pelvis:** A rigid **hip/pelvis block** below the torso, also 3D-printed black or dark gray, with left and right hip servos (XL430) mounted. Cables from the legs and battery/electronics run through or along this block.

### Arms (left and right)
- **Structure:** Each arm has three segments with **visible Dynamixel XL430 servos** at shoulder (lift), elbow, and a fist at the end. The shoulder pan is in the torso; the next servo is at the top of the upper arm.
- **Upper arm:** ~120 mm long; **25 mm diameter** tube or faceted 3D-printed link in black or dark gray. A **cable channel or open groove** runs along the back so the Dynamixel daisy-chain cable is visible (red/black and thin signal wires). The **shoulder-lift servo** (XL430) is at the shoulder end; the **elbow servo** at the elbow.
- **Forearm:** ~110 mm; **22 mm diameter** tube or link, same style. Cable continues in an open channel to the hand.
- **Hand:** A **fist** — a single 3D-printed block ~60×40×30 mm, no articulated fingers. **Left hand: blue** (or blue-gray). **Right hand: red** (or red-orange). This helps distinguish sides during karate forms.
- **Servos:** Dynamixel XL430 appear as **compact metallic rectangles** with a cable pigtail; they are not covered by cosmetic shells.

### Legs (left and right)
- **Structure:** Each leg has hip, thigh, knee, shin, ankle, and foot. **Hip, knee, and ankle servos** (XL430) are clearly visible at each joint.
- **Thigh:** ~140 mm; **30 mm diameter** tube or printed link, black/dark gray. **Cable channel** along the back or side so the servo cable is visible.
- **Shin:** ~120 mm; **28 mm diameter** tube, same treatment. Cable runs to the ankle and foot.
- **Foot:** **~100×60×30 mm** overall. **TPU (flexible) sole** in **black or dark gray** — slightly different sheen or texture than the rigid PETG. The sole can have a simple tread or flat contact surface. Optionally suggest **two small circular FSR (force) sensors** on the sole — small discs or pads under the ball and heel.
- **Ankle:** The ankle servo (XL430) is just above the foot; the foot is fixed to it (no toe joint).

### Head
- **Shell:** A **compact head** ~60×60×50 mm — squarish or slightly rounded, 3D-printed black or dark gray. It sits on a **two-DOF neck** (pan and tilt) so the head can look left/right and up/down.
- **Face / front:** Either (a) **a single small camera lens** (e.g. 5–8 mm) centered, suggesting an OAK-D or Pi Camera, or (b) a **small LCD** (like in the torso) used as a simple face, or (c) both: camera above, small status/face display below.
- **Neck:** Two small **neck servos** (e.g. XL330 — slightly smaller than XL430) in a neck bracket; the head is clearly mounted on top. Cables from the camera/display run down into the torso.

### Materials and colors (summary)
- **Frame (PETG):** Matte **black** or **dark gray**. Slight 3D-print texture; no gloss.
- **Feet (TPU):** **Black** or very dark gray; slightly different, rubber-like look.
- **Hands:** **Left = blue** (or blue-gray); **Right = red** (or red-orange).
- **Electronics:** **Green PCB** (Pi), **red and black** power wires, **multicolor** signal wires. **Metallic** servo housings (silver, dark gray, or black).
- **Battery:** Black or dark wrap; **red (+)** and **black (-)** leads prominent.

### Pose and attitude
- **Stance:** A **karate ready stance**: feet shoulder-width or slightly wider, knees slightly bent, weight centered. One hand (e.g. right) in a **fist** held near the hip or chest in a guard position; the other slightly forward or in a complementary guard. The pose should read as **disciplined, alert, and martial** — a robot prepared to perform kata or spar, not casual or toy-like.
- **Head:** Facing forward or slightly toward the “lead” hand; neck not drooping.

### Camera angle and lighting
- **Angle:** **Three-quarter view** (e.g. from front-left or front-right) so both the open chest and one side of the body are visible. Slightly from below or eye-level to emphasize the robot’s presence. Full body in frame.
- **Lighting:** **Studio or workshop lighting** — soft key light from the front-left or front-right, with a gentler fill so the chest cavity, cables, and servos are readable. Slight rim or edge light on the limbs to separate them from the background. No harsh shadows that hide the electronics.
- **Background:** **Neutral and uncluttered** — e.g. light gray or white cyclorama, or a simple workshop table/shelf with a matte surface. No busy backgrounds; the robot is the only focus. Optionally a **dojo** or **training** hint (e.g. wooden floor, plain wall, or soft mat) to support the karate context.

### Style and mood
- **Photorealistic or high-quality 3D render:** Sharp focus on the robot; materials and proportions should feel buildable and real.
- **Mood:** **Technical, purposeful, and maker-built** — like a serious hobby or research project. The exposed electronics and visible wiring should read as **intentional and functional**, not broken or messy. Sensei should look like it could walk, punch, and kick in a dojo.

### What to avoid
- Do not make it look like a toy, a cartoon, or a sleek commercial humanoid with smooth, featureless surfaces.
- Do not fully enclose the torso — the chest must show boards and wiring.
- Do not use human-like skin, hair, or facial features; the head is a mechanical shell with a camera and/or small display.
- Do not add extra limbs, wings, or non-humanoid features.
- Do not make the robot look damaged, rusty, or abandoned; it should look operational and well-maintained.
- **Do not use circular discs or round plates for joints** — servos are compact **rectangular** blocks (Dynamixel-style), not circles.
- **Do not use cylindrical 18650-style batteries** — use a **flat rectangular LiPo pack** in a tray.

---

## Refinement add-on (append after first generation)

If the first image had: **circular joint plates**, **cylindrical battery**, **enclosed limbs** (no visible cables), **one PCB instead of Pi + Teensy**, or **FSRs on top of feet** — append this block to your prompt and regenerate:

```
IMPORTANT: (1) In the chest show TWO separate boards: a larger green Raspberry Pi and a smaller Teensy/MCU board with pin headers — not a single combined PCB. (2) Battery is a FLAT RECTANGULAR LiPo pack in a tray, NOT cylindrical 18650 cells. (3) Every limb has an OPEN CABLE CHANNEL or half-tube along the back: red/black servo cables must be VISIBLE running along the outside of each upper arm, forearm, thigh, and shin from joint to joint. (4) At each joint the servo is a small RECTANGULAR metal block (Dynamixel-style ~28×46 mm), NOT a circular disc or round plate. (5) Feet: black TPU sole with a slightly rubber-like look; two small circular FSR pads on the UNDERSIDE (ball and heel), not on top.
```

---

## Revised copy-paste block (v2)

Use this instead of the first block if you want the refinements baked in (two boards, flat LiPo, visible limb cables, rectangular servos, FSR on sole):

```
Generate a photorealistic image of a small humanoid karate robot named Sensei, ~30 cm tall, exposed-electronics maker aesthetic. Open chest with TWO boards: larger green Raspberry Pi and smaller Teensy with pin headers; FLAT rectangular LiPo in a tray (not cylindrical cells); red/black wires. 3D-printed black PETG torso and pelvis, skeletal front frame. At every joint: small RECTANGULAR Dynamixel-style servo (~28×46 mm), NOT circular discs. Arms and legs: tubes with OPEN CABLE CHANNELS along the back — red/black servo cables VISIBLE on the outside of each upper arm, forearm, thigh, shin. Left fist blue, right fist red. Feet: black TPU sole, rubber-like; two small circular FSR pads on the UNDERSIDE (ball and heel). Compact head with camera, two-DOF neck. Karate ready stance, three-quarter view, studio lighting, neutral or dojo background. Technical, buildable — not toy. No enclosed torso, no human face.
```

---

## Notes for the prompt author

### Design sources
- **`docs/frame_cad_design.md`** — dimensions, part list, arm/leg/head/torso specs, PETG/TPU, hand (fist) and foot (FSR) details.
- **`docs/hardware_architecture.md`** — Pi5, Teensy, XL430, battery, IMU, foot FSR, optional OAK-D.
- **URDF / `sensei_*`** — left hand blue, right hand red (`hand_blue`, `hand_red`); 14 DOF layout.
- **“Exposed electronics” style** — open torso, visible Pi/Teensy/LiPo, red/black wiring, cable channels in limbs; see conversation on hybrid exposed design.

### Lessons from first generation (Gemini)
First runs often produce: circular disc joints (use “rectangular servo” explicitly), cylindrical 18650 battery (say “flat LiPo pack”), enclosed limbs (say “open cable channel” and “visible on the outside”), single PCB (say “two boards: Pi and Teensy”), FSR on top of feet (say “on the underside”). Use **Refinement add-on** or **Revised copy-paste block (v2)** to correct.

### Shorter variants (if the full prompt is too long)
- **Minimal:** “Small humanoid robot ~30 cm tall, 3D-printed black frame, open chest with visible Raspberry Pi, Teensy, and red/black wiring, Dynamixel servos at each joint, blue left fist and red right fist, black TPU feet, compact head with camera, karate ready stance, studio lighting, photorealistic.”
- **Style-focused:** “Maker-style humanoid robot with exposed electronics: open torso showing green Pi and red/black wires, visible servo cables in arms and legs, 3D-printed black PETG and black TPU feet, blue left fist and red right fist, karate guard stance, photorealistic 3D render, neutral background.”

### Optional additions (append to the main prompt if desired)
- **“Include a small IMU board (e.g. BNO085) visible in the torso or on the spine.”**
- **“Add a U2D2 or USB–Dynamixel dongle with a short cable near the Pi.”**
- **“The foot sole has two small circular force sensors (FSR) under the ball and heel.”**
- **“A small LCD in the upper chest shows a simple abstract face or status icon.”**
- **“Dojo setting: tatami or wooden floor, plain wall, soft natural light.”**

---

*Generated for the Sensei karate robot project. Last updated: January 2026.*
