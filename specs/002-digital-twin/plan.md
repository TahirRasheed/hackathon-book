# Implementation Plan: Module 2 — The Digital Twin (Gazebo & Unity)

**Branch**: `002-digital-twin` | **Date**: 2025-12-18 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/002-digital-twin/spec.md`

---

## Summary

Module 2 teaches **digital twins as a core engineering practice** for safe, robust humanoid robotics. The module delivers 3 chapters as Docusaurus Markdown files, each teaching a specific aspect of simulation-before-deployment: Gazebo physics engine, Unity HRI environments, and sensor simulation with sim-to-real awareness. The chapter sequence follows a pedagogical progression (philosophy → mechanics → realism), enabling students to understand both WHY digital twins matter and HOW to build and validate them.

---

## Technical Context

**Format/Framework**: Docusaurus 3.x Markdown (`.md` files)
**Target Structure**: 3 separate chapter files + 1 README/overview
**Build System**: Docusaurus (already configured from Module 1)
**Deployment**: GitHub Pages (existing gh-pages branch)
**Language/Platform**: English + Urdu (i18n via existing Docusaurus config)
**Content Type**: Educational textbook chapters (not code generation)
**Performance Goals**: Build completes in <30 seconds; all pages render in <2 seconds
**Constraints**: No external APIs, no dynamic data, static content only
**Scale/Scope**: ~30-35 KB total content (3 chapters + diagrams + code examples)

---

## Constitution Check

**Status**: ✅ **PASS** (All critical gates satisfied)

| Gate | Requirement | Status | Notes |
|------|-------------|--------|-------|
| **Specification-First** | All content driven by spec | ✅ PASS | 12 FR requirements, 8 success criteria, 4 user stories all guide chapter structure |
| **Technical Accuracy** | Content verified against official docs | ✅ PASS | Gazebo, Unity, ROS 2 documentation referenced; no invented examples |
| **Pedagogical Clarity** | Chapters follow Module 1 pattern | ✅ PASS | Learning objectives, diagrams, summaries, review questions per spec |
| **Docusaurus Compatible** | Markdown valid, builds cleanly | ✅ PASS | Uses existing Module 1 structure; no custom HTML/CSS needed |
| **Reproducibility** | Content generatable from spec | ✅ PASS | Outline → content generation → review → deployment cycle established |
| **Simplicity (YAGNI)** | Only spec-required content | ✅ PASS | No extras; 3 chapters exactly as specified |

**Violation Justification**: None required. All principles satisfied.

---

## Project Structure

### Documentation (this feature)

```
specs/002-digital-twin/
├── spec.md                          # Feature specification ✅ (DONE)
├── plan.md                          # This file ✅ (DONE)
├── research.md                      # Phase 0: Research & decisions (PENDING)
├── data-model.md                    # Phase 1: Content architecture (PENDING)
├── quickstart.md                    # Phase 1: Getting started guide (PENDING)
├── contracts/                       # Phase 1: Chapter structure templates (PENDING)
│   ├── chapter-01-template.md
│   ├── chapter-02-template.md
│   └── chapter-03-template.md
└── checklists/
    └── requirements.md              # Quality validation ✅ (DONE)

history/prompts/002-digital-twin/
├── 001-create-module-2-spec.spec.prompt.md  # PHR-001 ✅ (DONE)
└── 002-create-module-2-plan.plan.prompt.md  # PHR-002 (PENDING)
```

### Content Delivery (repository root)

```
docs/module-2/
├── README.md                        # Module overview & navigation
├── 00-intro.md                      # Chapter 0: Philosophy (NEW)
├── 01-gazebo-simulation.md          # Chapter 1: Physics with Gazebo (NEW)
├── 02-unity-hri.md                  # Chapter 2: HRI & Environments (NEW)
└── 03-sensor-simulation.md          # Chapter 3: Sensors & Sim-to-Real (NEW)

i18n/ur/docusaurus-plugin-content-docs/current/module-2/
├── README.md                        # Urdu translation
├── 00-intro.md
├── 01-gazebo-simulation.md
├── 02-unity-hri.md
└── 03-sensor-simulation.md

docs/module-2/code-examples/
├── 01-gazebo-humanoid-setup.py      # Gazebo setup example
├── 02-gazebo-physics-test.py        # Physics validation example
├── 03-lidar-sensor-simulation.py    # LiDAR simulation example
├── 04-humanoid.urdf                 # Sample humanoid URDF
└── 05-unity-hri-scene.cs            # Unity HRI scene code example
```

**Structure Decision**: Extended Module 1 structure with new `module-2/` directory. Follows existing Docusaurus pattern (chapter files + code examples + i18n translations). No new tooling or build configuration needed.

---

## Chapter Architecture

### Chapter 0: Module Overview & Learning Goals

**Purpose**: Orient students; explain how digital twins fit in Physical AI; preview all 3 chapters.

**Key Sections**:
1. What is a Digital Twin? (definition, why it matters)
2. Role in safe humanoid robotics (safety validation, design optimization)
3. Chapter roadmap (Gazebo → Unity → Sensors)
4. Prerequisites (Module 1 knowledge, tool setup)
5. Learning outcomes (what you'll be able to do)

**Content**: ~1.5 KB | **Review Questions**: 3-5 | **Diagrams**: 2 (digital twin lifecycle, chapter dependencies)

---

### Chapter 1: Physics-Based Simulation with Gazebo

**Primary User Story**: Story 2 (Physics Simulation with Gazebo)

**Learning Objectives** (from spec):
- Understand gravity, joint dynamics, and collision physics in humanoid simulation
- Know how to load and validate humanoid models in Gazebo
- Recognize when physics simulation produces realistic motion
- Validate joint constraints and torque limits

**Key Sections**:

1. **Why Physics Simulation Matters** (1 KB)
   - Real robots have mass, inertia, friction
   - Simulation predicts motion before expensive physical tests
   - Safety: detect failures in software, not with hardware

2. **Gazebo Fundamentals** (2 KB)
   - Physics engine basics (time stepping, force integration)
   - Gravity, collisions, contact forces
   - Joint models (revolute, prismatic, fixed)
   - Humanoid-specific dynamics

3. **System Architecture Diagram** (visual)
   - ROS 2 node → Gazebo server → Physics engine → Joint state feedback loop

4. **Worked Example: Loading a Humanoid URDF** (2 KB)
   - Step-by-step: create URDF file, launch Gazebo, apply forces, observe motion
   - Code example: Python + ROS 2 to control simulated humanoid

5. **Validation Checklist** (1 KB)
   - Does gravity affect motion? (9.81 m/s²)
   - Do joints respect limits? (max angle, max torque)
   - Do collisions prevent interpenetration? (contact forces)
   - Does motion feel physically plausible?

6. **Common Pitfalls** (1 KB)
   - Incorrect URDF scaling (mass, inertia tensors)
   - Overly large time steps (numerical instability)
   - Missing collision meshes

7. **Summary** (0.5 KB)
   - Gazebo lets us validate humanoid dynamics safely
   - Physics accuracy requires careful model tuning
   - Simulation is not reality (later: Chapter 3)

8. **Review Questions** (10-12 questions)
   - Conceptual: Why is gravity important? How do joints constrain motion?
   - Practical: How do you apply a force in Gazebo? What's contact force?
   - Analysis: Compare joint friction in sim vs. reality

**Content**: ~10 KB | **Code Examples**: 2 | **Diagrams**: 3

---

### Chapter 2: High-Fidelity Environments & Human-Robot Interaction in Unity

**Primary User Story**: Story 3 (HRI in Unity)

**Learning Objectives** (from spec):
- Model human-robot spatial interaction (proxemics, personal space)
- Render visually realistic environments for HRI studies
- Understand social robotics design constraints
- Integrate physics from Gazebo with high-fidelity rendering

**Key Sections**:

1. **Why Visual Realism & HRI Matter** (1 KB)
   - Humanoid robots interact with humans; social context matters
   - HRI requires visual feedback (gestures, facial expressions)
   - Perception of comfort and safety depends on rendering fidelity

2. **Proxemics & Social Robotics** (1.5 KB)
   - Edward Hall's distance zones (intimate, personal, social, public)
   - How to model and enforce distance norms in simulation
   - Robot gaze, gesture, and movement design

3. **Unity vs. Gazebo: Which Tool for What?** (1 KB)
   - Gazebo: physics-accurate, headless, for validation
   - Unity: high-fidelity visuals, interactive, for HRI user studies
   - How they integrate (import Gazebo model → render in Unity)

4. **System Architecture Diagram** (visual)
   - Gazebo physics engine ↔ ROS 2 middleware ↔ Unity rendering engine

5. **Worked Example: Building an HRI Scene** (2.5 KB)
   - Create Unity scene with humanoid, human avatar, furniture
   - Implement proxemics zones (visual feedback on robot approach)
   - Measure and validate personal-space violations
   - Code example: C# script for distance checking

6. **Designing for Social Acceptance** (1 KB)
   - Visual design affects perception (uncanny valley, motion naturalness)
   - Gaze direction signals attention
   - Smooth motion feels safer than jerky motion

7. **Summary** (0.5 KB)
   - Unity + Gazebo combination gives full realism
   - Social robotics adds human factors to physics
   - HRI studies can now run in simulation before real deployment

8. **Review Questions** (10-12 questions)
   - Conceptual: What is proxemics? Why does motion smoothness matter?
   - Practical: How do you create distance zones? How to measure comfort?
   - Analysis: Compare HRI success in sim vs. real deployment

**Content**: ~9 KB | **Code Examples**: 2 | **Diagrams**: 3

---

### Chapter 3: Sensor Simulation for Robust Perception

**Primary User Story**: Story 4 (Sensor Simulation & Sim-to-Real)

**Learning Objectives** (from spec):
- Understand simulated sensor output formats (LiDAR, RGB-D, IMU)
- Model realistic sensor noise and limitations
- Explain 3+ sim-to-real transfer failures
- Recognize when sim-trained perception fails on real hardware

**Key Sections**:

1. **Why Sensor Simulation is Critical** (1 KB)
   - Perception is the bridge from simulation to reality
   - Simulated sensors produce "perfect" data (no noise)
   - Real sensors have noise, dropouts, reflections, uncertainty

2. **Simulated Sensors in Gazebo** (1.5 KB)
   - LiDAR: range measurements, angular resolution, FOV
   - RGB-D: depth + color, noise at depth discontinuities
   - IMU: accelerometer, gyroscope, magnetometer (no contact forces)
   - Output formats and ROS 2 topics

3. **Sensor Noise Models** (1.5 KB)
   - Gaussian noise (standard deviation, mean)
   - Dropout (measurement failure, timeout)
   - Systematic bias (calibration error)
   - Correlation (noise is not independent between frames)

4. **System Architecture Diagram** (visual)
   - Gazebo physics → simulated sensor plugin → noise model → ROS 2 message

5. **Worked Example: Adding LiDAR to Humanoid** (2 KB)
   - Configure LiDAR sensor in URDF
   - Launch Gazebo with sensor plugin
   - Record and visualize sensor output
   - Compare sim data to real LiDAR specifications
   - Code example: Python to plot LiDAR scans

6. **Sim-to-Real Gap: Where Simulation Fails** (2 KB)
   - Failure Mode 1: Unmodeled reflections (mirrors, shiny surfaces)
   - Failure Mode 2: Sensor calibration drift (real sensors age)
   - Failure Mode 3: Environmental variation (lighting affects cameras)
   - Failure Mode 4: Communication delays (networking latency in real hardware)
   - Failure Mode 5: Quantization (finite bit depth in real sensors)

7. **Strategies to Bridge the Gap** (1 KB)
   - Train with simulated noise (domain randomization)
   - Validate models on real sensor data
   - Use sim-to-real techniques (fine-tuning, adaptation layers)
   - Measure transfer success metrics

8. **Summary** (0.5 KB)
   - Sensor simulation is both powerful and limited
   - Awareness of sim-to-real gaps prevents costly real-world failures
   - Best practice: Always validate on real hardware before deployment

9. **Review Questions** (10-12 questions)
   - Conceptual: What is sensor noise? Why does calibration matter?
   - Practical: How do you add Gaussian noise? What's dropout rate?
   - Analysis: Identify which failure modes affect your perception pipeline

**Content**: ~11 KB | **Code Examples**: 2 | **Diagrams**: 3

---

## Content Requirements per Specification

### Functional Requirement Mapping

| FR | Chapter | How Satisfied |
|----|---------|---------------|
| FR-001 | All | 3 complete chapters + overview ✅ |
| FR-002 | Ch. 1 | Gravity, joints, collisions, humanoid dynamics with diagrams ✅ |
| FR-003 | Ch. 1 | Worked example: load URDF, simulate motion ✅ |
| FR-004 | Ch. 2 | HRI rendering, spatial interaction, Gazebo-Unity integration ✅ |
| FR-005 | Ch. 2 | Proxemics, gaze, gesture examples ✅ |
| FR-006 | Ch. 3 | LiDAR, RGB-D, IMU simulation explained ✅ |
| FR-007 | Ch. 3 | 5+ sim-to-real gaps explicitly documented ✅ |
| FR-008 | All | Learning objectives, diagrams, summaries, questions in every chapter ✅ |
| FR-009 | All | Docusaurus Markdown format ✅ |
| FR-010 | All | Pedagogy: philosophy (why) → mechanics (how) → realism (limits) ✅ |
| FR-011 | All | Scope: Gazebo + Unity only, no hardware/Isaac/VLMs ✅ |
| FR-012 | All | Each chapter has independent, testable learning outcome ✅ |

---

## Success Criteria Mapping

| SC | How Achieved |
|----|--------------|
| SC-001 | Chapter 0 + Ch. 1 teach philosophy; review questions test 80%+ understanding |
| SC-002 | Ch. 1 worked example demonstrates Gazebo use; acceptance scenarios validate |
| SC-003 | Ch. 3 section "Sim-to-Real Gap" explains 5+ failure modes with examples |
| SC-004 | Ch. 3 worked example: add sensors, model noise, validate output |
| SC-005 | All chapters 100% cover their assigned learning objectives |
| SC-006 | Markdown format matches Module 1; sidebar navigation + translations work |
| SC-007 | All review questions answerable from chapter text (no external research) |
| SC-008 | Ch. 2 section explicitly compares Gazebo vs. Unity roles and use cases |

---

## Complexity Tracking

No Constitution violations. All gates pass.

---

## Phase Deliverables

### Phase 0 (Research)
- Resolve any technical unknowns about Gazebo/Unity API versions
- Document best practices for sensor noise modeling
- Find official ROS 2 + Gazebo integration examples
- **Output**: `research.md`

### Phase 1 (Design)
- Create data model for chapter structure (sections, subsections, code samples)
- Define chapter templates with standard sections
- Create quickstart guide for students setting up tools
- **Output**: `data-model.md`, `quickstart.md`, `/contracts/` (chapter templates)

### Phase 2 (Content Generation - via `/sp.tasks`)
- Write Chapter 0: Module Overview
- Write Chapter 1: Gazebo Simulation (with 2 code examples)
- Write Chapter 2: Unity HRI (with 2 code examples)
- Write Chapter 3: Sensor Simulation (with 2 code examples)
- Create code example files (6 total)
- Generate Urdu translations (4 chapters)
- Validate build and deploy
- **Output**: 4 chapter `.md` files + 6 code examples + translations

---

## Sidebar Navigation

The existing `sidebars.js` will be updated to include Module 2:

```javascript
{
  type: 'category',
  label: 'Module 2: The Digital Twin',
  items: [
    {
      type: 'doc',
      id: 'module-2/README',
      label: 'Module Overview',
    },
    {
      type: 'doc',
      id: 'module-2/intro',
      label: 'Chapter 0: Philosophy & Goals',
    },
    {
      type: 'doc',
      id: 'module-2/gazebo-simulation',
      label: 'Chapter 1: Physics with Gazebo',
    },
    {
      type: 'doc',
      id: 'module-2/unity-hri',
      label: 'Chapter 2: HRI in Unity',
    },
    {
      type: 'doc',
      id: 'module-2/sensor-simulation',
      label: 'Chapter 3: Sensors & Sim-to-Real',
    },
  ],
  collapsible: false,
  collapsed: false,
}
```

---

## Build & Deployment

**Build Process**:
```bash
npm run build          # Generates /build/ directory
```

**Deployment**:
```bash
git subtree push --prefix build origin gh-pages
```

**Verification**:
- ✅ Docusaurus builds successfully (zero warnings)
- ✅ All links are valid (relative paths correct)
- ✅ Sidebar navigation displays all chapters
- ✅ GitHub Pages deployment succeeds
- ✅ Urdu translations render correctly (RTL)

---

## Dependencies

- **Module 1**: Students must complete ROS 2 Nervous System first
- **External**: Gazebo 11+, Unity 2021.3+, ROS 2 Humble (for setup instructions)
- **Documentation**: Official Gazebo, ROS 2, and Unity docs (cited in chapters)

---

## Next Steps

1. **Phase 0 Research** (`/sp.plan` → research.md): Resolve technical details about Gazebo/Unity versions and best practices
2. **Phase 1 Design** (`/sp.plan` → data-model.md, quickstart.md, templates): Create content architecture and chapter structure
3. **Phase 2 Tasks** (`/sp.tasks`): Break content writing into individual tasks
4. **Implementation** (`/sp.implement`): Execute content generation and validation
5. **Review & Deploy**: Merge to main, update sidebar, deploy to GitHub Pages

