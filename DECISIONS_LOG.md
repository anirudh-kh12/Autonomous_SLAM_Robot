# Design Decisions Log

Record significant technical decisions, their rationale and any conditions that could cause them to change.

## D-001 — Primary application

**Status:** Agreed in principle

**Decision:** Indoor warehouse material transport.

**Reason:** Provides a practical environment for mapping, localization, navigation and obstacle handling.

## D-002 — Simulation-first development

**Status:** Agreed

**Decision:** Validate algorithms in simulation before constructing the complete physical robot.

**Reason:** Reduces avoidable hardware expense and makes algorithm behaviour easier to understand.

## D-003 — User interface

**Status:** Agreed in principle

**Decision:** Develop a browser-based dashboard for live monitoring, manual control and autonomous goal selection.

**Reason:** One interface can support laptops and phones without initially requiring separate native applications.

## D-004 — Wireless communication

**Status:** Proposed

**Decision:** Use local Wi-Fi as the primary dashboard communication method.

**Reason:** Suitable for map visualization, telemetry and command communication.

## D-005 — Drive configuration

**Status:** Open

**Options:** Two drive motors with a caster; four-motor skid-steer.

**Next step:** Compare mechanical complexity, odometry, wheel scrub, power and available hardware before deciding.

## D-006 — Obstacle sensors

**Status:** Proposed

**Decision:** Use 2D LiDAR as the primary mapping sensor, with optional ultrasonic sensors for supplementary distance monitoring.

**Safety note:** Ultrasonic sensors will not be the sole emergency-stop mechanism.

## D-007 — Mechanical CAD software

**Status:** Open

**Candidates:** Fusion, FreeCAD or another suitable mechanical CAD tool.

**Next step:** Confirm software availability and learn the selected tool when mechanical design begins.

## D-008 — Main computer and LiDAR

**Status:** Open

**Next step:** Check the lab inventory and software compatibility before procurement.

## D-009 — MATLAB products

**Status:** In progress

**Current environment:** MATLAB R2026a Update 5; MATLAB and Control System Toolbox were listed by `ver`.

**Next step:** Finish installing and verifying the required robotics products.

---

## Decision record template

### D-XXX — Decision title

**Status:** Proposed / Open / Agreed / Finalized

**Decision:**

**Alternatives considered:**

**Reasoning:**

**Trade-offs:**

**Evidence or tests:**

**Conditions for reconsideration:**