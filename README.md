# Autonomous Fleet Coordination & Software Simulation of AGV Systems

An advanced, decentralized distributed system architecture developed as a Bachelor Thesis project at the University of Novi Sad, Faculty of Technical Sciences. 

This project evolves traditional Automated Guided Vehicles (AGVs) that halt at static path obstructions into intelligent Autonomous Mobile Robots (AMRs). The system evaluates autonomous path-planning, decentralized fleet collision-avoidance, and integration wrappers alongside simulated enterprise resource platforms.

---

## 🏗️ System Architecture & Technology Stack

The framework runs across three decoupled, cross-language processing nodes communicating asynchronously via a central **MQTT broker** loop:

![System of communication](\pics\agv.jpg)

*   **Vehicle Core Subsystem (C++17):** High-performance independent runtime processes providing localized memory control, path finding, obstacle detection algorithms, and state machines.
*   **Operator Control Panel Dashboard (Java):** Desktop telemetry visualizer built using the standard `Swing` graphic engine framework to monitor positions, deploy obstacles, and dispatch routing overrides.
*   **Enterprise Integration Wrapper (Python):** Event-driven backend simulating automated business workflows (`WMS`/`ERP` endpoints) to dynamically evaluate assignment loading across fleet nodes.
*   **Communication Layer:** Leverages `libmosquitto` (C++) and `Eclipse Paho` (Java) to pass strict `JSON` payload structures over localized broker topics (`agv/<id>/telemetry`).

---

## 🚀 Key Algorithmic Implementations

### 1. Multi-Tier Path Planning & Trajectory Smoothing
When localized safety zones register a route blockage, vehicles evaluate a hierarchical bypass maneuver:
*   **Local Corridor Deviation:** Fast, proactive lane shifts onto parallel marker tracks.
*   **Global Path Search ($A^{*}$):** Fallback heuristic path-generation across discretized spatial maps.
*   **Trajectory Modification:** A localized *String Pulling / Funnel Algorithm* filters raw grid nodes, reducing redundant orientation shifts to produce smoother linear paths.

### 2. Decentralized Fleet Coordination & Prioritization
Avoids single points of failure by managing traffic collisions locally on the vehicle end:
*   **Synthetic Obstacle Mapping:** Fleet nodes subscribe to broad telemetry feeds (`agv/+/telemetry`), mapping foreign coordinates directly into local collision loops as active moving obstacles.
*   **Deterministic Priority Resolution:** Deadlocks are resolved gracefully by enforcing non-blocking priority rights mapped directly to a low-integer numerical vehicle identification scheme.

---

## 📊 Quantitative Benchmarks & Testing Performance

The operational algorithm was validated through intensive monte-carlo batch scenarios executing **1,000 independent randomized tracking loops** to profile real-world efficacy:

### Fleet Mission Completion Metric

| Operational Profiling Mode | Target Completion Rate | Average Loop Metric | Efficiency Progress |
| :--- | :---: | :---: | :---: |
| **Standard Baseline AGV** (Halt on Block) | `50.5%` | `223.5 Ticks` | Base Profile |
| **Autonomous AMR Upgrade** (Bypass Enabled) | **`70.5%`** | `232.1 Ticks` | **+20.0% Net Success Increase** |

*   **Dynamic Obstacle Recovery:** In scenarios where target path blocking clears dynamically, the bypass router registers an empirical timing improvement averaging **9.4 seconds saved per active conflict loop**.

---

## Source Component Blueprint

*   `/AGV` — Core application logic and algorithmic pipelines.
    *   `PathPlanner.cpp` / `PathPlanner.h` — Custom grid mapping and A* node-filtering calculations.
    *   `Sensor.cpp` / `Sensor.h` — Protective proximity boundary field tracking and target isolation logic.
    *   `main.cpp` — Master thread tracking loop linking module synchronization hooks.
*   `/AgvDashboard` — Monitor dash layout scripts using Java event threads.
*   `/simulation` — Order generation workflows and allocation load calculations.

---

## Build & Development Lifecycle

### System Dependencies
Ensure you have the following packages active inside your development environment loop:
*   A compatible standard C++17 compiler (GCC 9+ or MSVC 2019+).
*   Java Development Kit (JDK 11 or higher).
*   Python 3.8+ setup with base mapping packages.
*   Active local Mosquitto MQTT message broker daemon pipeline instance.

### Compilation and Launch
```bash
# 1. Compile the main vehicle execution pipelines via the provided Makefile
make

# 2. Deploy the core MQTT message tracking infrastructure daemon
mosquitto -v

# 3. Fire up the multi-agent vehicle simulators with structural identification tokens
./agv_simulator --id 1

# 4. Launch the Java controller visualization panel
java -jar build/libs/AgvDashboard.jar
```