# Collet: System Architecture and Mathematical Foundations

Collet is a distributed industrial orchestration platform for fleets of Autonomous Mobile Robots (AMRs) and Automated Guided Vehicles (AGVs). It is implemented natively in Gleam targeting the BEAM virtual machine.

This document details the mathematical models, algorithmic guarantees, concurrency architecture, and protocol implementations that govern Collet.

```
+-----------------------------------------------------------------------------------+
|                                 COLLET ECOSYSTEM                                  |
+-----------------------------------------------------------------------------------+
|                                                                                   |
|   +-------------------+    VDA 5050     +-------------------------------------+   |
|   |  External ERP /   | <============>  |      HTTP REST / Prometheus API     |   |
|   |  WMS / Fleet Mgr  |     (JSON)      |            (gleam_http)             |   |
|   +-------------------+                 +-------------------------------------+   |
|                                                            |                      |
|                                                            v                      |
|   +---------------------------------------------------------------------------+   |
|   |                       BEAM SUPERVISOR ROOT                                |   |
|   |                   (gleam/otp/static_supervisor)                           |   |
|   +---------------------------------------------------------------------------+   |
|                                |                                                  |
|                                v                                                  |
|   +---------------------------------------------------------------------------+   |
|   |                  FLEET MANAGER COORDINATOR ACTOR                          |   |
|   |                      (collet/actor/fleet_manager)                         |   |
|   +---------------------------------------------------------------------------+   |
|           |                           |                           |               |
|           v                           v                           v               |
|   +-----------------+         +-----------------+         +-----------------+     |
|   |  ROBOT ACTOR 1  |         |  ROBOT ACTOR 2  |         |  ROBOT ACTOR N  |     |
|   |  (amr-session)  |         |  (amr-session)  |         |  (amr-session)  |     |
|   +-----------------+         +-----------------+         +-----------------+     |
|           |                           |                           |               |
|           +---------------------------+---------------------------+               |
|                                       |                                           |
|                                       v                                           |
|   +---------------------------------------------------------------------------+   |
|   |                  MULTI-AGENT PATHFINDING ENGINE (MAPF)                    |   |
|   |  +------------------------+  +-----------------------------------------+  |   |
|   |  | Conflict-Based Search  |  | Space-Time A* with Dynamic Reservations |  |   |
|   |  |         (CBS)          |  |         (collet/routing/cbs)            |  |   |
|   |  +------------------------+  +-----------------------------------------+  |   |
|   +---------------------------------------------------------------------------+   |
|                                       |                                           |
|                                       v                                           |
|   +---------------------------------------------------------------------------+   |
|   |                     KINODYNAMIC TRAJECTORY PIPELINE                       |   |
|   |  +------------------------+  +-----------------------------------------+  |   |
|   |  | Trapezoidal Profiling  |  | Continuous Collision Swept SAT Detection|  |   |
|   |  |   (collet/kinematics)  |  |         (collet/reservation/sweep)      |  |   |
|   |  +------------------------+  +-----------------------------------------+  |   |
|   +---------------------------------------------------------------------------+   |
|                                                                                   |
+-----------------------------------------------------------------------------------+
```

---

## 1. Multi-Agent Pathfinding (MAPF) and Collision Resolution

### 1.1 Spatio-Temporal Graph Formulation

The warehouse facility is modeled as a directed topological graph $G = (V, E)$, where $V$ represents discrete physical waypoints, charging contacts, and buffer spurs, and $E$ represents traversable aisle segments.

Pathfinding operates over the time-expanded configuration space $G_T = V \times \mathbb{R}_{\ge 0}$. An agent path is defined as a sequence of spatio-temporal waypoints:

$$\pi_i = \langle (v_0, t_0), (v_1, t_1), \dots, (v_k, t_k) \rangle$$

where $v_j \in V$ and $t_j < t_{j+1}$.

### 1.2 Conflict Classification

Collet classifies multi-agent conflicts into five distinct categories:

1. **Vertex Conflict**: Two agents occupy the same topological node at overlapping time intervals:
   $$\exists i \ne j : v_i(t) = v_j(t)$$
2. **Edge Conflict**: Two agents traverse the same directed edge concurrently, violating minimum following headway.
3. **Swap Conflict (Head-On Collision)**: Two agents attempt to cross the same undirected corridor edge in opposing directions:
   $$\exists i \ne j : (u \to v \in \pi_i) \land (v \to u \in \pi_j) \land [t_{i, start}, t_{i, end}] \cap [t_{j, start}, t_{j, end}] \ne \emptyset$$
4. **Trailing Headway Conflict**: An agent follows another vehicle too closely such that the stopping distance requirement is violated.
5. **Footprint Swept Volume Conflict**: Even if topological centerpoints are disjoint, the physical Oriented Bounding Boxes (OBBs) of vehicle chassis collide during turning or corner negotiation.

### 1.3 Conflict-Based Search (CBS)

Collet implements a two-level Conflict-Based Search:

- **High-Level Search**: Explores a Constraint Tree (CT). Each node $C$ in the CT contains:
  - A set of spatio-temporal avoidance constraints across agents.
  - A set of individual routes $\{\pi_1, \pi_2, \dots, \pi_n\}$.
  - Total fleet cost $J = \sum_{i=1}^n \text{cost}(\pi_i)$.
- **Low-Level Search**: Single-Agent Space-Time A* searching for the minimum-time kinodynamically feasible route satisfying all constraints assigned to agent $i$ by the high-level node.

When a conflict $(a_i, a_j, \text{resource}, [t_s, t_e])$ is detected:
1. High-level search branches into two child nodes:
   - Child 1: Adds constraint $(a_i, \text{resource}, [t_s, t_e])$.
   - Child 2: Adds constraint $(a_j, \text{resource}, [t_s, t_e])$.
2. Each child recalculates the low-level path for its newly constrained agent.
3. High-level priority queue pops the CT node with minimal total cost.

---

## 2. Kinodynamic Trajectory Generation

### 2.1 Trapezoidal and S-Curve Velocity Profiling

For a path segment of arc length $s$, Collet bounds linear acceleration $a_{max}$, linear deceleration $d_{max}$, and maximum velocity $v_{max}$.

The critical distance required to reach peak velocity from rest is:

$$s_{crit} = \frac{v_{max}^2}{2 a_{max}} + \frac{v_{max}^2}{2 d_{max}}$$

- If $s \ge s_{crit}$, a three-phase trapezoidal profile is formed (Acceleration, Cruise, Deceleration).
- If $s < s_{crit}$, a triangular profile is computed with reachable peak velocity:

$$v_{peak} = \sqrt{\frac{2 a_{max} d_{max} s}{a_{max} + d_{max}}}$$

### 2.2 Cubic Spline Curvature and Lateral Acceleration

Path turning transitions are parameterized as parametric cubic curves $C(u) = [x(u), y(u)]^T$ for $u \in [0, 1]$.

Instantaneous curvature $\kappa(u)$ is computed via:

$$\kappa(u) = \frac{\dot{x}(u) \ddot{y}(u) - \dot{y}(u) \ddot{x}(u)}{\left(\dot{x}(u)^2 + \dot{y}(u)^2\right)^{3/2}}$$

To prevent wheel slippage and lateral rollover, linear velocity through turns is dynamically constrained by:

$$v_{turn}(u) \le \sqrt{\frac{a_{lateral, max}}{|\kappa(u)|}}$$

---

## 3. Continuous Collision Detection (CCD) via Swept SAT

Discrete point-in-time collision checks risk tunneling at industrial speeds ($v \ge 2.0\text{ m/s}$). Collet synthesizes a convex swept polygon between consecutive trajectory knots $K_a$ and $K_b$:

$$\text{SweptPolygon}(K_a, K_b) = \text{ConvexHull}\left(\text{Footprint}(K_a) \cup \text{Footprint}(K_b)\right)$$

Collision testing is executed using the Separating Axis Theorem (SAT):
Two convex planar polygons $P$ and $Q$ are disjoint if and only if there exists a projection axis $L$ along which their 1D projections do not overlap:

$$\max_{p \in P}(p \cdot \hat{n}) < \min_{q \in Q}(q \cdot \hat{n}) \quad \lor \quad \max_{q \in Q}(q \cdot \hat{n}) < \min_{p \in P}(p \cdot \hat{n})$$

Collet tests all edge normal vectors of $P$ and $Q$.

---

## 4. Deadlock Detection and Dynamic Preemption

When unforeseen obstacles or external human traffic stall an AMR, circular wait conditions emerge.

### 4.1 Wait-For Graph (WFG)

The system maintains a directed graph $W = (A, D)$, where $A$ is the set of active AMRs and $(a_i \to a_j) \in D$ indicates agent $a_i$ is waiting for a topological resource reserved or occupied by $a_j$.

Directed cycles in $W$ are detected using depth-first search cycle detection.

### 4.2 Autonomous Priority-Based Resolution

When a cycle $C = \langle a_1, a_2, \dots, a_k, a_1 \rangle$ is detected:
1. The coordinator identifies the victim agent with the lowest operational priority rank:
   $$a_{victim} = \arg\min_{a \in C} \text{Priority}(a)$$
2. The coordinator evaluates adjacent topological options for $a_{victim}$:
   - **Strategy 1 (Yield to Buffer)**: If an adjacent parking buffer or holding bay is reachable, $a_{victim}$ is commanded to pull aside into the buffer.
   - **Strategy 2 (Reverse to Junction)**: If a predecessor junction is vacant, $a_{victim}$ reverses to clear the single-lane aisle.
   - **Strategy 3 (Dynamic Detour)**: The high-priority beneficiary agent is granted reservation lock while $a_{victim}$ undergoes a full Space-Time A* reroute.

---

## 5. Electrochemistry and Battery Energy Management

Collet incorporates an electrochemistry and tractive power model:

### 5.1 Mechanical Tractive Draw

$$P_{motion} = \frac{1}{\eta_{drive}} \left[ (m_{veh} + m_{payload}) g C_{rr} v + (m_{veh} + m_{payload}) a v \right] + P_{aux}$$

where:
- $g = 9.80665\text{ m/s}^2$
- $C_{rr} = 0.018$ (polyurethane wheels on industrial epoxy floor)
- $\eta_{drive} = 0.85$ (inverter and brushless motor drivetrain efficiency)
- $P_{aux} = 45\text{ W}$ (onboard LiDAR safety scanners, depth cameras, industrial compute)

### 5.2 CC-CV Charging Profile

During automated docking, charging power $P_{chg}$ follows a Constant Current / Constant Voltage (CC-CV) curve. Below 80% State of Charge (SoC), maximum power is accepted. Above 80%, power is tapered linearly to prevent lithium plating:

$$P_{effective} = P_{dock} \cdot \min\left(1.0, \frac{1.0 - \text{SoC}}{0.20}\right) \cdot \eta_{chg}$$

---

## 6. BEAM Actor Architecture and Supervision Hierarchy

Collet leverages Erlang OTP primitives via `gleam_otp` to guarantee fault tolerance:

```
                  +--------------------------------+
                  |    StaticSupervisor Root       |
                  |     (Strategy: OneForOne)      |
                  +--------------------------------+
                                  |
            +---------------------+---------------------+
            |                                           |
            v                                           v
+-----------------------+                   +-----------------------+
|  FleetManager Actor   |                   |  RobotSession Actors  |
|  - World Topology     |                   |  - Pose Tracking      |
|  - Reservation Table  |                   |  - Odometry & Slip    |
|  - Mission Dispatcher |                   |  - VDA 5050 Codecs    |
+-----------------------+                   +-----------------------+
```

- **Fault Isolation**: If a network timeout occurs with an individual AMR, only its `RobotSession` process fails or transitions to disconnected mode. The central `FleetManager` and peer vehicle sessions remain fully operational.
- **Mailbox Concurrency**: State updates and telemetries are processed sequentially per actor, eliminating shared-memory data races without explicit mutex locks.
