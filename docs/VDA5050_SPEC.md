# Collet: VDA 5050 Protocol Implementation Guide

Collet provides native compliance with the German Automotive Industry Association (VDA) and VDMA standard **VDA 5050 version 2.0.0** ("Interface for Automated Guided Vehicles and Autonomous Mobile Robots").

This guide specifies how Collet serializes, validates, ingests, and dispatches VDA 5050 compliant orders, actions, states, and connection messages.

---

## 1. Message Structure and Envelope

Every VDA 5050 protocol interaction wraps its payload in a standardized envelope header:

```json
{
  "headerId": 1042,
  "timestamp": "2026-11-18T10:45:00.000Z",
  "version": "2.0.0",
  "manufacturer": "ColletRobotics",
  "serialNumber": "AMR-FACILITY-04"
}
```

### Header Fields
- `headerId` (Integer): Monotonically increasing sequence counter per sender.
- `timestamp` (String): ISO 8601 UTC timestamp of message generation.
- `version` (String): Fixed to `"2.0.0"`.
- `manufacturer` (String): Hardware or software vendor identifier.
- `serialNumber` (String): Globally unique physical robot or fleet controller identifier.

---

## 2. Order Message Specification

Orders represent navigation paths and actions commanded to an AMR.

### 2.1 Order JSON Format

```json
{
  "headerId": 2001,
  "timestamp": "2026-11-18T10:45:00.000Z",
  "version": "2.0.0",
  "manufacturer": "ColletRobotics",
  "serialNumber": "AMR-FACILITY-04",
  "orderId": "ORD-2026-9901",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "N_PICKUP_A",
      "sequenceId": 0,
      "released": true,
      "nodePosition": {
        "x": 12.5,
        "y": 4.0,
        "theta": 0.0,
        "mapId": "FACILITY_MAIN"
      },
      "actions": [
        {
          "actionType": "pick",
          "actionId": "ACT_LIFT_01",
          "actionDescription": "Lift EUR pallet at station A",
          "blockingType": "HARD"
        }
      ]
    },
    {
      "nodeId": "N_DROPOFF_B",
      "sequenceId": 2,
      "released": true,
      "nodePosition": {
        "x": 35.0,
        "y": 14.0,
        "theta": 1.5708,
        "mapId": "FACILITY_MAIN"
      },
      "actions": [
        {
          "actionType": "drop",
          "actionId": "ACT_DROP_02",
          "actionDescription": "Lower EUR pallet at station B",
          "blockingType": "HARD"
        }
      ]
    }
  ],
  "edges": [
    {
      "edgeId": "E_PICKUP_DROPOFF",
      "sequenceId": 1,
      "released": true,
      "startNodeId": "N_PICKUP_A",
      "endNodeId": "N_DROPOFF_B",
      "maxSpeed": 1.8,
      "maxHeight": 2.5,
      "minHeight": 0.0,
      "corridorWidth": 2.0,
      "direction": "FORWARD",
      "actions": []
    }
  ]
}
```

### 2.2 Sequence Ordering Rules
- Node and edge sequences must interleave strictly: `Node(0) -> Edge(1) -> Node(2) -> Edge(3) -> ... -> Node(2n)`.
- Sequence IDs must be strictly monotonically increasing.
- For an edge $E_{2k+1}$, its `startNodeId` must equal $N_{2k}.\text{nodeId}$ and its `endNodeId` must equal $N_{2k+2}.\text{nodeId}$.
- Violations are rejected immediately with HTTP 400 Bad Request by `collet/api/router.gleam`.

---

## 3. Action Protocol and Blocking Semantics

Actions represent non-navigation tasks executed by the AMR (e.g., pallet lifting, RFID scanning, door opening requests, battery charging).

### Blocking Classifications
1. **HARD**: Halts vehicle motion entirely until the action is fully completed and confirmed.
2. **SOFT**: Vehicle may continue slow maneuvering if motion does not interfere with the action.
3. **NONE**: Action executes concurrently while cruising at full rated speed (e.g. barcode reading or environmental telemetry sampling).

---

## 4. State Message Telemetry

Robots stream state updates at regular intervals (nominal 1 Hz) or upon state transition events:

```json
{
  "headerId": 3050,
  "timestamp": "2026-11-18T10:45:05.000Z",
  "version": "2.0.0",
  "manufacturer": "ColletRobotics",
  "serialNumber": "AMR-FACILITY-04",
  "orderId": "ORD-2026-9901",
  "orderUpdateId": 0,
  "lastNodeId": "N_PICKUP_A",
  "lastNodeSequenceId": 0,
  "driving": true,
  "operatingMode": "AUTOMATIC",
  "batteryState": {
    "batteryCharge": 84.5,
    "batteryVoltage": 51.2,
    "batteryHealth": 98.0,
    "charging": false,
    "reach": 1240.0
  },
  "safetyState": {
    "eStop": "NONE",
    "fieldViolation": false
  }
}
```

---

## 5. Connection State Lifecycle

Collet maintains heartbeat tracking over BEAM actors. If no telemetry is received within the configured connection timeout, the robot session transitions:

```
[ONLINE] ---> (heartbeat missed > 5s) ---> [WARNING] ---> (heartbeat missed > 15s) ---> [OFFLINE]
```

When an offline transition occurs:
1. Active reservations held by the vehicle are locked to prevent collisions.
2. A WFG deadlock check is triggered to prevent cascaded stalls.
3. The dispatcher re-auctions pending tasks originally queued for that robot.
