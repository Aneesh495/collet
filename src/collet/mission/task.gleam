import collet/reservation/table.{type VehicleId}
import collet/topology/node.{type NodeId}
import gleam/option.{type Option, None, Some}

// Priority classification for logistics task scheduling and preemption.
pub type PriorityClass {
  // Production-line critical emergency parts delivery.
  Critical
  // High-priority urgent shipment with strict SLA deadlines.
  Express
  // Standard routine warehouse transport operation.
  Standard
  // Low-priority opportunistic repositioning or staging.
  Opportunity
  // Background maintenance or idle repositioning.
  Background
}

// Converts a priority class into an integer weight for auction bidding and preemption ranking.
pub fn priority_to_int(priority: PriorityClass) -> Int {
  case priority {
    Critical -> 100
    Express -> 75
    Standard -> 50
    Opportunity -> 25
    Background -> 10
  }
}

// Converts an integer weight back to the closest priority class.
pub fn int_to_priority(val: Int) -> PriorityClass {
  case val >= 90 {
    True -> Critical
    False ->
      case val >= 70 {
        True -> Express
        False ->
          case val >= 40 {
            True -> Standard
            False ->
              case val >= 20 {
                True -> Opportunity
                False -> Background
              }
          }
      }
  }
}

// Operational state lifecycle of a logistics task.
pub type TaskStatus {
  Pending
  Assigned(vehicle_id: VehicleId, assigned_at: Float)
  InTransitToPickup(vehicle_id: VehicleId, start_time: Float)
  Loading(vehicle_id: VehicleId, started_at: Float)
  InTransitToDropoff(vehicle_id: VehicleId, start_time: Float)
  Unloading(vehicle_id: VehicleId, started_at: Float)
  Completed(completion_time: Float)
  Failed(reason: String)
  Cancelled
}

// Specific payload and equipment specifications for transport.
pub type PayloadSpec {
  PayloadSpec(
    payload_type: String,
    weight_kg: Float,
    width_m: Float,
    length_m: Float,
    height_m: Float,
  )
}

// Discrete fleet mission task dispatched to one or more AMRs.
pub type MissionTask {
  // Point-to-point material transport mission.
  TransportMission(
    task_id: String,
    pickup_node_id: NodeId,
    dropoff_node_id: NodeId,
    payload: PayloadSpec,
    priority: PriorityClass,
    deadline_sec: Float,
    created_at_sec: Float,
    status: TaskStatus,
  )
  // Automated battery replenishment mission.
  ChargingMission(
    task_id: String,
    charger_node_id: NodeId,
    target_soc: Float,
    priority: PriorityClass,
    status: TaskStatus,
  )
  // Fleet rebalancing repositioning to a staging buffer.
  RelocateMission(
    task_id: String,
    target_node_id: NodeId,
    priority: PriorityClass,
    status: TaskStatus,
  )
}

// Creates a new pending material transport task.
pub fn new_transport(
  task_id: String,
  pickup: NodeId,
  dropoff: NodeId,
  payload_type: String,
  weight_kg: Float,
  priority: PriorityClass,
  deadline_sec: Float,
  now_sec: Float,
) -> MissionTask {
  TransportMission(
    task_id: task_id,
    pickup_node_id: pickup,
    dropoff_node_id: dropoff,
    payload: PayloadSpec(
      payload_type: payload_type,
      weight_kg: weight_kg,
      width_m: 0.8,
      length_m: 1.2,
      height_m: 1.0,
    ),
    priority: priority,
    deadline_sec: deadline_sec,
    created_at_sec: now_sec,
    status: Pending,
  )
}

// Creates a new charging mission.
pub fn new_charging(
  task_id: String,
  charger_node: NodeId,
  target_soc: Float,
  now_sec: Float,
) -> MissionTask {
  let _ = now_sec
  ChargingMission(
    task_id: task_id,
    charger_node_id: charger_node,
    target_soc: target_soc,
    priority: Express,
    status: Pending,
  )
}

// Returns the unique identifier of the mission task.
pub fn id(task: MissionTask) -> String {
  case task {
    TransportMission(task_id: tid, ..) -> tid
    ChargingMission(task_id: tid, ..) -> tid
    RelocateMission(task_id: tid, ..) -> tid
  }
}

// Returns the priority class of the mission task.
pub fn priority(task: MissionTask) -> PriorityClass {
  case task {
    TransportMission(priority: p, ..) -> p
    ChargingMission(priority: p, ..) -> p
    RelocateMission(priority: p, ..) -> p
  }
}

// Returns the active status of the task.
pub fn status(task: MissionTask) -> TaskStatus {
  case task {
    TransportMission(status: s, ..) -> s
    ChargingMission(status: s, ..) -> s
    RelocateMission(status: s, ..) -> s
  }
}

// Assigns a vehicle to a pending task.
pub fn assign(
  task: MissionTask,
  vehicle_id: VehicleId,
  now_sec: Float,
) -> Result(MissionTask, String) {
  case status(task) {
    Pending -> {
      let new_st = Assigned(vehicle_id: vehicle_id, assigned_at: now_sec)
      Ok(update_status(task, new_st))
    }
    _ -> Error("Cannot assign task that is not in Pending status")
  }
}

// Advances the task to InTransitToPickup state.
pub fn start_transit_to_pickup(
  task: MissionTask,
  vehicle_id: VehicleId,
  now_sec: Float,
) -> Result(MissionTask, String) {
  case task {
    TransportMission(..) -> {
      let new_st =
        InTransitToPickup(vehicle_id: vehicle_id, start_time: now_sec)
      Ok(update_status(task, new_st))
    }
    _ -> Error("Only transport missions feature transit to pickup")
  }
}

// Transitions task to completed status.
pub fn complete(task: MissionTask, now_sec: Float) -> MissionTask {
  update_status(task, Completed(completion_time: now_sec))
}

// Transitions task to failed status with diagnostic reason.
pub fn fail(task: MissionTask, reason: String) -> MissionTask {
  update_status(task, Failed(reason: reason))
}

// Transitions task to cancelled status.
pub fn cancel(task: MissionTask) -> MissionTask {
  update_status(task, Cancelled)
}

// Internal helper updating task status across all variant types.
fn update_status(task: MissionTask, new_status: TaskStatus) -> MissionTask {
  case task {
    TransportMission(
      task_id: tid,
      pickup_node_id: p,
      dropoff_node_id: d,
      payload: pay,
      priority: pr,
      deadline_sec: dl,
      created_at_sec: cr,
      status: _,
    ) ->
      TransportMission(
        task_id: tid,
        pickup_node_id: p,
        dropoff_node_id: d,
        payload: pay,
        priority: pr,
        deadline_sec: dl,
        created_at_sec: cr,
        status: new_status,
      )
    ChargingMission(
      task_id: tid,
      charger_node_id: chg,
      target_soc: soc,
      priority: pr,
      status: _,
    ) ->
      ChargingMission(
        task_id: tid,
        charger_node_id: chg,
        target_soc: soc,
        priority: pr,
        status: new_status,
      )
    RelocateMission(task_id: tid, target_node_id: tgt, priority: pr, status: _) ->
      RelocateMission(
        task_id: tid,
        target_node_id: tgt,
        priority: pr,
        status: new_status,
      )
  }
}

// Checks if the task is currently active (not terminal).
pub fn is_active(task: MissionTask) -> Bool {
  case status(task) {
    Completed(_) -> False
    Failed(_) -> False
    Cancelled -> False
    _ -> True
  }
}

// Checks if an active transport mission has breached or will breach its SLA deadline.
pub fn is_tardy(task: MissionTask, current_time_sec: Float) -> Bool {
  case task {
    TransportMission(deadline_sec: dl, ..) -> {
      case status(task) {
        Completed(comp_t) -> comp_t >. dl
        _ -> current_time_sec >. dl
      }
    }
    _ -> False
  }
}

// Extracts assigned vehicle ID if the task is currently assigned to a robot.
pub fn assigned_vehicle(task: MissionTask) -> Option(VehicleId) {
  case status(task) {
    Assigned(vid, _) -> Some(vid)
    InTransitToPickup(vid, _) -> Some(vid)
    Loading(vid, _) -> Some(vid)
    InTransitToDropoff(vid, _) -> Some(vid)
    Unloading(vid, _) -> Some(vid)
    _ -> None
  }
}
