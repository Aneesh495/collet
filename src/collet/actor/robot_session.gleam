import collet/energy/battery.{type BatteryModel}
import collet/geometry/transform.{type Transform2D}
import collet/protocol/vda5050/order.{type Order}
import collet/reservation/table.{type VehicleId}
import collet/topology/node.{type NodeId}
import gleam/erlang/process.{type Subject}
import gleam/option.{type Option, None, Some}
import gleam/otp/actor

// Telemetry snapshot of an individual AMR actor.
pub type VehicleTelemetry {
  VehicleTelemetry(
    vehicle_id: VehicleId,
    current_node_id: NodeId,
    pose: Transform2D,
    battery_soc: Float,
    is_e_stopped: Bool,
    active_order_id: Option(String),
    last_heartbeat_sec: Float,
  )
}

// Internal persistent state for the robot session actor.
pub type SessionState {
  SessionState(
    vehicle_id: VehicleId,
    current_node_id: NodeId,
    pose: Transform2D,
    battery: BatteryModel,
    active_order: Option(Order),
    is_e_stopped: Bool,
    last_heartbeat_sec: Float,
  )
}

// Messages accepted by the robot session actor.
pub type RobotMessage {
  Heartbeat(timestamp_sec: Float)
  UpdatePosition(node_id: NodeId, pose: Transform2D)
  UpdateBattery(battery: BatteryModel)
  AssignOrder(order: Order, reply_to: Subject(Result(Nil, String)))
  ClearOrder
  GetTelemetry(reply_to: Subject(VehicleTelemetry))
  EmergencyStop(reason: String, reply_to: Subject(Nil))
  ResetEmergencyStop(reply_to: Subject(Nil))
  Stop
}

// Starts a new concurrent robot session actor process.
pub fn start(
  vehicle_id: VehicleId,
  initial_node: NodeId,
  initial_battery: BatteryModel,
) -> Result(actor.Started(Subject(RobotMessage)), actor.StartError) {
  let initial_state =
    SessionState(
      vehicle_id: vehicle_id,
      current_node_id: initial_node,
      pose: transform.identity(),
      battery: initial_battery,
      active_order: None,
      is_e_stopped: False,
      last_heartbeat_sec: 0.0,
    )

  actor.new(initial_state)
  |> actor.on_message(handle_message)
  |> actor.start
}

// Message handler for the robot session actor.
fn handle_message(
  state: SessionState,
  message: RobotMessage,
) -> actor.Next(SessionState, RobotMessage) {
  case message {
    Heartbeat(t) -> {
      actor.continue(SessionState(..state, last_heartbeat_sec: t))
    }

    UpdatePosition(nid, p) -> {
      actor.continue(
        SessionState(..state, current_node_id: nid, pose: p),
      )
    }

    UpdateBattery(bat) -> {
      actor.continue(SessionState(..state, battery: bat))
    }

    AssignOrder(new_order, client) -> {
      case state.is_e_stopped {
        True -> {
          process.send(
            client,
            Error("Cannot assign order to emergency-stopped vehicle"),
          )
          actor.continue(state)
        }
        False -> {
          case state.active_order {
            Some(curr) -> {
              process.send(
                client,
                Error(
                  "Vehicle is already executing active order " <> curr.order_id,
                ),
              )
              actor.continue(state)
            }
            None -> {
              process.send(client, Ok(Nil))
              actor.continue(
                SessionState(..state, active_order: Some(new_order)),
              )
            }
          }
        }
      }
    }

    ClearOrder -> {
      actor.continue(SessionState(..state, active_order: None))
    }

    GetTelemetry(client) -> {
      let telem =
        VehicleTelemetry(
          vehicle_id: state.vehicle_id,
          current_node_id: state.current_node_id,
          pose: state.pose,
          battery_soc: battery.state_of_charge(state.battery),
          is_e_stopped: state.is_e_stopped,
          active_order_id: option.map(state.active_order, fn(o) { o.order_id }),
          last_heartbeat_sec: state.last_heartbeat_sec,
        )
      process.send(client, telem)
      actor.continue(state)
    }

    EmergencyStop(_reason, client) -> {
      process.send(client, Nil)
      actor.continue(
        SessionState(
          ..state,
          is_e_stopped: True,
          active_order: None,
        ),
      )
    }

    ResetEmergencyStop(client) -> {
      process.send(client, Nil)
      actor.continue(SessionState(..state, is_e_stopped: False))
    }

    Stop -> actor.stop()
  }
}

// Client API: Queries telemetry from the robot session process.
pub fn get_telemetry(
  session: Subject(RobotMessage),
  timeout_ms: Int,
) -> VehicleTelemetry {
  process.call(session, timeout_ms, GetTelemetry)
}

// Client API: Dispatches an order to the robot session process.
pub fn assign_order(
  session: Subject(RobotMessage),
  order: Order,
  timeout_ms: Int,
) -> Result(Nil, String) {
  process.call(session, timeout_ms, fn(reply_to) {
    AssignOrder(order, reply_to)
  })
}

// Client API: Sends an emergency stop command.
pub fn trigger_emergency_stop(
  session: Subject(RobotMessage),
  reason: String,
  timeout_ms: Int,
) -> Nil {
  process.call(session, timeout_ms, fn(reply_to) {
    EmergencyStop(reason, reply_to)
  })
}

// Client API: Sends heartbeat ping.
pub fn ping_heartbeat(session: Subject(RobotMessage), timestamp: Float) -> Nil {
  process.send(session, Heartbeat(timestamp))
}
