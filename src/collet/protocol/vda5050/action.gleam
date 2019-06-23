import collet/protocol/vda5050/header.{type Header}
import gleam/option.{type Option}

// Operational lifecycle status of a requested VDA 5050 action.
pub type ActionStatus {
  Waiting
  Initializing
  Running
  Finished
  Failed
}

// Blocking behavior governing vehicle movement during action execution.
pub type BlockingType {
  // Vehicle continues moving without interruption while action executes in parallel.
  NoneBlocking
  // Soft boundary: Vehicle may not traverse next edge until action has initiated.
  SoftBlocking
  // Hard boundary: Vehicle must remain completely stationary until action reaches Finished status.
  HardBlocking
}

// Generic key-value parameter passed to an action invocation.
pub type ActionParameter {
  ActionParameter(key: String, value: String)
}

// Instant action commanded directly to an AMR outside of regular order progression.
// Used for immediate actions like emergency stops, order cancellation, and manual overrides.
pub type InstantAction {
  InstantAction(
    action_type: String,
    action_id: String,
    blocking_type: BlockingType,
    action_parameters: List(ActionParameter),
  )
}

// InstantActions protocol payload containing a header and a list of urgent actions.
pub type InstantActionsMessage {
  InstantActionsMessage(
    header: Header,
    actions: List(InstantAction),
  )
}

// Execution state tracking the progress of an active action reported in AMR state telemetry.
pub type ActionState {
  ActionState(
    action_id: String,
    action_type: String,
    action_status: ActionStatus,
    result_description: Option(String),
  )
}

// Serializes ActionStatus to standard VDA 5050 uppercase string.
pub fn status_to_string(status: ActionStatus) -> String {
  case status {
    Waiting -> "WAITING"
    Initializing -> "INITIALIZING"
    Running -> "RUNNING"
    Finished -> "FINISHED"
    Failed -> "FAILED"
  }
}

// Parses standard VDA 5050 action status string.
pub fn string_to_status(str: String) -> Result(ActionStatus, Nil) {
  case str {
    "WAITING" -> Ok(Waiting)
    "INITIALIZING" -> Ok(Initializing)
    "RUNNING" -> Ok(Running)
    "FINISHED" -> Ok(Finished)
    "FAILED" -> Ok(Failed)
    _ -> Error(Nil)
  }
}

// Serializes BlockingType to standard VDA 5050 uppercase string.
pub fn blocking_type_to_string(bt: BlockingType) -> String {
  case bt {
    NoneBlocking -> "NONE"
    SoftBlocking -> "SOFT"
    HardBlocking -> "HARD"
  }
}

// Constructs an emergency stop instant action.
pub fn new_emergency_stop(action_id: String) -> InstantAction {
  InstantAction(
    action_type: "emergencyStop",
    action_id: action_id,
    blocking_type: HardBlocking,
    action_parameters: [],
  )
}

// Constructs an order cancellation instant action.
pub fn new_cancel_order(action_id: String, order_id: String) -> InstantAction {
  InstantAction(
    action_type: "cancelOrder",
    action_id: action_id,
    blocking_type: HardBlocking,
    action_parameters: [ActionParameter(key: "orderId", value: order_id)],
  )
}
