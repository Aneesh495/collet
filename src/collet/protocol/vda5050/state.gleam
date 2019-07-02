import collet/protocol/vda5050/action.{type ActionState}
import collet/protocol/vda5050/header.{type Header}
import gleam/option.{type Option}

// Operational driving mode of the vehicle.
pub type OperatingMode {
  Automatic
  Semiautomatic
  Manual
  Service
  TeachIn
}

// Emergency stop state classification.
pub type EStopState {
  // E-stop is physically engaged; motion hardware interlock active.
  AutoAck
  ManualAck
  RemoteAck
  NoneEStop
}

// Severity tier of an error or warning reported by the AMR.
pub type ErrorLevel {
  Warning
  Fatal
}

// Formal protocol error or diagnostic warning message.
pub type ProtocolError {
  ProtocolError(
    error_type: String,
    error_description: Option(String),
    error_level: ErrorLevel,
  )
}

// Safety hardware sub-system state report.
pub type SafetyState {
  SafetyState(
    e_stop: EStopState,
    field_violation: Bool,
  )
}

// Real-time battery state of charge telemetry.
pub type BatteryState {
  BatteryState(
    battery_charge_percent: Float,
    battery_voltage: Option(Float),
    charging: Bool,
    reach_meters: Option(Float),
  )
}

// Estimated Cartesian position and orientation of the AMR on the map.
pub type AgvPosition {
  AgvPosition(
    x: Float,
    y: Float,
    theta: Float,
    map_id: String,
    position_initialized: Bool,
    localization_score: Option(Float),
  )
}

// Instantaneous velocity vector in local coordinate frame.
pub type Velocity {
  Velocity(
    vx_mps: Float,
    vy_mps: Float,
    omega_radps: Float,
  )
}

// Complete VDA 5050 periodic state telemetry report broadcast by AMR.
pub type StateMessage {
  StateMessage(
    header: Header,
    order_id: String,
    order_update_id: Int,
    last_node_id: String,
    last_node_sequence_id: Int,
    driving: Bool,
    operating_mode: OperatingMode,
    safety_state: SafetyState,
    battery_state: BatteryState,
    agv_position: Option(AgvPosition),
    velocity: Option(Velocity),
    action_states: List(ActionState),
    errors: List(ProtocolError),
  )
}

// Converts OperatingMode to standard VDA 5050 string.
pub fn operating_mode_to_string(mode: OperatingMode) -> String {
  case mode {
    Automatic -> "AUTOMATIC"
    Semiautomatic -> "SEMIAUTOMATIC"
    Manual -> "MANUAL"
    Service -> "SERVICE"
    TeachIn -> "TEACHIN"
  }
}

// Parses standard VDA 5050 operating mode string.
pub fn string_to_operating_mode(str: String) -> Result(OperatingMode, Nil) {
  case str {
    "AUTOMATIC" -> Ok(Automatic)
    "SEMIAUTOMATIC" -> Ok(Semiautomatic)
    "MANUAL" -> Ok(Manual)
    "SERVICE" -> Ok(Service)
    "TEACHIN" -> Ok(TeachIn)
    _ -> Error(Nil)
  }
}
