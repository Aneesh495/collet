import collet/protocol/vda5050/header.{type Header}
import collet/protocol/vda5050/state.{type AgvPosition, type Velocity}
import gleam/option.{type Option}

// High-frequency visual telemetry stream for supervisory operator dashboards.
// Provides low-latency position and velocity updates independent of heavy state reports.
pub type VisualizationMessage {
  VisualizationMessage(
    header: Header,
    agv_position: AgvPosition,
    velocity: Option(Velocity),
  )
}
