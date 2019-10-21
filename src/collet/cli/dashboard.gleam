import collet/actor/fleet_manager.{type FleetStatusSnapshot}
import collet/actor/robot_session.{type VehicleTelemetry}
import gleam/float
import gleam/int
import gleam/list
import gleam/option.{None, Some}
import gleam/string

// ANSI escape codes for rich terminal styling.
const ansi_reset = "\u{001b}[0m"

const ansi_bold = "\u{001b}[1m"

const ansi_cyan = "\u{001b}[36m"

const ansi_green = "\u{001b}[32m"

const ansi_yellow = "\u{001b}[33m"

const ansi_red = "\u{001b}[31m"

const ansi_dim = "\u{001b}[2m"

// Renders an ASCII battery meter bar from a 0.0 to 1.0 State of Charge.
pub fn render_battery_bar(soc: Float, width: Int) -> String {
  let clamped_soc = case soc <. 0.0 {
    True -> 0.0
    False ->
      case soc >. 1.0 {
        True -> 1.0
        False -> soc
      }
  }

  let filled_chars = float.round(clamped_soc *. int.to_float(width))
  let empty_chars = width - filled_chars

  let filled_str = string.repeat("█", filled_chars)
  let empty_str = string.repeat("░", empty_chars)

  let color = case clamped_soc <. 0.2 {
    True -> ansi_red
    False ->
      case clamped_soc <. 0.5 {
        True -> ansi_yellow
        False -> ansi_green
      }
  }

  let pct = float.round(clamped_soc *. 100.0)
  "["
  <> color
  <> filled_str
  <> ansi_dim
  <> empty_str
  <> ansi_reset
  <> "] "
  <> int.to_string(pct)
  <> "%"
}

// Formats the live fleet status snapshot into a terminal dashboard.
pub fn render_dashboard(
  snapshot: FleetStatusSnapshot,
  clock_sec: Float,
) -> String {
  let header =
    ansi_bold
    <> ansi_cyan
    <> "╔══════════════════════════════════════════════════════════════════════════════╗\n"
    <> "║                 COLLET INDUSTRIAL AMR FLEET ORCHESTRATION                    ║\n"
    <> "╚══════════════════════════════════════════════════════════════════════════════╝"
    <> ansi_reset
    <> "\n"

  let util_pct = float.round(snapshot.utilization_ratio *. 100.0)

  let stats =
    "  "
    <> ansi_bold
    <> "Time: "
    <> ansi_reset
    <> float.to_string(clock_sec)
    <> "s"
    <> "  |  "
    <> ansi_bold
    <> "Fleet Size: "
    <> ansi_reset
    <> int.to_string(snapshot.total_robots)
    <> "  |  "
    <> ansi_bold
    <> "Active Missions: "
    <> ansi_reset
    <> int.to_string(snapshot.active_missions_count)
    <> "  |  "
    <> ansi_bold
    <> "Utilization: "
    <> ansi_reset
    <> int.to_string(util_pct)
    <> "%\n"

  let divider =
    ansi_dim
    <> "--------------------------------------------------------------------------------"
    <> ansi_reset
    <> "\n"

  let table_header =
    ansi_bold
    <> "  VEHICLE ID   CURRENT NODE    POSITION (X, Y)       BATTERY SOC        STATUS\n"
    <> ansi_reset

  let rows =
    list.map(snapshot.telemetries, fn(t) { render_telemetry_row(t) })
    |> string.join("\n")

  header <> "\n" <> stats <> divider <> table_header <> divider <> rows <> "\n"
}

// Formats a single AMR telemetry record into a formatted dashboard row.
fn render_telemetry_row(telem: VehicleTelemetry) -> String {
  let e_stop_indicator = case telem.is_e_stopped {
    True -> ansi_red <> ansi_bold <> "E-STOPPED" <> ansi_reset
    False ->
      case telem.active_order_id {
        Some(oid) -> ansi_green <> "IN_TRANSIT (" <> oid <> ")" <> ansi_reset
        None -> ansi_dim <> "IDLE" <> ansi_reset
      }
  }

  let pos_str =
    "("
    <> float.to_string(int.to_float(float.round(telem.pose.x *. 10.0)) /. 10.0)
    <> ", "
    <> float.to_string(int.to_float(float.round(telem.pose.y *. 10.0)) /. 10.0)
    <> ")"

  let bat_bar = render_battery_bar(telem.battery_soc, 8)

  "  "
  <> string.pad_end(telem.vehicle_id, to: 12, with: " ")
  <> " "
  <> string.pad_end(telem.current_node_id, to: 15, with: " ")
  <> " "
  <> string.pad_end(pos_str, to: 21, with: " ")
  <> " "
  <> string.pad_end(bat_bar, to: 20, with: " ")
  <> " "
  <> e_stop_indicator
}
