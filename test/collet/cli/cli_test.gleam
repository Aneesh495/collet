import collet/actor/fleet_manager.{FleetStatusSnapshot}
import collet/actor/robot_session.{VehicleTelemetry}
import collet/cli/args.{
  BenchmarkMode, DaemonMode, SimulateMode, ValidateTopologyMode,
}
import collet/cli/dashboard
import collet/geometry/transform.{Transform2D}
import gleam/option.{None, Some}
import gleam/string

pub fn cli_argument_parsing_defaults_test() {
  let cfg = args.parse([])
  assert cfg.mode == SimulateMode
  assert cfg.fleet_size == 4
  assert cfg.duration_sec == 60.0
  assert cfg.api_port == 8080
  assert cfg.enable_cbs_solver == True
}

pub fn cli_argument_parsing_flags_test() {
  let cfg1 =
    args.parse(["daemon", "--port", "9090", "--fleet-size", "12", "--no-cbs"])
  assert cfg1.mode == DaemonMode
  assert cfg1.api_port == 9090
  assert cfg1.fleet_size == 12
  assert cfg1.enable_cbs_solver == False

  let cfg2 = args.parse(["validate", "-m", "custom_floor.json"])
  assert cfg2.mode == ValidateTopologyMode
  assert cfg2.map_filepath == "custom_floor.json"

  let cfg3 = args.parse(["benchmark", "-d", "120"])
  assert cfg3.mode == BenchmarkMode
  assert cfg3.duration_sec == 120.0
}

pub fn battery_bar_rendering_test() {
  let bar_full = dashboard.render_battery_bar(1.0, 10)
  assert string.contains(bar_full, "100%") == True
  assert string.contains(bar_full, "██████████") == True

  let bar_half = dashboard.render_battery_bar(0.50, 10)
  assert string.contains(bar_half, "50%") == True

  let bar_empty = dashboard.render_battery_bar(0.0, 10)
  assert string.contains(bar_empty, "0%") == True
  assert string.contains(bar_empty, "░░░░░░░░░░") == True
}

pub fn dashboard_rendering_test() {
  let t1 =
    VehicleTelemetry(
      vehicle_id: "amr-01",
      current_node_id: "NODE_PICKUP_A",
      pose: Transform2D(12.5, 4.2, 0.0),
      battery_soc: 0.88,
      is_e_stopped: False,
      active_order_id: Some("ORDER_99"),
      last_heartbeat_sec: 14.2,
    )

  let t2 =
    VehicleTelemetry(
      vehicle_id: "amr-02",
      current_node_id: "NODE_CHG_1",
      pose: Transform2D(0.0, 5.0, 1.57),
      battery_soc: 0.15,
      is_e_stopped: True,
      active_order_id: None,
      last_heartbeat_sec: 14.1,
    )

  let snapshot =
    FleetStatusSnapshot(
      total_robots: 2,
      active_missions_count: 1,
      pending_tasks_count: 0,
      utilization_ratio: 0.50,
      telemetries: [t1, t2],
    )

  let output = dashboard.render_dashboard(snapshot, 14.5)

  assert string.contains(output, "COLLET INDUSTRIAL AMR FLEET ORCHESTRATION") == True
  assert string.contains(output, "amr-01") == True
  assert string.contains(output, "amr-02") == True
  assert string.contains(output, "E-STOPPED") == True
  assert string.contains(output, "IN_TRANSIT (ORDER_99)") == True
  assert string.contains(output, "Utilization:") == True
  assert string.contains(output, "50%") == True
}
