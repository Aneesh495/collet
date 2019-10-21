import collet/actor/fleet_manager.{type FleetStatusSnapshot}
import collet/simulator/world.{type SimMetrics}
import gleam/float
import gleam/int
import gleam/list
import gleam/string

// Renders complete fleet performance telemetry into Prometheus exposition format (version 0.0.4).
pub fn render_prometheus_metrics(
  snapshot: FleetStatusSnapshot,
  metrics: SimMetrics,
) -> String {
  let lines = [
    "# HELP collet_fleet_total_robots Total registered autonomous mobile robots",
    "# TYPE collet_fleet_total_robots gauge",
    "collet_fleet_total_robots " <> int.to_string(snapshot.total_robots),
    "",
    "# HELP collet_fleet_active_missions Current count of actively executing transport orders",
    "# TYPE collet_fleet_active_missions gauge",
    "collet_fleet_active_missions "
      <> int.to_string(snapshot.active_missions_count),
    "",
    "# HELP collet_fleet_pending_tasks Unassigned pending transport tasks in dispatcher queue",
    "# TYPE collet_fleet_pending_tasks gauge",
    "collet_fleet_pending_tasks " <> int.to_string(snapshot.pending_tasks_count),
    "",
    "# HELP collet_fleet_utilization_ratio Active vehicle allocation ratio across the fleet",
    "# TYPE collet_fleet_utilization_ratio gauge",
    "collet_fleet_utilization_ratio "
      <> float.to_string(snapshot.utilization_ratio),
    "",
    "# HELP collet_distance_traveled_meters_total Cumulative distance traversed by all fleet AMRs",
    "# TYPE collet_distance_traveled_meters_total counter",
    "collet_distance_traveled_meters_total "
      <> float.to_string(metrics.total_distance_meters),
    "",
    "# HELP collet_energy_consumed_watthours_total Cumulative electrical energy consumed across all AMRs",
    "# TYPE collet_energy_consumed_watthours_total counter",
    "collet_energy_consumed_watthours_total "
      <> float.to_string(metrics.total_energy_consumed_wh),
    "",
    "# HELP collet_missions_completed_total Total successfully fulfilled material transport missions",
    "# TYPE collet_missions_completed_total counter",
    "collet_missions_completed_total "
      <> int.to_string(metrics.missions_completed_count),
    "",
    "# HELP collet_collisions_total Total physical collision incidents recorded by the safety supervisor",
    "# TYPE collet_collisions_total counter",
    "collet_collisions_total " <> int.to_string(metrics.collisions_count),
    "",
  ]

  // Per-vehicle battery SoC gauges
  let per_robot_lines =
    list.flat_map(snapshot.telemetries, fn(t) {
      [
        "collet_robot_battery_soc{vehicle_id=\""
        <> t.vehicle_id
        <> "\"} "
        <> float.to_string(t.battery_soc),
      ]
    })

  string.join(list.append(lines, per_robot_lines), "\n") <> "\n"
}
