import collet/actor/fleet_manager.{type FleetStatusSnapshot, FleetStatusSnapshot}
import collet/actor/robot_session.{VehicleTelemetry}
import collet/api/router
import collet/geometry/transform.{Transform2D}
import collet/simulator/world.{type SimMetrics, SimMetrics}
import gleam/http.{Get, Post}
import gleam/http/request
import gleam/option.{Some}
import gleam/string

fn sample_snapshot() -> FleetStatusSnapshot {
  let t =
    VehicleTelemetry(
      vehicle_id: "amr-api",
      current_node_id: "NODE_STN_1",
      pose: Transform2D(5.0, 10.0, 0.0),
      battery_soc: 0.85,
      is_e_stopped: False,
      active_order_id: Some("ORD_77"),
      last_heartbeat_sec: 100.0,
    )

  FleetStatusSnapshot(
    total_robots: 1,
    active_missions_count: 1,
    pending_tasks_count: 0,
    utilization_ratio: 1.0,
    telemetries: [t],
  )
}

fn sample_metrics() -> SimMetrics {
  SimMetrics(
    total_distance_meters: 1540.2,
    total_energy_consumed_wh: 380.5,
    missions_completed_count: 12,
    near_miss_incidents: 0,
    collisions_count: 0,
  )
}

pub fn health_endpoint_test() {
  let req =
    request.new()
    |> request.set_method(Get)
    |> request.set_path("/health")

  let resp = router.handle_request(req, sample_snapshot(), sample_metrics())

  assert resp.status == 200
  assert string.contains(resp.body, "\"status\":\"UP\"") == True
  assert string.contains(resp.body, "\"engine\":\"collet\"") == True
}

pub fn prometheus_metrics_endpoint_test() {
  let req =
    request.new()
    |> request.set_method(Get)
    |> request.set_path("/metrics")

  let resp = router.handle_request(req, sample_snapshot(), sample_metrics())

  assert resp.status == 200
  assert string.contains(resp.body, "collet_fleet_total_robots 1") == True
  assert string.contains(resp.body, "collet_fleet_active_missions 1") == True
  assert string.contains(
      resp.body,
      "collet_distance_traveled_meters_total 1540.2",
    )
    == True
  assert string.contains(
      resp.body,
      "collet_robot_battery_soc{vehicle_id=\"amr-api\"} 0.85",
    )
    == True
}

pub fn fleet_status_and_telemetry_test() {
  let req_status =
    request.new()
    |> request.set_method(Get)
    |> request.set_path("/api/v1/status")

  let resp_status =
    router.handle_request(req_status, sample_snapshot(), sample_metrics())

  assert resp_status.status == 200
  assert string.contains(resp_status.body, "\"total_robots\":1") == True
  assert string.contains(resp_status.body, "\"utilization_ratio\":1.0") == True

  let req_fleet =
    request.new()
    |> request.set_method(Get)
    |> request.set_path("/api/v1/fleet")

  let resp_fleet =
    router.handle_request(req_fleet, sample_snapshot(), sample_metrics())

  assert resp_fleet.status == 200
  assert string.contains(resp_fleet.body, "amr-api") == True
  assert string.contains(resp_fleet.body, "NODE_STN_1") == True
}

pub fn vda5050_order_ingest_test() {
  let valid_json = "{\"orderId\": \"ORD_2026_01\", \"orderUpdateId\": 0}"
  let req_valid =
    request.new()
    |> request.set_method(Post)
    |> request.set_path("/api/v1/orders")
    |> request.set_body(valid_json)

  let resp_valid =
    router.handle_request(req_valid, sample_snapshot(), sample_metrics())

  assert resp_valid.status == 202
  assert string.contains(resp_valid.body, "\"status\":\"ACCEPTED\"") == True
  assert string.contains(resp_valid.body, "ORD_2026_01") == True

  let invalid_json = "{\"broken\": true}"
  let req_invalid =
    request.new()
    |> request.set_method(Post)
    |> request.set_path("/api/v1/orders")
    |> request.set_body(invalid_json)

  let resp_invalid =
    router.handle_request(req_invalid, sample_snapshot(), sample_metrics())

  assert resp_invalid.status == 400
  assert string.contains(resp_invalid.body, "\"status\":\"REJECTED\"") == True
}

pub fn not_found_route_test() {
  let req =
    request.new()
    |> request.set_method(Get)
    |> request.set_path("/nonexistent")

  let resp = router.handle_request(req, sample_snapshot(), sample_metrics())
  assert resp.status == 404
}
