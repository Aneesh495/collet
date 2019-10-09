import collet/actor/fleet_manager.{type FleetStatusSnapshot}
import collet/api/metrics
import collet/protocol/vda5050/codec
import collet/simulator/world.{type SimMetrics}
import gleam/http.{Get, Post}
import gleam/http/request.{type Request}
import gleam/http/response.{type Response}
import gleam/json
import gleam/list
import gleam/option.{None, Some}

// Routes incoming HTTP requests to their respective handlers.
pub fn handle_request(
  req: Request(String),
  snapshot: FleetStatusSnapshot,
  sim_metrics: SimMetrics,
) -> Response(String) {
  let path = req.path

  case req.method, path {
    Get, "/health" -> handle_health()
    Get, "/metrics" -> handle_metrics(snapshot, sim_metrics)
    Get, "/api/v1/status" -> handle_fleet_status(snapshot)
    Get, "/api/v1/fleet" -> handle_fleet_telemetry(snapshot)
    Post, "/api/v1/orders" -> handle_order_ingest(req.body)
    _, _ ->
      response.new(404)
      |> response.set_header("content-type", "application/json")
      |> response.set_body("{\"error\": \"Route not found\"}")
  }
}

// Health check endpoint.
fn handle_health() -> Response(String) {
  let body =
    json.object([
      #("status", json.string("UP")),
      #("engine", json.string("collet")),
      #("version", json.string("1.0.0")),
      #("protocol", json.string("VDA-5050-v2.0.0")),
    ])
    |> json.to_string

  response.new(200)
  |> response.set_header("content-type", "application/json")
  |> response.set_body(body)
}

// Prometheus metrics exposition endpoint.
fn handle_metrics(
  snapshot: FleetStatusSnapshot,
  sim_metrics: SimMetrics,
) -> Response(String) {
  let prometheus_text =
    metrics.render_prometheus_metrics(snapshot, sim_metrics)

  response.new(200)
  |> response.set_header(
    "content-type",
    "text/plain; version=0.0.4; charset=utf-8",
  )
  |> response.set_body(prometheus_text)
}

// Fleet operational status overview JSON endpoint.
fn handle_fleet_status(snapshot: FleetStatusSnapshot) -> Response(String) {
  let body =
    json.object([
      #("total_robots", json.int(snapshot.total_robots)),
      #("active_missions", json.int(snapshot.active_missions_count)),
      #("pending_tasks", json.int(snapshot.pending_tasks_count)),
      #("utilization_ratio", json.float(snapshot.utilization_ratio)),
    ])
    |> json.to_string

  response.new(200)
  |> response.set_header("content-type", "application/json")
  |> response.set_body(body)
}

// Full vehicle telemetry list endpoint.
fn handle_fleet_telemetry(snapshot: FleetStatusSnapshot) -> Response(String) {
  let robots_json =
    list.map(snapshot.telemetries, fn(t) {
      json.object([
        #("vehicle_id", json.string(t.vehicle_id)),
        #("current_node_id", json.string(t.current_node_id)),
        #("battery_soc", json.float(t.battery_soc)),
        #("is_e_stopped", json.bool(t.is_e_stopped)),
        #(
          "active_order_id",
          case t.active_order_id {
            Some(oid) -> json.string(oid)
            None -> json.null()
          },
        ),
        #(
          "pose",
          json.object([
            #("x", json.float(t.pose.x)),
            #("y", json.float(t.pose.y)),
            #("theta", json.float(t.pose.theta)),
          ]),
        ),
      ])
    })

  let body =
    json.object([#("vehicles", json.array(robots_json, fn(item) { item }))])
    |> json.to_string

  response.new(200)
  |> response.set_header("content-type", "application/json")
  |> response.set_body(body)
}

// Ingests and validates incoming VDA 5050 order JSON documents.
fn handle_order_ingest(body: String) -> Response(String) {
  case codec.decode_order_summary(body) {
    Ok(valid_order) -> {
      let resp_body =
        json.object([
          #("status", json.string("ACCEPTED")),
          #("order_id", json.string(valid_order.order_id)),
          #("order_update_id", json.int(valid_order.order_update_id)),
        ])
        |> json.to_string

      response.new(202)
      |> response.set_header("content-type", "application/json")
      |> response.set_body(resp_body)
    }
    Error(decode_err) -> {
      let resp_body =
        json.object([
          #("status", json.string("REJECTED")),
          #("error", json.string(decode_err)),
        ])
        |> json.to_string

      response.new(400)
      |> response.set_header("content-type", "application/json")
      |> response.set_body(resp_body)
    }
  }
}
