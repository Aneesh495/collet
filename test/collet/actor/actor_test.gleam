import collet/actor/fleet_manager
import collet/actor/robot_session
import collet/actor/supervisor
import collet/energy/battery
import collet/geometry/point.{Point2D}
import collet/geometry/transform.{Transform2D}
import collet/mission/task.{Standard}
import collet/topology/edge
import collet/topology/map
import collet/topology/node
import gleam/erlang/process

pub fn robot_session_lifecycle_and_telemetry_test() {
  let bat = battery.new_lifepo4_48v_50ah()
  let assert Ok(session) = robot_session.start("amr-01", "NODE_INIT", bat)
  let session_subject = session.data

  // 1. Initial telemetry query
  let telem1 = robot_session.get_telemetry(session_subject, 1000)
  assert telem1.vehicle_id == "amr-01"
  assert telem1.current_node_id == "NODE_INIT"
  assert telem1.battery_soc == 1.0
  assert telem1.is_e_stopped == False

  // 2. Heartbeat update
  robot_session.ping_heartbeat(session_subject, 42.5)

  // 3. Movement update
  process.send(
    session_subject,
    robot_session.UpdatePosition("NODE_TARGET", Transform2D(10.0, 5.0, 1.57)),
  )

  let telem2 = robot_session.get_telemetry(session_subject, 1000)
  assert telem2.current_node_id == "NODE_TARGET"
  assert telem2.last_heartbeat_sec == 42.5
  assert telem2.pose.x == 10.0
  assert telem2.pose.y == 5.0

  // 4. Trigger E-Stop
  robot_session.trigger_emergency_stop(
    session_subject,
    "Obstacle in safety zone",
    1000,
  )
  let telem3 = robot_session.get_telemetry(session_subject, 1000)
  assert telem3.is_e_stopped == True

  // Clean shutdown
  process.send(session_subject, robot_session.Stop)
}

pub fn fleet_manager_actor_coordination_test() {
  let n1 = node.new_transit("N1", Point2D(0.0, 0.0))
  let n2 = node.new_transit("N2", Point2D(10.0, 0.0))
  let e1_2 = edge.new_linear("E1_2", "N1", "N2", 10.0, 2.0)

  let wmap =
    map.new("FLEET_MAP", "Fleet Facility")
    |> map.add_node(n1)
    |> map.add_node(n2)
    |> map.add_edge(e1_2)

  let assert Ok(mgr) = fleet_manager.start(wmap)
  let mgr_subject = mgr.data

  let bat = battery.new_lifepo4_48v_50ah()
  let assert Ok(session) = robot_session.start("amr-coord", "N1", bat)

  // Register robot
  fleet_manager.register_robot(mgr_subject, "amr-coord", session.data, 1000)

  // Send initial telemetry to manager
  let initial_telem = robot_session.get_telemetry(session.data, 1000)
  process.send(mgr_subject, fleet_manager.UpdateRobotTelemetry(initial_telem))

  // Submit mission
  let t =
    task.new_transport(
      "TASK_COORD",
      "N1",
      "N2",
      "PALLET",
      200.0,
      Standard,
      300.0,
      0.0,
    )
  let tid = fleet_manager.submit_task(mgr_subject, t, 1000)
  assert tid == "TASK_COORD"

  // Check initial fleet status snapshot
  let status1 = fleet_manager.get_fleet_status(mgr_subject, 1000)
  assert status1.total_robots == 1
  assert status1.pending_tasks_count == 1
  assert status1.active_missions_count == 0

  // Run dispatch tick
  let assigned_count = fleet_manager.tick_dispatch(mgr_subject, 1000)
  assert assigned_count == 1

  // Check updated fleet status
  let status2 = fleet_manager.get_fleet_status(mgr_subject, 1000)
  assert status2.pending_tasks_count == 0
  assert status2.active_missions_count == 1
  assert status2.utilization_ratio == 1.0

  // Clean shutdown
  process.send(mgr_subject, fleet_manager.Stop)
  process.send(session.data, robot_session.Stop)
}

pub fn supervisor_tree_startup_test() {
  let wmap = map.new("SUP_MAP", "Supervisor Facility")
  let assert Ok(sup) = supervisor.start_fleet_system(wmap)
  assert process.is_alive(sup.pid) == True
}
