import collet/energy/battery
import collet/geometry/point.{Point2D}
import collet/geometry/transform.{Transform2D}
import collet/kinematics/odometry
import collet/kinematics/trajectory
import collet/kinematics/vehicle
import collet/simulator/world.{type SimWorld, SimulatedRobot}
import collet/topology/edge
import collet/topology/map
import collet/topology/node
import gleam/option.{None}

// Prepares a 2-agent head-on corridor scenario with a side buffer bay.
pub fn setup_corridor_deadlock_scenario() -> SimWorld {
  let n1 = node.new_transit("N1", Point2D(0.0, 0.0))
  let n2 = node.new_transit("N2", Point2D(20.0, 0.0))
  let n_buf = node.new_park("N_BUF", Point2D(10.0, 5.0), "BAY_1")

  let wmap =
    map.new("MAP_CORRIDOR", "Corridor Deadlock Test")
    |> map.add_node(n1)
    |> map.add_node(n2)
    |> map.add_node(n_buf)
    |> map.add_edge(edge.new_linear("E1_2", "N1", "N2", 20.0, 1.5))
    |> map.add_edge(edge.new_linear("E2_1", "N2", "N1", 20.0, 1.5))
    |> map.add_edge(edge.new_linear("E1_BUF", "N1", "N_BUF", 11.2, 1.0))

  let prof =
    vehicle.standard_differential_amr("prof_diff", "Standard Differential AMR")

  let robot_low =
    SimulatedRobot(
      vehicle_id: "amr-low",
      profile: prof,
      pose: Transform2D(0.0, 0.0, 0.0),
      linear_velocity_mps: 0.0,
      angular_velocity_radps: 0.0,
      active_trajectory: None,
      trajectory_cursor_sec: 0.0,
      battery: battery.new_lifepo4_48v_50ah(),
      payload: None,
      odometry: odometry.init(Point2D(0.0, 0.0), 0.0),
      is_docked: False,
      accumulated_distance_m: 0.0,
    )

  let robot_high =
    SimulatedRobot(
      vehicle_id: "amr-high",
      profile: prof,
      pose: Transform2D(20.0, 0.0, 3.14159),
      linear_velocity_mps: 0.0,
      angular_velocity_radps: 0.0,
      active_trajectory: None,
      trajectory_cursor_sec: 0.0,
      battery: battery.new_lifepo4_48v_50ah(),
      payload: None,
      odometry: odometry.init(Point2D(20.0, 0.0), 3.14159),
      is_docked: False,
      accumulated_distance_m: 0.0,
    )

  world.new(wmap, 0.1)
  |> world.add_robot(robot_low)
  |> world.add_robot(robot_high)
}

// Prepares a 2-agent orthogonal intersection crossing scenario.
pub fn setup_intersection_crossing_scenario() -> SimWorld {
  // Horizontal path from W (0, 10) to E (20, 10)
  // Vertical path from S (10, 0) to N (10, 20)
  // Intersection at C (10, 10)
  let n_w = node.new_transit("N_W", Point2D(0.0, 10.0))
  let n_e = node.new_transit("N_E", Point2D(20.0, 10.0))
  let n_s = node.new_transit("N_S", Point2D(10.0, 0.0))
  let n_n = node.new_transit("N_N", Point2D(10.0, 20.0))
  let n_c = node.new_transit("N_C", Point2D(10.0, 10.0))

  let wmap =
    map.new("MAP_INTERSECTION", "Orthogonal Crossing")
    |> map.add_node(n_w)
    |> map.add_node(n_e)
    |> map.add_node(n_s)
    |> map.add_node(n_n)
    |> map.add_node(n_c)
    |> map.add_edge(edge.new_linear("E_WC", "N_W", "N_C", 10.0, 1.5))
    |> map.add_edge(edge.new_linear("E_CE", "N_C", "N_E", 10.0, 1.5))
    |> map.add_edge(edge.new_linear("E_SC", "N_S", "N_C", 10.0, 1.5))
    |> map.add_edge(edge.new_linear("E_CN", "N_C", "N_N", 10.0, 1.5))

  let prof =
    vehicle.standard_differential_amr("prof_diff", "Standard Differential AMR")

  // Generate trajectory for Horizontal AMR: (0, 10) -> (20, 10)
  let traj_horiz =
    trajectory.generate_linear(
      Point2D(0.0, 10.0),
      Point2D(20.0, 10.0),
      0.0,
      0.0,
      prof,
      0.0,
    )

  // Generate trajectory for Vertical AMR: (10, 0) -> (10, 20), delayed start to avoid collision
  let traj_vert =
    trajectory.generate_linear(
      Point2D(10.0, 0.0),
      Point2D(10.0, 20.0),
      1.5708,
      1.5708,
      prof,
      5.0,
    )

  let r_horiz =
    SimulatedRobot(
      vehicle_id: "amr-horiz",
      profile: prof,
      pose: Transform2D(0.0, 10.0, 0.0),
      linear_velocity_mps: 0.0,
      angular_velocity_radps: 0.0,
      active_trajectory: option.Some(traj_horiz),
      trajectory_cursor_sec: 0.0,
      battery: battery.new_lifepo4_48v_50ah(),
      payload: None,
      odometry: odometry.init(Point2D(0.0, 10.0), 0.0),
      is_docked: False,
      accumulated_distance_m: 0.0,
    )

  let r_vert =
    SimulatedRobot(
      vehicle_id: "amr-vert",
      profile: prof,
      pose: Transform2D(10.0, 0.0, 1.5708),
      linear_velocity_mps: 0.0,
      angular_velocity_radps: 0.0,
      active_trajectory: option.Some(traj_vert),
      trajectory_cursor_sec: 0.0,
      battery: battery.new_lifepo4_48v_50ah(),
      payload: None,
      odometry: odometry.init(Point2D(10.0, 0.0), 1.5708),
      is_docked: False,
      accumulated_distance_m: 0.0,
    )

  world.new(wmap, 0.1)
  |> world.add_robot(r_horiz)
  |> world.add_robot(r_vert)
}

// Runs the simulation until clock reaches target duration.
pub fn run_until(sim: SimWorld, target_clock_sec: Float) -> SimWorld {
  case sim.clock_sec >=. target_clock_sec {
    True -> sim
    False -> {
      let #(next_sim, _) = world.step(sim)
      run_until(next_sim, target_clock_sec)
    }
  }
}
