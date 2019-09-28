import collet/energy/battery
import collet/geometry/point.{Point2D}
import collet/geometry/transform.{Transform2D}
import collet/kinematics/odometry
import collet/kinematics/trajectory
import collet/kinematics/vehicle
import collet/simulator/scenario
import collet/simulator/world.{SimulatedRobot}
import collet/topology/map
import collet/topology/node
import gleam/dict
import gleam/option.{None, Some}

import gleam/result

pub fn single_robot_trajectory_simulation_test() {
  let n1 = node.new_transit("N1", Point2D(0.0, 0.0))
  let n2 = node.new_transit("N2", Point2D(10.0, 0.0))
  let wmap =
    map.new("SIM_TEST_MAP", "Simulation Test Map")
    |> map.add_node(n1)
    |> map.add_node(n2)

  let prof =
    vehicle.standard_differential_amr("prof_diff", "Standard Differential AMR")
  let traj =
    trajectory.generate_linear(
      Point2D(0.0, 0.0),
      Point2D(10.0, 0.0),
      0.0,
      0.0,
      prof,
      0.0,
    )

  let r1 =
    SimulatedRobot(
      vehicle_id: "amr-sim",
      profile: prof,
      pose: Transform2D(0.0, 0.0, 0.0),
      linear_velocity_mps: 0.0,
      angular_velocity_radps: 0.0,
      active_trajectory: Some(traj),
      trajectory_cursor_sec: 0.0,
      battery: battery.new_lifepo4_48v_50ah(),
      payload: None,
      odometry: odometry.init(Point2D(0.0, 0.0), 0.0),
      is_docked: False,
      accumulated_distance_m: 0.0,
    )

  let sim =
    world.new(wmap, 0.1)
    |> world.add_robot(r1)

  // Step 50 ticks (5.0 seconds)
  let stepped = world.step_n(sim, 50)

  assert stepped.clock_sec >=. 4.99
  assert stepped.metrics.total_distance_meters >. 2.0
  assert stepped.metrics.total_energy_consumed_wh >. 0.0

  let assert Ok(robot_state) = dict.get(stepped.robots, "amr-sim")
  assert robot_state.accumulated_distance_m >. 2.0
  assert robot_state.pose.x >. 2.0
}

pub fn corridor_scenario_setup_test() {
  let sim = scenario.setup_corridor_deadlock_scenario()
  assert dict.size(sim.robots) == 2
  assert result.is_ok(map.get_node(sim.wmap, "N_BUF")) == True
}

pub fn intersection_crossing_simulation_test() {
  let sim = scenario.setup_intersection_crossing_scenario()

  // Run simulation for 12.0 seconds
  let finished_sim = scenario.run_until(sim, 12.0)

  assert finished_sim.clock_sec >=. 12.0
  // No collisions should occur
  assert finished_sim.metrics.collisions_count == 0
  assert finished_sim.metrics.total_distance_meters >. 10.0
}
