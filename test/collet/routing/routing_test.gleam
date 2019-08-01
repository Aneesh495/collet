import collet/geometry/point.{Point2D}
import collet/kinematics/vehicle
import collet/reservation/table
import collet/routing/cbs.{AgentTask}
import collet/routing/corridor
import collet/routing/heuristic
import collet/routing/space_time_astar.{AvoidNode}
import collet/routing/whca_star
import collet/topology/parser
import gleam/dict
import gleam/list

pub fn heuristic_precomputation_test() {
  let grid = parser.create_grid_warehouse("GRID_3x3", "Heuristic Test", 3, 3, 2.0)
  // Distance from N_0_0 to N_2_2 is 4 edges * (2.0m / 2.0m/s) = 4.0s (using v_max = 2.0)
  let h_table = heuristic.precompute_reverse_dijkstra(grid, "N_2_2", 2.0)

  let h_start = heuristic.lookup(h_table, "N_0_0", 999.0)
  assert h_start >=. 3.99 && h_start <=. 4.01

  let h_goal = heuristic.lookup(h_table, "N_2_2", 999.0)
  assert h_goal == 0.0
}

pub fn space_time_astar_test() {
  let grid = parser.create_grid_warehouse("GRID_3x3", "STA Test", 3, 3, 2.0)
  let amr = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")
  let res_tbl = table.new()

  // 1. Direct unconstrained path from N_0_0 to N_2_0 (2 hops: N_0_0 -> N_1_0 -> N_2_0)
  // At speed 1.8 m/s over 4.0 meters, duration = 4.0 / 1.8 = 2.222s
  case
    space_time_astar.search(
      grid,
      amr,
      res_tbl,
      [],
      "N_0_0",
      0.0,
      "N_2_0",
      50.0,
    )
  {
    Ok(path) -> {
      assert path.total_travel_time_sec >=. 2.21 && path.total_travel_time_sec <=. 2.23
      assert list.length(path.route) == 3
      Nil
    }
    Error(_) -> panic as "STA* failed to find direct path"
  }

  // 2. Dynamic wait behavior: constrain N_1_0 at t=1.11s.
  // The robot waits at N_0_0 for 1s until the obstacle passes, then traverses N_1_0 at t=2.11s.
  let wait_constraint = [AvoidNode(node_id: "N_1_0", time_sec: 1.11)]
  case
    space_time_astar.search(
      grid,
      amr,
      res_tbl,
      wait_constraint,
      "N_0_0",
      0.0,
      "N_2_0",
      50.0,
    )
  {
    Ok(path_delayed) -> {
      assert path_delayed.total_travel_time_sec >. 3.0
      // Ensure robot did not arrive at N_1_0 during the forbidden window around 1.11s
      let arrived_early =
        list.any(path_delayed.route, fn(r) {
          r.node_id == "N_1_0" && r.arrival_time_sec <. 1.8
        })
      assert !arrived_early
      Nil
    }
    Error(_) -> panic as "STA* failed with wait constraint"
  }

  // 3. Complete spatial detour: constrain N_1_0 across all arrival times.
  // The robot must take the spatial detour via N_0_1 -> N_1_1 -> N_2_1 -> N_2_0.
  let detour_constraints = [
    AvoidNode(node_id: "N_1_0", time_sec: 1.11),
    AvoidNode(node_id: "N_1_0", time_sec: 2.11),
    AvoidNode(node_id: "N_1_0", time_sec: 3.11),
    AvoidNode(node_id: "N_1_0", time_sec: 4.11),
  ]
  case
    space_time_astar.search(
      grid,
      amr,
      res_tbl,
      detour_constraints,
      "N_0_0",
      0.0,
      "N_2_0",
      50.0,
    )
  {
    Ok(path_detour) -> {
      assert path_detour.total_travel_time_sec >=. 4.40
      let visited_n10 =
        list.any(path_detour.route, fn(r) { r.node_id == "N_1_0" })
      assert !visited_n10
      Nil
    }
    Error(_) -> panic as "STA* failed with spatial detour"
  }
}

pub fn cbs_two_agent_crossing_test() {
  let grid = parser.create_grid_warehouse("GRID_3x3", "CBS Test", 3, 3, 2.0)
  let amr1 = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")
  let amr2 = vehicle.standard_differential_amr("AMR_2", "Hermes 1000")

  // Agent 1: moves from (0, 1) to (2, 1) horizontally
  let task1 =
    AgentTask(
      vehicle_id: "AMR_1",
      profile: amr1,
      start_node_id: "N_0_1",
      goal_node_id: "N_2_1",
      start_time_sec: 0.0,
    )

  // Agent 2: moves from (1, 0) to (1, 2) vertically, crossing Agent 1 at (1, 1)
  let task2 =
    AgentTask(
      vehicle_id: "AMR_2",
      profile: amr2,
      start_node_id: "N_1_0",
      goal_node_id: "N_1_2",
      start_time_sec: 0.0,
    )

  case cbs.solve(grid, [task1, task2], 50) {
    Ok(sol) -> {
      assert dict.size(sol.paths) == 2
      assert sol.sum_of_costs_sec >. 0.0
      Nil
    }
    Error(err) -> panic as err
  }
}

pub fn whca_fleet_test() {
  let grid = parser.create_grid_warehouse("GRID_4x4", "WHCA Test", 4, 4, 2.0)
  let amr = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")

  let tasks = [
    AgentTask("AMR_1", amr, "N_0_0", "N_3_3", 0.0),
    AgentTask("AMR_2", amr, "N_3_0", "N_0_3", 0.0),
    AgentTask("AMR_3", amr, "N_0_3", "N_3_0", 0.0),
  ]

  case whca_star.plan_fleet(grid, tasks, 30.0) {
    Ok(res) -> {
      assert dict.size(res.paths) == 3
      Nil
    }
    Error(err) -> panic as err
  }
}

pub fn corridor_synthesis_test() {
  let grid = parser.create_grid_warehouse("GRID_3x3", "Corridor Test", 3, 3, 2.0)
  let amr = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")

  case
    space_time_astar.search(
      grid,
      amr,
      table.new(),
      [],
      "N_0_0",
      0.0,
      "N_2_0",
      20.0,
    )
  {
    Ok(path) -> {
      let nav_corridor = corridor.synthesize_corridor(path.route, grid, 0.8)
      assert nav_corridor.total_length_meters >=. 3.99 && nav_corridor.total_length_meters <=. 4.01
      assert list.length(nav_corridor.segments) == 2

      // Position along middle of aisle is within corridor
      assert corridor.is_within_corridor(nav_corridor, Point2D(1.0, 0.0), 1.0)
      // Position far outside aisle is not within corridor
      assert !corridor.is_within_corridor(nav_corridor, Point2D(1.0, 5.0), 1.0)
      Nil
    }
    Error(_) -> panic as "Pathfinding failed for corridor test"
  }
}
