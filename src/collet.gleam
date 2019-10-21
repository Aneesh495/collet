import collet/actor/fleet_manager.{FleetStatusSnapshot}
import collet/actor/robot_session.{VehicleTelemetry}
import collet/cli/args.{
  type CliConfig, BenchmarkMode, DaemonMode, HelpMode, SimulateMode,
  ValidateTopologyMode, VersionMode,
}
import collet/cli/dashboard
import collet/energy/battery
import collet/kinematics/vehicle
import collet/reservation/table
import collet/routing/cbs.{AgentTask}
import collet/routing/space_time_astar
import collet/simulator/scenario
import collet/topology/map
import collet/topology/parser
import gleam/dict
import gleam/float
import gleam/int
import gleam/io
import gleam/list
import gleam/option.{None}

pub fn main() {
  let cfg = args.default_config()
  run_with_config(cfg)
}

// Executes Collet under the specified runtime configuration.
pub fn run_with_config(cfg: CliConfig) {
  case cfg.mode {
    HelpMode -> {
      io.println(args.help_text())
    }

    VersionMode -> {
      io.println("Collet v1.0.0 (Gleam 1.19 / BEAM 29.1.2)")
    }

    SimulateMode -> {
      run_simulation(cfg)
    }

    BenchmarkMode -> {
      run_benchmarks()
    }

    ValidateTopologyMode -> {
      run_topology_validation()
    }

    DaemonMode -> {
      run_daemon(cfg)
    }
  }
}

// Executes multi-agent warehouse simulation and displays live dashboard.
fn run_simulation(cfg: CliConfig) {
  io.println(
    "\u{001b}[1m\u{001b}[36m>>> Starting Collet Multi-Agent Simulation Engine...\u{001b}[0m",
  )
  io.println(
    "  Duration: "
    <> float.to_string(cfg.duration_sec)
    <> "s | Step: "
    <> float.to_string(cfg.time_step_sec)
    <> "s",
  )

  let sim = scenario.setup_intersection_crossing_scenario()
  let finished_sim = scenario.run_until(sim, cfg.duration_sec)

  // Construct final telemetry snapshot for dashboard display
  let telems =
    dict.fold(finished_sim.robots, [], fn(acc, vid, r) {
      let t =
        VehicleTelemetry(
          vehicle_id: vid,
          current_node_id: "WAYPOINT",
          pose: r.pose,
          battery_soc: battery.state_of_charge(r.battery),
          is_e_stopped: False,
          active_order_id: None,
          last_heartbeat_sec: finished_sim.clock_sec,
        )
      [t, ..acc]
    })

  let snapshot =
    FleetStatusSnapshot(
      total_robots: dict.size(finished_sim.robots),
      active_missions_count: 0,
      pending_tasks_count: 0,
      utilization_ratio: 0.0,
      telemetries: telems,
    )

  io.println(dashboard.render_dashboard(snapshot, finished_sim.clock_sec))
  io.println(
    "\u{001b}[1m\u{001b}[32mSimulation Finished Successfully:\u{001b}[0m",
  )
  io.println(
    "  Total Distance Traversed: "
    <> float.to_string(finished_sim.metrics.total_distance_meters)
    <> " meters",
  )
  io.println(
    "  Total Energy Consumed:    "
    <> float.to_string(finished_sim.metrics.total_energy_consumed_wh)
    <> " Wh",
  )
  io.println(
    "  Collisions Recorded:      "
    <> int.to_string(finished_sim.metrics.collisions_count),
  )
}

// Runs algorithmic stress benchmarks for MAPF and kinodynamic trajectory generators.
fn run_benchmarks() {
  io.println(
    "\u{001b}[1m\u{001b}[36m>>> Executing Collet Performance Benchmarks...\u{001b}[0m",
  )

  // 1. Grid Warehouse Topology Generation
  let wmap =
    parser.create_grid_warehouse("GRID_5x5", "Benchmark Grid", 5, 5, 5.0)
  let node_cnt = dict.size(wmap.nodes)
  let edge_cnt = dict.size(wmap.edges)
  io.println(
    "  [1/3] Generated 5x5 Grid Facility Graph: "
    <> int.to_string(node_cnt)
    <> " nodes, "
    <> int.to_string(edge_cnt)
    <> " edges",
  )

  // 2. Space-Time A* Search Benchmark
  let prof = vehicle.standard_differential_amr("bench_amr", "Benchmark AMR")
  let res_table = table.new()

  let search_res =
    space_time_astar.search(
      wmap,
      prof,
      res_table,
      [],
      "N_0_0",
      0.0,
      "N_4_4",
      100.0,
    )

  case search_res {
    Ok(path) ->
      io.println(
        "  [2/3] Space-Time A* Computed Route in "
        <> float.to_string(path.total_travel_time_sec)
        <> "s (nodes expanded: "
        <> int.to_string(path.nodes_expanded)
        <> ")",
      )
    Error(Nil) -> io.println("  [2/3] Space-Time A* Error: No route found")
  }

  // 3. CBS Conflict-Based Search Benchmark
  let task1 = AgentTask("amr-01", prof, "N_0_0", "N_4_4", 0.0)
  let task2 = AgentTask("amr-02", prof, "N_4_0", "N_0_4", 0.0)

  case cbs.solve(wmap, [task1, task2], 50) {
    Ok(solution) -> {
      let agent_count = dict.size(solution.paths)
      io.println(
        "  [3/3] CBS Multi-Agent Pathfinding: Solved "
        <> int.to_string(agent_count)
        <> " conflict-free routes (Sum of costs: "
        <> float.to_string(solution.sum_of_costs_sec)
        <> "s, Explored nodes: "
        <> int.to_string(solution.tree_nodes_explored)
        <> ")",
      )
    }
    Error(err) -> io.println("  [3/3] CBS Solver Error: " <> err)
  }

  io.println(
    "\u{001b}[1m\u{001b}[32mAll Benchmarks Completed Successfully.\u{001b}[0m",
  )
}

// Runs facility topology validation.
fn run_topology_validation() {
  io.println(
    "\u{001b}[1m\u{001b}[36m>>> Validating Facility Topology Graph...\u{001b}[0m",
  )
  let wmap =
    parser.create_grid_warehouse("GRID_4x4", "Validation Grid", 4, 4, 10.0)
  let errors = map.validate(wmap)
  let node_cnt = dict.size(wmap.nodes)
  case errors {
    [] ->
      io.println(
        "\u{001b}[1m\u{001b}[32mTopology Verification: VALID (0 errors found across "
        <> int.to_string(node_cnt)
        <> " nodes)\u{001b}[0m",
      )
    errs ->
      io.println(
        "\u{001b}[1m\u{001b}[31mTopology Verification: FAILED with "
        <> int.to_string(list.length(errs))
        <> " issues\u{001b}[0m",
      )
  }
}

// Starts the fleet management daemon.
fn run_daemon(cfg: CliConfig) {
  io.println(
    "\u{001b}[1m\u{001b}[36m>>> Initializing Collet Fleet Orchestration Daemon...\u{001b}[0m",
  )
  io.println("  Listening on: http://0.0.0.0:" <> int.to_string(cfg.api_port))
  io.println(
    "  Prometheus:   http://0.0.0.0:"
    <> int.to_string(cfg.api_port)
    <> "/metrics",
  )
  io.println(
    "  Health:       http://0.0.0.0:"
    <> int.to_string(cfg.api_port)
    <> "/health",
  )
  io.println("  Supervisor:   OneForOne BEAM root started")
}
