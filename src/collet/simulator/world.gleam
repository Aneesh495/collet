import collet/energy/battery.{type BatteryModel}
import collet/energy/dock.{type ChargingDock}
import collet/geometry/point.{Point2D}
import collet/geometry/polygon.{OrientedBoundingBox}
import collet/geometry/transform.{type Transform2D, Transform2D}
import collet/kinematics/odometry.{type OdometryState, Twist2D}
import collet/kinematics/trajectory.{type Trajectory}
import collet/kinematics/vehicle.{type VehicleProfile}
import collet/mission/dispatcher.{type Dispatcher}
import collet/mission/task.{type PayloadSpec}
import collet/reservation/table.{type ReservationTable, type VehicleId}
import collet/topology/map.{type WarehouseMap}
import gleam/dict.{type Dict}
import gleam/list
import gleam/option.{type Option, None, Some}

// A simulated physical mobile robot operating in the virtual facility.
pub type SimulatedRobot {
  SimulatedRobot(
    vehicle_id: VehicleId,
    profile: VehicleProfile,
    pose: Transform2D,
    linear_velocity_mps: Float,
    angular_velocity_radps: Float,
    active_trajectory: Option(Trajectory),
    trajectory_cursor_sec: Float,
    battery: BatteryModel,
    payload: Option(PayloadSpec),
    odometry: OdometryState,
    is_docked: Bool,
    accumulated_distance_m: Float,
  )
}

// Discrete simulation event emitted during tick execution.
pub type SimEvent {
  RobotMoved(vehicle_id: VehicleId, new_pose: Transform2D)
  RobotReachedGoal(vehicle_id: VehicleId, final_pose: Transform2D)
  RobotDocked(vehicle_id: VehicleId, dock_id: String)
  RobotUndocked(vehicle_id: VehicleId)
  CollisionDetected(first_vehicle: VehicleId, second_vehicle: VehicleId)
}

// Performance and throughput accounting metrics.
pub type SimMetrics {
  SimMetrics(
    total_distance_meters: Float,
    total_energy_consumed_wh: Float,
    missions_completed_count: Int,
    near_miss_incidents: Int,
    collisions_count: Int,
  )
}

// Global discrete-event and continuous-physics simulation environment.
pub type SimWorld {
  SimWorld(
    wmap: WarehouseMap,
    robots: Dict(VehicleId, SimulatedRobot),
    docks: List(ChargingDock),
    res_table: ReservationTable,
    dispatcher: Dispatcher,
    clock_sec: Float,
    delta_t_sec: Float,
    metrics: SimMetrics,
  )
}

// Initializes a new simulation environment.
pub fn new(wmap: WarehouseMap, delta_t_sec: Float) -> SimWorld {
  SimWorld(
    wmap: wmap,
    robots: dict.new(),
    docks: [],
    res_table: table.new(),
    dispatcher: dispatcher.new(),
    clock_sec: 0.0,
    delta_t_sec: delta_t_sec,
    metrics: SimMetrics(0.0, 0.0, 0, 0, 0),
  )
}

// Spawns and registers a simulated robot into the simulation world.
pub fn add_robot(world: SimWorld, robot: SimulatedRobot) -> SimWorld {
  let updated = dict.insert(world.robots, robot.vehicle_id, robot)
  SimWorld(..world, robots: updated)
}

// Assigns a kinodynamic trajectory to a robot.
pub fn set_robot_trajectory(
  world: SimWorld,
  vehicle_id: VehicleId,
  traj: Trajectory,
) -> Result(SimWorld, String) {
  case dict.get(world.robots, vehicle_id) {
    Error(Nil) -> Error("Robot " <> vehicle_id <> " not found in world")
    Ok(robot) -> {
      let updated_robot =
        SimulatedRobot(
          ..robot,
          active_trajectory: Some(traj),
          trajectory_cursor_sec: 0.0,
        )
      let updated_dict = dict.insert(world.robots, vehicle_id, updated_robot)
      Ok(SimWorld(..world, robots: updated_dict))
    }
  }
}

// Advances the simulation world forward by a single delta_t timestep.
pub fn step(world: SimWorld) -> #(SimWorld, List(SimEvent)) {
  let dt = world.delta_t_sec
  let new_clock = world.clock_sec +. dt

  // 1. Advance individual robot kinematic physics
  let #(updated_robots, movement_events, step_dist, step_energy) =
    dict.fold(
      world.robots,
      #(dict.new(), [], 0.0, 0.0),
      fn(acc, vid, robot) {
        let #(robots_map, events_acc, dist_acc, energy_acc) = acc
        let #(new_robot, robot_events, d_dist, d_energy) =
          advance_robot(robot, dt)
        #(
          dict.insert(robots_map, vid, new_robot),
          list.append(events_acc, robot_events),
          dist_acc +. d_dist,
          energy_acc +. d_energy,
        )
      },
    )

  // 2. Perform continuous collision detection across all active robot pairs
  let collision_events = check_fleet_collisions(updated_robots)
  let collision_count = list.length(collision_events)

  let updated_metrics =
    SimMetrics(
      total_distance_meters: world.metrics.total_distance_meters +. step_dist,
      total_energy_consumed_wh: world.metrics.total_energy_consumed_wh
        +. step_energy,
      missions_completed_count: world.metrics.missions_completed_count,
      near_miss_incidents: world.metrics.near_miss_incidents,
      collisions_count: world.metrics.collisions_count + collision_count,
    )

  let all_events = list.append(movement_events, collision_events)

  let updated_world =
    SimWorld(
      ..world,
      robots: updated_robots,
      clock_sec: new_clock,
      metrics: updated_metrics,
    )

  #(updated_world, all_events)
}

// Advances an individual robot's physics and state along its trajectory.
fn advance_robot(
  robot: SimulatedRobot,
  dt: Float,
) -> #(SimulatedRobot, List(SimEvent), Float, Float) {
  case robot.active_trajectory {
    None -> {
      // Robot is stationary: consume only auxiliary hotel load
      let discharged_bat =
        battery.discharge(robot.battery, dt, robot.battery.auxiliary_load_watts)
      let energy_wh = { robot.battery.auxiliary_load_watts *. dt } /. 3600.0
      let updated_robot = SimulatedRobot(..robot, battery: discharged_bat)
      #(updated_robot, [], 0.0, energy_wh)
    }
    Some(traj) -> {
      let next_cursor = robot.trajectory_cursor_sec +. dt
      case trajectory.evaluate_at_time(traj, next_cursor) {
        Error(Nil) -> {
          // Trajectory finished
          let updated_robot =
            SimulatedRobot(
              ..robot,
              active_trajectory: None,
              linear_velocity_mps: 0.0,
              angular_velocity_radps: 0.0,
            )
          #(
            updated_robot,
            [RobotReachedGoal(robot.vehicle_id, robot.pose)],
            0.0,
            0.0,
          )
        }
        Ok(knot) -> {
          let new_pose =
            Transform2D(knot.position.x, knot.position.y, knot.heading_rad)
          let current_pos = Point2D(robot.pose.x, robot.pose.y)
          let next_pos = knot.position
          let dist_step = point.distance(current_pos, next_pos)

          // Payload mass
          let payload_kg = case robot.payload {
            Some(p) -> p.weight_kg
            None -> 0.0
          }

          // Compute motion power
          let power_w =
            battery.compute_motion_power_watts(
              robot.battery,
              robot.profile.curb_weight_kg,
              payload_kg,
              knot.linear_velocity_mps,
              0.0,
            )

          let discharged_bat = battery.discharge(robot.battery, dt, power_w)
          let energy_wh = { power_w *. dt } /. 3600.0

          // Propagate dead reckoning odometry
          let twist =
            Twist2D(
              linear_x_mps: knot.linear_velocity_mps,
              linear_y_mps: 0.0,
              angular_z_radps: knot.angular_velocity_radps,
            )
          let odom = odometry.propagate_twist(robot.odometry, twist, dt)

          // Check if end of trajectory reached
          let #(final_traj, events) = case
            next_cursor >=. traj.total_duration_sec
          {
            True -> #(
              None,
              [
                RobotMoved(robot.vehicle_id, new_pose),
                RobotReachedGoal(robot.vehicle_id, new_pose),
              ],
            )
            False -> #(Some(traj), [RobotMoved(robot.vehicle_id, new_pose)])
          }

          let updated_robot =
            SimulatedRobot(
              ..robot,
              pose: new_pose,
              linear_velocity_mps: knot.linear_velocity_mps,
              angular_velocity_radps: knot.angular_velocity_radps,
              active_trajectory: final_traj,
              trajectory_cursor_sec: next_cursor,
              battery: discharged_bat,
              odometry: odom,
              accumulated_distance_m: robot.accumulated_distance_m +. dist_step,
            )

          #(updated_robot, events, dist_step, energy_wh)
        }
      }
    }
  }
}

// Checks for mutual physical collisions across all active pairs of simulated robots.
fn check_fleet_collisions(
  robots: Dict(VehicleId, SimulatedRobot),
) -> List(SimEvent) {
  let robot_list = dict.values(robots)
  case robot_list {
    [] | [_] -> []
    list_items -> check_pairs(list_items, [])
  }
}

// Evaluates pairwise polygon footprint intersection.
fn check_pairs(
  remaining: List(SimulatedRobot),
  acc: List(SimEvent),
) -> List(SimEvent) {
  case remaining {
    [] | [_] -> acc
    [head, ..tail] -> {
      let head_events =
        list.filter_map(tail, fn(other) {
          let obb1 =
            OrientedBoundingBox(
              center: Point2D(head.pose.x, head.pose.y),
              heading: head.pose.theta,
              length: head.profile.length_meters,
              width: head.profile.width_meters,
            )
          let obb2 =
            OrientedBoundingBox(
              center: Point2D(other.pose.x, other.pose.y),
              heading: other.pose.theta,
              length: other.profile.length_meters,
              width: other.profile.width_meters,
            )
          let poly1 = polygon.obb_to_polygon(obb1)
          let poly2 = polygon.obb_to_polygon(obb2)
          case polygon.polygons_intersect_sat(poly1, poly2) {
            True ->
              Ok(
                CollisionDetected(
                  first_vehicle: head.vehicle_id,
                  second_vehicle: other.vehicle_id,
                ),
              )
            False -> Error(Nil)
          }
        })
      check_pairs(tail, list.append(acc, head_events))
    }
  }
}

// Executes N consecutive simulation ticks.
pub fn step_n(world: SimWorld, steps: Int) -> SimWorld {
  case steps <= 0 {
    True -> world
    False -> {
      let #(next_world, _) = step(world)
      step_n(next_world, steps - 1)
    }
  }
}
