import collet/geometry/orientation
import collet/geometry/point.{type Point2D}
import collet/geometry/polygon.{type OrientedBoundingBox, OrientedBoundingBox}
import collet/geometry/spline.{type BezierCubic}
import collet/kinematics/profile
import collet/kinematics/vehicle.{type VehicleProfile}
import gleam/list

// Discrete spatio-temporal state sample along an executing trajectory.
// Mirrors the VDA 5050 trajectory knot data structure.
pub type TrajectoryKnot {
  TrajectoryKnot(
    time_sec: Float,
    position: Point2D,
    heading_rad: Float,
    linear_velocity_mps: Float,
    angular_velocity_radps: Float,
  )
}

// Complete, continuous parameterized spatio-temporal trajectory over a time horizon.
pub type Trajectory {
  Trajectory(
    knots: List(TrajectoryKnot),
    start_time_sec: Float,
    total_duration_sec: Float,
    total_length_meters: Float,
  )
}

// Generates a dynamically feasible linear trajectory between two spatial waypoints.
pub fn generate_linear(
  start_pos: Point2D,
  end_pos: Point2D,
  start_heading: Float,
  end_heading: Float,
  veh_profile: VehicleProfile,
  start_time_sec: Float,
) -> Trajectory {
  let dist = point.distance(start_pos, end_pos)
  let prof =
    profile.plan_trapezoidal(
      dist,
      0.0,
      veh_profile.max_linear_velocity,
      0.0,
      veh_profile.max_linear_acceleration,
      veh_profile.max_linear_deceleration,
    )

  // Generate 20 knots along the trajectory profile
  let knot_count = 20
  let step = prof.total_duration_sec /. 19.0
  let indices = int_range(0, knot_count - 1)

  let knots =
    list.map(indices, fn(i) {
      let dt = int_to_float_erl(i) *. step
      let sample =
        profile.sample(
          prof,
          dt,
          0.0,
          0.0,
          veh_profile.max_linear_acceleration,
          veh_profile.max_linear_deceleration,
        )
      let s_ratio = case dist <. 0.001 {
        True -> 0.0
        False -> sample.distance_meters /. dist
      }
      let pos = point.lerp(start_pos, end_pos, s_ratio)
      let heading = orientation.slerp(start_heading, end_heading, s_ratio)

      TrajectoryKnot(
        time_sec: start_time_sec +. dt,
        position: pos,
        heading_rad: heading,
        linear_velocity_mps: sample.velocity_mps,
        angular_velocity_radps: 0.0,
      )
    })

  Trajectory(
    knots: knots,
    start_time_sec: start_time_sec,
    total_duration_sec: prof.total_duration_sec,
    total_length_meters: dist,
  )
}

// Generates a smooth trajectory along a cubic Bezier spline.
pub fn generate_spline(
  curve: BezierCubic,
  veh_profile: VehicleProfile,
  start_time_sec: Float,
) -> Trajectory {
  let arc_len = spline.arc_length(curve, 20)
  let prof =
    profile.plan_trapezoidal(
      arc_len,
      0.0,
      veh_profile.max_linear_velocity,
      0.0,
      veh_profile.max_linear_acceleration,
      veh_profile.max_linear_deceleration,
    )

  let knot_count = 20
  let step = prof.total_duration_sec /. 19.0
  let indices = int_range(0, knot_count - 1)

  let knots =
    list.map(indices, fn(i) {
      let dt = int_to_float_erl(i) *. step
      let sample =
        profile.sample(
          prof,
          dt,
          0.0,
          0.0,
          veh_profile.max_linear_acceleration,
          veh_profile.max_linear_deceleration,
        )
      let s_ratio = case arc_len <. 0.001 {
        True -> 0.0
        False -> sample.distance_meters /. arc_len
      }
      let pos = spline.evaluate(curve, s_ratio)
      let heading = spline.tangent_heading(curve, s_ratio)
      let k = spline.curvature(curve, s_ratio)
      let omega = sample.velocity_mps *. k

      TrajectoryKnot(
        time_sec: start_time_sec +. dt,
        position: pos,
        heading_rad: heading,
        linear_velocity_mps: sample.velocity_mps,
        angular_velocity_radps: omega,
      )
    })

  Trajectory(
    knots: knots,
    start_time_sec: start_time_sec,
    total_duration_sec: prof.total_duration_sec,
    total_length_meters: arc_len,
  )
}

// Evaluates the continuous trajectory at an arbitrary time query using linear interpolation.
pub fn evaluate_at_time(
  traj: Trajectory,
  query_time: Float,
) -> Result(TrajectoryKnot, Nil) {
  case traj.knots {
    [] -> Error(Nil)
    [first, ..rest] -> {
      case query_time <=. first.time_sec {
        True -> Ok(first)
        False -> find_and_interpolate_knots(first, rest, query_time)
      }
    }
  }
}

// Discretizes trajectory into time-stamped Oriented Bounding Boxes (OBBs) for collision checking.
pub fn to_oriented_bounding_boxes(
  traj: Trajectory,
  width: Float,
  length: Float,
) -> List(#(Float, OrientedBoundingBox)) {
  list.map(traj.knots, fn(k) {
    let obb =
      OrientedBoundingBox(
        center: k.position,
        width: width,
        length: length,
        heading: k.heading_rad,
      )
    #(k.time_sec, obb)
  })
}

// Recursively walks knots to find the bounding bracket [k_prev, k_next] and interpolates.
fn find_and_interpolate_knots(
  prev_knot: TrajectoryKnot,
  remaining: List(TrajectoryKnot),
  query_time: Float,
) -> Result(TrajectoryKnot, Nil) {
  case remaining {
    [] -> Ok(prev_knot)
    [curr, ..rest] -> {
      case query_time <=. curr.time_sec {
        True -> {
          let dt_segment = curr.time_sec -. prev_knot.time_sec
          let alpha = case dt_segment <. 0.0001 {
            True -> 0.0
            False -> { query_time -. prev_knot.time_sec } /. dt_segment
          }
          let interp_pos = point.lerp(prev_knot.position, curr.position, alpha)
          let interp_heading =
            orientation.slerp(prev_knot.heading_rad, curr.heading_rad, alpha)
          let interp_v =
            prev_knot.linear_velocity_mps
            +. { curr.linear_velocity_mps -. prev_knot.linear_velocity_mps }
            *. alpha
          let interp_omega =
            prev_knot.angular_velocity_radps
            +. {
              curr.angular_velocity_radps -. prev_knot.angular_velocity_radps
            }
            *. alpha

          Ok(TrajectoryKnot(
            time_sec: query_time,
            position: interp_pos,
            heading_rad: interp_heading,
            linear_velocity_mps: interp_v,
            angular_velocity_radps: interp_omega,
          ))
        }
        False -> find_and_interpolate_knots(curr, rest, query_time)
      }
    }
  }
}

// Generates an inclusive integer range [start, stop].
fn int_range(start: Int, stop: Int) -> List(Int) {
  case start > stop {
    True -> []
    False -> [start, ..int_range(start + 1, stop)]
  }
}

@external(erlang, "erlang", "float")
fn int_to_float_erl(x: Int) -> Float
