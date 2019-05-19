import collet/geometry/math_ffi
import collet/geometry/orientation
import collet/geometry/point.{type Point2D, Point2D}

// 2D spatial vehicle pose in the global world coordinate frame.
pub type Pose2D {
  Pose2D(position: Point2D, heading_rad: Float)
}

// 2D velocity twist in the local body coordinate frame.
pub type Twist2D {
  Twist2D(linear_x_mps: Float, linear_y_mps: Float, angular_z_radps: Float)
}

// Raw optical or magnetic wheel encoder tick delta measurement.
pub type WheelTicks {
  WheelTicks(left_ticks: Int, right_ticks: Int, dt_sec: Float)
}

// Complete estimated dead-reckoning odometry state of an operating AMR.
pub type OdometryState {
  OdometryState(
    pose: Pose2D,
    twist: Twist2D,
    total_distance_traveled_meters: Float,
    is_slip_detected: Bool,
  )
}

// Initializes an odometry tracker at a specified global starting pose.
pub fn init(position: Point2D, heading_rad: Float) -> OdometryState {
  OdometryState(
    pose: Pose2D(
      position: position,
      heading_rad: orientation.normalize(heading_rad),
    ),
    twist: Twist2D(linear_x_mps: 0.0, linear_y_mps: 0.0, angular_z_radps: 0.0),
    total_distance_traveled_meters: 0.0,
    is_slip_detected: False,
  )
}

// Propagates vehicle pose given body-frame velocity twist using 2nd-order Runge-Kutta integration.
pub fn propagate_twist(
  state: OdometryState,
  twist: Twist2D,
  dt_sec: Float,
) -> OdometryState {
  let dt = math_ffi.max_float(0.0, dt_sec)
  let d_theta = twist.angular_z_radps *. dt

  // Average heading during the integration step (Runge-Kutta midpoint)
  let heading_mid = state.pose.heading_rad +. d_theta /. 2.0
  let cos_h = math_ffi.cos(heading_mid)
  let sin_h = math_ffi.sin(heading_mid)

  // Displacements in body frame
  let dx_body = twist.linear_x_mps *. dt
  let dy_body = twist.linear_y_mps *. dt

  // Rotate body displacements to world frame
  let dx_world = cos_h *. dx_body -. sin_h *. dy_body
  let dy_world = sin_h *. dx_body +. cos_h *. dy_body

  let new_pos =
    Point2D(
      x: state.pose.position.x +. dx_world,
      y: state.pose.position.y +. dy_world,
    )
  let new_heading = orientation.normalize(state.pose.heading_rad +. d_theta)
  let step_dist = math_ffi.sqrt(dx_world *. dx_world +. dy_world *. dy_world)

  OdometryState(
    pose: Pose2D(position: new_pos, heading_rad: new_heading),
    twist: twist,
    total_distance_traveled_meters: state.total_distance_traveled_meters
      +. step_dist,
    is_slip_detected: state.is_slip_detected,
  )
}

// Computes body twist and propagates state from discrete differential-drive encoder ticks.
pub fn propagate_differential_ticks(
  state: OdometryState,
  ticks: WheelTicks,
  wheel_radius: Float,
  wheel_track: Float,
  ticks_per_rev: Int,
) -> OdometryState {
  let safe_tpr = math_ffi.max_float(1.0, int_to_float_erl(ticks_per_rev))
  let meters_per_tick = 2.0 *. math_ffi.pi() *. wheel_radius /. safe_tpr

  let d_left = int_to_float_erl(ticks.left_ticks) *. meters_per_tick
  let d_right = int_to_float_erl(ticks.right_ticks) *. meters_per_tick

  // Forward displacement and angular rotation deltas
  let d_center = { d_left +. d_right } /. 2.0
  let d_theta = { d_right -. d_left } /. wheel_track

  let safe_dt = math_ffi.max_float(0.0001, ticks.dt_sec)
  let twist =
    Twist2D(
      linear_x_mps: d_center /. safe_dt,
      linear_y_mps: 0.0,
      angular_z_radps: d_theta /. safe_dt,
    )

  propagate_twist(state, twist, ticks.dt_sec)
}

// Identifies wheel slip / traction loss by comparing commanded velocity twist against encoder odometry.
pub fn detect_wheel_slip(
  commanded: Twist2D,
  measured: Twist2D,
  linear_threshold: Float,
  angular_threshold: Float,
) -> Bool {
  let linear_err =
    math_ffi.abs_float(commanded.linear_x_mps -. measured.linear_x_mps)
  let angular_err =
    math_ffi.abs_float(commanded.angular_z_radps -. measured.angular_z_radps)

  linear_err >. linear_threshold || angular_err >. angular_threshold
}

@external(erlang, "erlang", "float")
fn int_to_float_erl(x: Int) -> Float
