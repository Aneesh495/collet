import collet/geometry/point.{Point2D}
import collet/geometry/spline
import collet/kinematics/odometry.{Twist2D, WheelTicks}
import collet/kinematics/profile
import collet/kinematics/trajectory
import collet/kinematics/vehicle
import gleam/list

pub fn vehicle_profile_and_stopping_distance_test() {
  let amr = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")
  assert amr.max_payload_kg == 1000.0
  assert vehicle.can_turn_in_place(amr)

  // Stopping distance at 1.8 m/s with 1.2 m/s^2 deceleration: v^2 / (2 * a) = 3.24 / 2.4 = 1.35m
  let d_stop = vehicle.stopping_distance(amr, 1.8, False)
  assert d_stop >=. 1.34 && d_stop <=. 1.36

  // Emergency stopping distance at 1.8 m/s with 3.5 m/s^2 deceleration: 3.24 / 7.0 = 0.463m
  let d_emer = vehicle.stopping_distance(amr, 1.8, True)
  assert d_emer >=. 0.46 && d_emer <=. 0.47

  // Payload derating: at full payload, deceleration is derated to 60%
  let decel_full = vehicle.payload_derated_deceleration(amr, 1000.0)
  assert decel_full >=. 0.71 && decel_full <=. 0.73
}

pub fn trapezoidal_and_triangular_profiles_test() {
  // Long distance: 10.0 meters with max speed 2.0, accel 1.0, decel 1.0
  // Accel: v: 0 -> 2.0 takes 2.0s, dist = 2.0m
  // Decel: v: 2.0 -> 0 takes 2.0s, dist = 2.0m
  // Cruise: 10.0 - 4.0 = 6.0m takes 3.0s
  // Total time = 7.0s
  let prof_long = profile.plan_trapezoidal(10.0, 0.0, 2.0, 0.0, 1.0, 1.0)
  assert prof_long.total_duration_sec == 7.0
  assert prof_long.duration_cruise_sec == 3.0
  assert prof_long.peak_velocity_mps == 2.0

  let sample_mid = profile.sample(prof_long, 3.5, 0.0, 0.0, 1.0, 1.0)
  assert sample_mid.velocity_mps == 2.0
  assert sample_mid.acceleration_mps2 == 0.0

  // Short distance: 1.0 meter (triangular profile)
  let prof_short = profile.plan_trapezoidal(1.0, 0.0, 2.0, 0.0, 1.0, 1.0)
  assert prof_short.duration_cruise_sec == 0.0
  assert prof_short.peak_velocity_mps == 1.0
  assert prof_short.total_duration_sec == 2.0
}

pub fn trajectory_generation_and_sampling_test() {
  let amr = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")
  let start_pt = Point2D(0.0, 0.0)
  let end_pt = Point2D(10.0, 0.0)

  let traj = trajectory.generate_linear(start_pt, end_pt, 0.0, 0.0, amr, 0.0)
  assert traj.total_length_meters == 10.0
  assert list.length(traj.knots) == 20

  case trajectory.evaluate_at_time(traj, 0.0) {
    Ok(start_knot) -> {
      assert point.is_approximately_equal(start_knot.position, start_pt, 0.001)
      Nil
    }
    Error(_) -> panic as "Evaluation at t=0 failed"
  }

  // Spline trajectory
  let curve =
    spline.new_bezier(
      Point2D(0.0, 0.0),
      Point2D(0.0, 5.0),
      Point2D(5.0, 10.0),
      Point2D(10.0, 10.0),
    )
  let spline_traj = trajectory.generate_spline(curve, amr, 0.0)
  assert spline_traj.total_length_meters >. 10.0
}

pub fn odometry_dead_reckoning_and_slip_test() {
  let init_state = odometry.init(Point2D(0.0, 0.0), 0.0)

  // 1. Forward motion: 1.0 m/s for 2.0s
  let forward_twist = Twist2D(1.0, 0.0, 0.0)
  let state_fwd = odometry.propagate_twist(init_state, forward_twist, 2.0)
  assert point.is_approximately_equal(
    state_fwd.pose.position,
    Point2D(2.0, 0.0),
    0.001,
  )
  assert state_fwd.pose.heading_rad == 0.0
  assert state_fwd.total_distance_traveled_meters == 2.0

  // 2. Pure rotation in place: 90 deg/s for 1.0s
  let rot_twist = Twist2D(0.0, 0.0, 1.570796)
  let state_rot = odometry.propagate_twist(state_fwd, rot_twist, 1.0)
  assert state_rot.pose.heading_rad >=. 1.57
    && state_rot.pose.heading_rad <=. 1.58
  assert point.is_approximately_equal(
    state_rot.pose.position,
    Point2D(2.0, 0.0),
    0.001,
  )

  // 3. Differential wheel ticks: equal ticks moves forward
  let ticks_fwd = WheelTicks(left_ticks: 1000, right_ticks: 1000, dt_sec: 1.0)
  let state_ticks =
    odometry.propagate_differential_ticks(init_state, ticks_fwd, 0.1, 0.6, 1000)
  // Distance = 2 * pi * 0.1 = 0.6283m
  assert state_ticks.pose.position.x >=. 0.62
    && state_ticks.pose.position.x <=. 0.63

  // 4. Wheel slip detection
  let cmd = Twist2D(1.5, 0.0, 0.0)
  let meas_slipping = Twist2D(0.8, 0.0, 0.0)
  assert odometry.detect_wheel_slip(cmd, meas_slipping, 0.3, 0.2)
  let meas_traction = Twist2D(1.45, 0.0, 0.0)
  assert !odometry.detect_wheel_slip(cmd, meas_traction, 0.3, 0.2)
}
