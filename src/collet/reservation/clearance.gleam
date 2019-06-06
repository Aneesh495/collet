import collet/geometry/math_ffi
import collet/geometry/point.{type Point2D}
import collet/geometry/polygon.{type OrientedBoundingBox, OrientedBoundingBox}
import collet/kinematics/vehicle.{type VehicleProfile}

// Reaction latency buffer in seconds (actuator communication delay + PLC compute cycle).
const reaction_time_seconds = 0.2

// Lateral tracking tolerance buffer in meters for localization jitter and wheel scrub.
const lateral_uncertainty_margin_meters = 0.15

// Computes dynamically expanded footprint dimensions accounting for braking distance and speed.
// In the longitudinal direction: L_eff = L_base + v * t_reaction + v^2 / (2 * a_decel)
// In the lateral direction: W_eff = W_base + 2 * margin
pub fn dynamic_clearance_dimensions(
  profile: VehicleProfile,
  linear_velocity_mps: Float,
) -> #(Float, Float) {
  let v = math_ffi.abs_float(linear_velocity_mps)
  let reaction_distance = v *. reaction_time_seconds
  let braking_distance = vehicle.stopping_distance(profile, v, False)

  let expanded_length =
    profile.length_meters +. reaction_distance +. braking_distance
  let expanded_width =
    profile.width_meters +. 2.0 *. lateral_uncertainty_margin_meters

  #(expanded_width, expanded_length)
}

// Generates an Oriented Bounding Box enclosing the dynamic safety envelope of the AMR.
pub fn dynamic_clearance_obb(
  profile: VehicleProfile,
  center: Point2D,
  heading_rad: Float,
  linear_velocity_mps: Float,
) -> OrientedBoundingBox {
  let #(w_eff, l_eff) = dynamic_clearance_dimensions(profile, linear_velocity_mps)
  OrientedBoundingBox(
    center: center,
    width: w_eff,
    length: l_eff,
    heading: heading_rad,
  )
}

// Computes the absolute minimum longitudinal headway required between two vehicles on an aisle.
// d_safe = v_follower * t_reaction + v_follower^2 / (2 * a_follower) - v_leader^2 / (2 * a_leader) + standstill_gap
pub fn minimum_safe_headway(
  follower_speed: Float,
  follower_decel: Float,
  leader_speed: Float,
  leader_decel: Float,
  standstill_gap_meters: Float,
) -> Float {
  let v_f = math_ffi.max_float(0.0, follower_speed)
  let v_l = math_ffi.max_float(0.0, leader_speed)

  let d_reaction = v_f *. reaction_time_seconds
  let d_f_stop = { v_f *. v_f } /. { 2.0 *. math_ffi.max_float(0.1, follower_decel) }
  let d_l_stop = { v_l *. v_l } /. { 2.0 *. math_ffi.max_float(0.1, leader_decel) }

  let raw_gap = d_reaction +. d_f_stop -. d_l_stop +. standstill_gap_meters
  math_ffi.max_float(standstill_gap_meters, raw_gap)
}
