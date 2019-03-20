import collet/geometry/math_ffi
import collet/geometry/point.{type Point2D}

// High-precision heading orientation in radians.
// Internally canonicalized to the principal branch [-pi, +pi).
pub type Heading {
  Heading(radians: Float)
}

// Direction of rotation on the planar manifold.
pub type TurnDirection {
  Clockwise
  CounterClockwise
  Collinear
}

// Angular interval defined by a start angle and an end angle.
// Traversed counter-clockwise from start to end.
pub type AngularInterval {
  AngularInterval(start_rad: Float, end_rad: Float)
}

// Creates a Heading from radians with automatic normalization.
pub fn from_radians(rad: Float) -> Heading {
  Heading(radians: normalize(rad))
}

// Creates a Heading from degrees with automatic conversion and normalization.
pub fn from_degrees(deg: Float) -> Heading {
  let rad = deg *. math_ffi.pi() /. 180.0
  Heading(radians: normalize(rad))
}

// Converts heading radians to degrees in the interval [-180.0, +180.0).
pub fn to_degrees(h: Heading) -> Float {
  h.radians *. 180.0 /. math_ffi.pi()
}

// Normalizes an arbitrary angle in radians to the principal branch [-pi, +pi).
pub fn normalize(angle: Float) -> Float {
  let two_pi = 2.0 *. math_ffi.pi()
  let a = math_ffi.atan2(math_ffi.sin(angle), math_ffi.cos(angle))
  case a <. 0.0 -. math_ffi.pi() {
    True -> a +. two_pi
    False -> a
  }
}

// Normalizes an angle to the positive circle interval [0.0, 2*pi).
pub fn normalize_positive(angle: Float) -> Float {
  let two_pi = 2.0 *. math_ffi.pi()
  let normalized = normalize(angle)
  case normalized <. 0.0 {
    True -> normalized +. two_pi
    False -> normalized
  }
}

// Computes the signed shortest angular difference (to - from) in [-pi, +pi].
// Positive values indicate counter-clockwise rotation; negative indicates clockwise.
pub fn shortest_difference(from_rad: Float, to_rad: Float) -> Float {
  let diff = to_rad -. from_rad
  normalize(diff)
}

// Computes the absolute angular distance between two headings in [0.0, pi].
pub fn angular_distance(a_rad: Float, b_rad: Float) -> Float {
  math_ffi.abs_float(shortest_difference(a_rad, b_rad))
}

// Interpolates smoothly along the shortest rotational arc between two angles.
pub fn slerp(from_rad: Float, to_rad: Float, t: Float) -> Float {
  let clamped_t = math_ffi.clamp_float(t, 0.0, 1.0)
  let diff = shortest_difference(from_rad, to_rad)
  normalize(from_rad +. diff *. clamped_t)
}

// Determines the turn direction for the shortest rotation from 'from_rad' to 'to_rad'.
pub fn turn_direction(
  from_rad: Float,
  to_rad: Float,
  tolerance: Float,
) -> TurnDirection {
  let diff = shortest_difference(from_rad, to_rad)
  case math_ffi.abs_float(diff) <=. tolerance {
    True -> Collinear
    False ->
      case diff >. 0.0 {
        True -> CounterClockwise
        False -> Clockwise
      }
  }
}

// Computes the heading angle corresponding to a 2D directional vector.
pub fn heading_from_vector(v: Point2D) -> Float {
  math_ffi.atan2(v.y, v.x)
}

// Checks if an angle falls within an angular interval (traversed counter-clockwise).
pub fn interval_contains(interval: AngularInterval, angle_rad: Float) -> Bool {
  let norm_start = normalize_positive(interval.start_rad)
  let norm_end = normalize_positive(interval.end_rad)
  let norm_target = normalize_positive(angle_rad)

  case norm_start <=. norm_end {
    // Non-wrapping interval
    True -> norm_target >=. norm_start && norm_target <=. norm_end
    // Wrapping interval across 0 / 2*pi boundary
    False -> norm_target >=. norm_start || norm_target <=. norm_end
  }
}
