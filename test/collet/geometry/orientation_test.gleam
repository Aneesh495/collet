import collet/geometry/math_ffi
import collet/geometry/orientation.{
  AngularInterval, Clockwise, Collinear, CounterClockwise,
}
import collet/geometry/point.{Point2D}

pub fn heading_conversion_and_normalization_test() {
  let pi = math_ffi.pi()
  let deg180 = orientation.from_degrees(180.0)
  assert deg180.radians >=. 3.1415 && deg180.radians <=. 3.1416

  let deg90 = orientation.from_degrees(90.0)
  assert orientation.to_degrees(deg90) == 90.0

  // 3 * pi normalizes to pi
  let norm_large = orientation.normalize(3.0 *. pi)
  assert norm_large >=. 3.1415 || norm_large <=. -3.1415

  // Positive circle interval
  let norm_pos = orientation.normalize_positive(-0.5 *. pi)
  assert norm_pos >=. 1.5 *. pi -. 0.0001 && norm_pos <=. 1.5 *. pi +. 0.0001
}

pub fn shortest_difference_and_distance_test() {
  let pi = math_ffi.pi()

  // From 170 deg (2.967 rad) to -170 deg (-2.967 rad):
  // Shortest route is counter-clockwise through pi: +20 deg (+0.349 rad)
  let rad_170 = 170.0 *. pi /. 180.0
  let rad_neg_170 = -170.0 *. pi /. 180.0
  let diff = orientation.shortest_difference(rad_170, rad_neg_170)
  let diff_deg = diff *. 180.0 /. pi
  assert diff_deg >=. 19.99 && diff_deg <=. 20.01

  let dist = orientation.angular_distance(rad_170, rad_neg_170)
  let dist_deg = dist *. 180.0 /. pi
  assert dist_deg >=. 19.99 && dist_deg <=. 20.01
}

pub fn slerp_and_turn_direction_test() {
  let pi = math_ffi.pi()
  let from_rad = 0.0
  let to_rad = pi /. 2.0

  let mid = orientation.slerp(from_rad, to_rad, 0.5)
  assert mid >=. pi /. 4.0 -. 0.0001 && mid <=. pi /. 4.0 +. 0.0001

  // Turn directions
  assert orientation.turn_direction(0.0, 0.5, 0.001) == CounterClockwise
  assert orientation.turn_direction(0.5, 0.0, 0.001) == Clockwise
  assert orientation.turn_direction(0.0, 0.00001, 0.001) == Collinear
}

pub fn vector_heading_and_interval_test() {
  let pi = math_ffi.pi()
  let unit_x = Point2D(1.0, 0.0)
  let unit_y = Point2D(0.0, 1.0)

  assert orientation.heading_from_vector(unit_x) == 0.0
  let hy = orientation.heading_from_vector(unit_y)
  assert hy >=. pi /. 2.0 -. 0.0001 && hy <=. pi /. 2.0 +. 0.0001

  // Non-wrapping interval [0, pi/2]
  let int_simple = AngularInterval(0.0, pi /. 2.0)
  assert orientation.interval_contains(int_simple, pi /. 4.0)
  assert !orientation.interval_contains(int_simple, pi)

  // Wrapping interval: [350 deg, 10 deg] across 0
  let rad_350 = 350.0 *. pi /. 180.0
  let rad_10 = 10.0 *. pi /. 180.0
  let int_wrapping = AngularInterval(rad_350, rad_10)

  assert orientation.interval_contains(int_wrapping, 0.0)
  let rad_5 = 5.0 *. pi /. 180.0
  assert orientation.interval_contains(int_wrapping, rad_5)
  let rad_180 = pi
  assert !orientation.interval_contains(int_wrapping, rad_180)
}
