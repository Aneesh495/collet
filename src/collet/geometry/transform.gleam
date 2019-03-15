import collet/geometry/math_ffi
import collet/geometry/point.{type Point2D, Point2D}

// Special Euclidean group SE(2) rigid body transformation in the 2D plane.
// Represents a translation (x, y) in meters and a rotation theta in radians.
pub type Transform2D {
  Transform2D(x: Float, y: Float, theta: Float)
}

// The identity transformation: zero translation, zero rotation.
pub fn identity() -> Transform2D {
  Transform2D(x: 0.0, y: 0.0, theta: 0.0)
}

// Creates a pure translation transformation.
pub fn from_translation(x: Float, y: Float) -> Transform2D {
  Transform2D(x: x, y: y, theta: 0.0)
}

// Creates a pure rotation transformation around the origin.
pub fn from_rotation(theta: Float) -> Transform2D {
  Transform2D(x: 0.0, y: 0.0, theta: theta)
}

// Composes two SE(2) transformations: applies 'parent' first, then 'child'.
// Corresponds to matrix multiplication: T_composite = T_parent * T_child.
pub fn compose(parent: Transform2D, child: Transform2D) -> Transform2D {
  let c = math_ffi.cos(parent.theta)
  let s = math_ffi.sin(parent.theta)
  let new_x = parent.x +. c *. child.x -. s *. child.y
  let new_y = parent.y +. s *. child.x +. c *. child.y
  let new_theta = normalize_angle(parent.theta +. child.theta)
  Transform2D(x: new_x, y: new_y, theta: new_theta)
}

// Computes the exact inverse of an SE(2) transformation: T_inv * T = Identity.
pub fn inverse(t: Transform2D) -> Transform2D {
  let c = math_ffi.cos(t.theta)
  let s = math_ffi.sin(t.theta)
  let inv_x = 0.0 -. { c *. t.x +. s *. t.y }
  let inv_y = s *. t.x -. c *. t.y
  let inv_theta = normalize_angle(0.0 -. t.theta)
  Transform2D(x: inv_x, y: inv_y, theta: inv_theta)
}

// Maps a point in local body coordinates to world coordinates.
// p_world = R(theta) * p_local + translation.
pub fn apply_to_point(t: Transform2D, p: Point2D) -> Point2D {
  let c = math_ffi.cos(t.theta)
  let s = math_ffi.sin(t.theta)
  Point2D(
    x: t.x +. c *. p.x -. s *. p.y,
    y: t.y +. s *. p.x +. c *. p.y,
  )
}

// Maps a directional vector in local body coordinates to world coordinates.
// Directional vectors are unaffected by translation (pure rotation).
pub fn apply_to_vector(t: Transform2D, v: Point2D) -> Point2D {
  let c = math_ffi.cos(t.theta)
  let s = math_ffi.sin(t.theta)
  Point2D(
    x: c *. v.x -. s *. v.y,
    y: s *. v.x +. c *. v.y,
  )
}

// Maps a point in world coordinates back into local body coordinates.
// Equivalent to apply_to_point(inverse(t), world_pt).
pub fn world_to_local(t: Transform2D, world_pt: Point2D) -> Point2D {
  let inv = inverse(t)
  apply_to_point(inv, world_pt)
}

// Computes the relative SE(2) transformation from pose 'source' to pose 'target'.
// Relative = inverse(source) * target.
pub fn relative_transform(source: Transform2D, target: Transform2D) -> Transform2D {
  compose(inverse(source), target)
}

// Geodesic interpolation along the SE(2) manifold between transform a and transform b.
// The interpolation parameter alpha is clamped to [0.0, 1.0].
pub fn interpolate(a: Transform2D, b: Transform2D, alpha: Float) -> Transform2D {
  let clamped_alpha = math_ffi.clamp_float(alpha, 0.0, 1.0)
  let rel = relative_transform(a, b)
  let interp_x = a.x +. { b.x -. a.x } *. clamped_alpha
  let interp_y = a.y +. { b.y -. a.y } *. clamped_alpha
  let interp_theta = normalize_angle(a.theta +. rel.theta *. clamped_alpha)
  Transform2D(x: interp_x, y: interp_y, theta: interp_theta)
}

// Normalizes an angle into the standard principal interval [-pi, +pi).
pub fn normalize_angle(angle: Float) -> Float {
  let two_pi = 2.0 *. math_ffi.pi()
  let a = math_ffi.atan2(math_ffi.sin(angle), math_ffi.cos(angle))
  case a <. 0.0 -. math_ffi.pi() {
    True -> a +. two_pi
    False -> a
  }
}
