import collet/geometry/math_ffi

// Continuous two-dimensional Cartesian point in metric units (meters).
pub type Point2D {
  Point2D(x: Float, y: Float)
}

// Discrete integer coordinate representing a cell in a quantized space-time grid.
pub type LatticePoint {
  LatticePoint(x: Int, y: Int)
}

// Creates a new 2D point from Cartesian coordinates.
pub fn new(x: Float, y: Float) -> Point2D {
  Point2D(x: x, y: y)
}

// The coordinate origin (0.0, 0.0).
pub fn origin() -> Point2D {
  Point2D(x: 0.0, y: 0.0)
}

// Vector addition of two points.
pub fn add(a: Point2D, b: Point2D) -> Point2D {
  Point2D(x: a.x +. b.x, y: a.y +. b.y)
}

// Vector subtraction: computes (a - b).
pub fn sub(a: Point2D, b: Point2D) -> Point2D {
  Point2D(x: a.x -. b.x, y: a.y -. b.y)
}

// Multiplies vector components by a scalar factor.
pub fn scale(p: Point2D, factor: Float) -> Point2D {
  Point2D(x: p.x *. factor, y: p.y *. factor)
}

// Negates point coordinates.
pub fn negate(p: Point2D) -> Point2D {
  Point2D(x: 0.0 -. p.x, y: 0.0 -. p.y)
}

// Dot product (inner product) between two vectors.
pub fn dot(a: Point2D, b: Point2D) -> Float {
  a.x *. b.x +. a.y *. b.y
}

// 2D cross product (determinant): a.x * b.y - a.y * b.x.
// Represents the signed area of the parallelogram spanned by vectors a and b.
pub fn cross_2d(a: Point2D, b: Point2D) -> Float {
  a.x *. b.y -. a.y *. b.x
}

// Squared Euclidean norm (magnitude) of a point from the origin.
pub fn squared_norm(p: Point2D) -> Float {
  p.x *. p.x +. p.y *. p.y
}

// Euclidean norm (magnitude) of a point from the origin.
pub fn norm(p: Point2D) -> Float {
  math_ffi.sqrt(squared_norm(p))
}

// Squared Euclidean distance between points a and b.
// Useful for collision checks where computing the square root can be avoided.
pub fn squared_distance(a: Point2D, b: Point2D) -> Float {
  let dx = a.x -. b.x
  let dy = a.y -. b.y
  dx *. dx +. dy *. dy
}

// Euclidean distance between points a and b in meters.
pub fn distance(a: Point2D, b: Point2D) -> Float {
  math_ffi.sqrt(squared_distance(a, b))
}

// Manhattan (L1) distance between two points: |dx| + |dy|.
pub fn manhattan_distance(a: Point2D, b: Point2D) -> Float {
  math_ffi.abs_float(a.x -. b.x) +. math_ffi.abs_float(a.y -. b.y)
}

// Chebyshev (L-infinity) distance between two points: max(|dx|, |dy|).
pub fn chebyshev_distance(a: Point2D, b: Point2D) -> Float {
  let dx = math_ffi.abs_float(a.x -. b.x)
  let dy = math_ffi.abs_float(a.y -. b.y)
  math_ffi.max_float(dx, dy)
}

// Normalizes a vector to unit length.
// Returns an error if the vector length is zero or within numerical epsilon.
pub fn normalize(p: Point2D) -> Result(Point2D, Nil) {
  let len = norm(p)
  case len <. 0.000000001 {
    True -> Error(Nil)
    False -> Ok(Point2D(x: p.x /. len, y: p.y /. len))
  }
}

// Linear interpolation between point a and point b at interpolation parameter t in [0.0, 1.0].
pub fn lerp(a: Point2D, b: Point2D, t: Float) -> Point2D {
  let clamped_t = math_ffi.clamp_float(t, 0.0, 1.0)
  Point2D(
    x: a.x +. { b.x -. a.x } *. clamped_t,
    y: a.y +. { b.y -. a.y } *. clamped_t,
  )
}

// Computes the geometric midpoint between two points.
pub fn midpoint(a: Point2D, b: Point2D) -> Point2D {
  Point2D(x: { a.x +. b.x } /. 2.0, y: { a.y +. b.y } /. 2.0)
}

// Checks if two points are approximately equal within a specified tolerance epsilon.
pub fn is_approximately_equal(a: Point2D, b: Point2D, epsilon: Float) -> Bool {
  let dx = math_ffi.abs_float(a.x -. b.x)
  let dy = math_ffi.abs_float(a.y -. b.y)
  dx <=. epsilon && dy <=. epsilon
}

// Computes the shortest distance from a query point p to a line segment bounded by [seg_start, seg_end].
pub fn distance_to_segment(p: Point2D, seg_start: Point2D, seg_end: Point2D) -> Float {
  let seg_vec = sub(seg_end, seg_start)
  let seg_len_sq = squared_norm(seg_vec)
  case seg_len_sq <. 0.000000001 {
    // Degenerate segment: start equals end
    True -> distance(p, seg_start)
    False -> {
      let pt_vec = sub(p, seg_start)
      let projection = dot(pt_vec, seg_vec) /. seg_len_sq
      let t = math_ffi.clamp_float(projection, 0.0, 1.0)
      let closest_point = add(seg_start, scale(seg_vec, t))
      distance(p, closest_point)
    }
  }
}

// Converts a continuous point into a discrete lattice point given a spatial grid cell size.
pub fn to_lattice(p: Point2D, cell_size: Float) -> LatticePoint {
  let ix = math_ffi.floor(p.x /. cell_size)
  let iy = math_ffi.floor(p.y /. cell_size)
  LatticePoint(x: float_to_int_erl(ix), y: float_to_int_erl(iy))
}

// Converts a discrete lattice point to the continuous center of the grid cell.
pub fn from_lattice(lp: LatticePoint, cell_size: Float) -> Point2D {
  let fx = int_to_float_erl(lp.x) *. cell_size +. { cell_size /. 2.0 }
  let fy = int_to_float_erl(lp.y) *. cell_size +. { cell_size /. 2.0 }
  Point2D(x: fx, y: fy)
}

// Manhattan distance between two discrete lattice points.
pub fn lattice_manhattan_distance(a: LatticePoint, b: LatticePoint) -> Int {
  let dx = case a.x >= b.x {
    True -> a.x - b.x
    False -> b.x - a.x
  }
  let dy = case a.y >= b.y {
    True -> a.y - b.y
    False -> b.y - a.y
  }
  dx + dy
}

@external(erlang, "erlang", "trunc")
fn float_to_int_erl(x: Float) -> Int

@external(erlang, "erlang", "float")
fn int_to_float_erl(x: Int) -> Float
