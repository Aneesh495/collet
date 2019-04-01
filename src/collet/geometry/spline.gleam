import collet/geometry/math_ffi
import collet/geometry/point.{type Point2D, Point2D}
import gleam/list

// Cubic Bezier curve segment defined by four control points (P0, P1, P2, P3).
// Fully compatible with the VDA 5050 trajectory knot specification.
pub type BezierCubic {
  BezierCubic(p0: Point2D, p1: Point2D, p2: Point2D, p3: Point2D)
}

// Cubic Hermite curve segment defined by endpoints (P0, P1) and boundary tangent vectors (V0, V1).
pub type HermiteCubic {
  HermiteCubic(p0: Point2D, v0: Point2D, p1: Point2D, v1: Point2D)
}

// Creates a new cubic Bezier segment.
pub fn new_bezier(
  p0: Point2D,
  p1: Point2D,
  p2: Point2D,
  p3: Point2D,
) -> BezierCubic {
  BezierCubic(p0: p0, p1: p1, p2: p2, p3: p3)
}

// Converts a cubic Hermite representation into an equivalent cubic Bezier representation.
// P1 = P0 + V0 / 3.0
// P2 = P1_endpoint - V1 / 3.0
pub fn hermite_to_bezier(h: HermiteCubic) -> BezierCubic {
  let p1 = point.add(h.p0, point.scale(h.v0, 1.0 /. 3.0))
  let p2 = point.sub(h.p1, point.scale(h.v1, 1.0 /. 3.0))
  BezierCubic(p0: h.p0, p1: p1, p2: p2, p3: h.p1)
}

// Converts a cubic Bezier curve into cubic Hermite parameters.
// V0 = 3 * (P1 - P0)
// V1 = 3 * (P3 - P2)
pub fn bezier_to_hermite(b: BezierCubic) -> HermiteCubic {
  let v0 = point.scale(point.sub(b.p1, b.p0), 3.0)
  let v1 = point.scale(point.sub(b.p3, b.p2), 3.0)
  HermiteCubic(p0: b.p0, v0: v0, p1: b.p3, v1: v1)
}

// Evaluates position along the cubic Bezier curve at parameter t in [0.0, 1.0].
// Uses explicit cubic Bernstein polynomial expansion.
pub fn evaluate(b: BezierCubic, t: Float) -> Point2D {
  let clamped_t = math_ffi.clamp_float(t, 0.0, 1.0)
  let one_minus_t = 1.0 -. clamped_t

  let c0 = one_minus_t *. one_minus_t *. one_minus_t
  let c1 = 3.0 *. one_minus_t *. one_minus_t *. clamped_t
  let c2 = 3.0 *. one_minus_t *. clamped_t *. clamped_t
  let c3 = clamped_t *. clamped_t *. clamped_t

  Point2D(
    x: c0 *. b.p0.x +. c1 *. b.p1.x +. c2 *. b.p2.x +. c3 *. b.p3.x,
    y: c0 *. b.p0.y +. c1 *. b.p1.y +. c2 *. b.p2.y +. c3 *. b.p3.y,
  )
}

// Evaluates the first derivative vector B'(t) representing velocity along the curve.
pub fn evaluate_derivative(b: BezierCubic, t: Float) -> Point2D {
  let clamped_t = math_ffi.clamp_float(t, 0.0, 1.0)
  let one_minus_t = 1.0 -. clamped_t

  let c0 = 3.0 *. one_minus_t *. one_minus_t
  let c1 = 6.0 *. one_minus_t *. clamped_t
  let c2 = 3.0 *. clamped_t *. clamped_t

  let d0 = point.sub(b.p1, b.p0)
  let d1 = point.sub(b.p2, b.p1)
  let d2 = point.sub(b.p3, b.p2)

  Point2D(
    x: c0 *. d0.x +. c1 *. d1.x +. c2 *. d2.x,
    y: c0 *. d0.y +. c1 *. d1.y +. c2 *. d2.y,
  )
}

// Evaluates the second derivative vector B''(t) representing acceleration along the curve.
pub fn evaluate_second_derivative(b: BezierCubic, t: Float) -> Point2D {
  let clamped_t = math_ffi.clamp_float(t, 0.0, 1.0)
  let one_minus_t = 1.0 -. clamped_t

  // a0 = 6 * (P2 - 2*P1 + P0)
  let a0_x = 6.0 *. { b.p2.x -. 2.0 *. b.p1.x +. b.p0.x }
  let a0_y = 6.0 *. { b.p2.y -. 2.0 *. b.p1.y +. b.p0.y }

  // a1 = 6 * (P3 - 2*P2 + P1)
  let a1_x = 6.0 *. { b.p3.x -. 2.0 *. b.p2.x +. b.p1.x }
  let a1_y = 6.0 *. { b.p3.y -. 2.0 *. b.p2.y +. b.p1.y }

  Point2D(
    x: one_minus_t *. a0_x +. clamped_t *. a1_x,
    y: one_minus_t *. a0_y +. clamped_t *. a1_y,
  )
}

// Computes instantaneous path curvature kappa(t) = |x' * y'' - y' * x''| / (x'^2 + y'^2)^(3/2).
// Curvature is in units of 1/meters. Turning radius R = 1.0 / kappa.
pub fn curvature(b: BezierCubic, t: Float) -> Float {
  let d1 = evaluate_derivative(b, t)
  let d2 = evaluate_second_derivative(b, t)

  let numerator = math_ffi.abs_float(d1.x *. d2.y -. d1.y *. d2.x)
  let speed_sq = d1.x *. d1.x +. d1.y *. d1.y

  case speed_sq <. 0.000000001 {
    True -> 0.0
    False -> {
      let speed = math_ffi.sqrt(speed_sq)
      numerator /. { speed *. speed_sq }
    }
  }
}

// Computes the tangent heading angle in radians at parameter t.
pub fn tangent_heading(b: BezierCubic, t: Float) -> Float {
  let d = evaluate_derivative(b, t)
  math_ffi.atan2(d.y, d.x)
}

// Computes total arc length of the spline using composite Simpson numerical integration.
pub fn arc_length(b: BezierCubic, intervals: Int) -> Float {
  let n = case intervals < 2 {
    True -> 10
    False ->
      case intervals % 2 != 0 {
        True -> intervals + 1
        False -> intervals
      }
  }

  let h = 1.0 /. int_to_float_erl(n)
  let f0 = speed_at(b, 0.0)
  let fn_val = speed_at(b, 1.0)

  let indices = int_range(1, n - 1)
  let sum_inner =
    list.fold(indices, 0.0, fn(acc, i) {
      let t_i = int_to_float_erl(i) *. h
      let s = speed_at(b, t_i)
      case i % 2 == 0 {
        True -> acc +. 2.0 *. s
        False -> acc +. 4.0 *. s
      }
    })

  { h /. 3.0 } *. { f0 +. fn_val +. sum_inner }
}

// Discretizes the curve into N equally-spaced parameter samples.
pub fn sample_waypoints(b: BezierCubic, count: Int) -> List(Point2D) {
  let safe_count = case count < 2 {
    True -> 2
    False -> count
  }
  let step = 1.0 /. int_to_float_erl(safe_count - 1)
  let indices = int_range(0, safe_count - 1)
  list.map(indices, fn(i) {
    let t = int_to_float_erl(i) *. step
    evaluate(b, t)
  })
}

// Internal helper: magnitude of curve tangent velocity vector at parameter t.
fn speed_at(b: BezierCubic, t: Float) -> Float {
  let d = evaluate_derivative(b, t)
  point.norm(d)
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
