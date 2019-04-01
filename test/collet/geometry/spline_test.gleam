import collet/geometry/point.{Point2D}
import collet/geometry/spline.{HermiteCubic}
import gleam/list

pub fn bezier_boundary_evaluation_test() {
  let p0 = Point2D(0.0, 0.0)
  let p1 = Point2D(0.0, 5.0)
  let p2 = Point2D(5.0, 10.0)
  let p3 = Point2D(10.0, 10.0)
  let b = spline.new_bezier(p0, p1, p2, p3)

  let start_eval = spline.evaluate(b, 0.0)
  assert point.is_approximately_equal(start_eval, p0, 0.0001)

  let end_eval = spline.evaluate(b, 1.0)
  assert point.is_approximately_equal(end_eval, p3, 0.0001)
}

pub fn linear_bezier_and_curvature_test() {
  // Collinear control points form a straight line of length 10.0
  let p0 = Point2D(0.0, 0.0)
  let p1 = Point2D(10.0 /. 3.0, 0.0)
  let p2 = Point2D(20.0 /. 3.0, 0.0)
  let p3 = Point2D(10.0, 0.0)
  let b = spline.new_bezier(p0, p1, p2, p3)

  let mid = spline.evaluate(b, 0.5)
  assert point.is_approximately_equal(mid, Point2D(5.0, 0.0), 0.001)

  // Curvature on straight line is 0.0
  let k_mid = spline.curvature(b, 0.5)
  assert k_mid <=. 0.000001

  // Arc length of straight line is 10.0
  let len = spline.arc_length(b, 10)
  assert len >=. 9.99 && len <=. 10.01

  // Tangent heading is 0.0 (pointing along +X)
  let heading = spline.tangent_heading(b, 0.5)
  assert heading <=. 0.0001 && heading >=. -0.0001
}

pub fn hermite_bezier_conversion_test() {
  let p0 = Point2D(0.0, 0.0)
  let v0 = Point2D(3.0, 0.0)
  let p1 = Point2D(10.0, 10.0)
  let v1 = Point2D(0.0, 3.0)

  let h = HermiteCubic(p0: p0, v0: v0, p1: p1, v1: v1)
  let b = spline.hermite_to_bezier(h)

  assert point.is_approximately_equal(b.p0, p0, 0.0001)
  assert point.is_approximately_equal(b.p3, p1, 0.0001)

  let h_recovered = spline.bezier_to_hermite(b)
  assert point.is_approximately_equal(h_recovered.p0, p0, 0.0001)
  assert point.is_approximately_equal(h_recovered.p1, p1, 0.0001)
  assert point.is_approximately_equal(h_recovered.v0, v0, 0.0001)
  assert point.is_approximately_equal(h_recovered.v1, v1, 0.0001)
}

pub fn waypoint_sampling_test() {
  let p0 = Point2D(0.0, 0.0)
  let p1 = Point2D(2.0, 5.0)
  let p2 = Point2D(8.0, 5.0)
  let p3 = Point2D(10.0, 0.0)
  let b = spline.new_bezier(p0, p1, p2, p3)

  let samples = spline.sample_waypoints(b, 5)
  assert list.length(samples) == 5

  case list.first(samples), list.last(samples) {
    Ok(first), Ok(last) -> {
      assert point.is_approximately_equal(first, p0, 0.0001)
      assert point.is_approximately_equal(last, p3, 0.0001)
    }
    _, _ -> panic as "Sampling failed"
  }
}
