import collet/geometry/point.{Point2D}
import collet/geometry/polygon.{AABB, OrientedBoundingBox}

pub fn aabb_computation_and_intersection_test() {
  let v1 = Point2D(0.0, 0.0)
  let v2 = Point2D(4.0, 0.0)
  let v3 = Point2D(4.0, 2.0)
  let v4 = Point2D(0.0, 2.0)
  let poly = polygon.new([v1, v2, v3, v4])

  case polygon.compute_aabb(poly) {
    Ok(box) -> {
      assert box.min_x == 0.0
      assert box.min_y == 0.0
      assert box.max_x == 4.0
      assert box.max_y == 2.0

      assert polygon.aabb_contains_point(box, Point2D(2.0, 1.0))
      assert !polygon.aabb_contains_point(box, Point2D(5.0, 1.0))

      let overlapping_box = AABB(min_x: 3.0, min_y: 1.0, max_x: 6.0, max_y: 3.0)
      assert polygon.aabb_intersects(box, overlapping_box)

      let disjoint_box = AABB(min_x: 5.0, min_y: 5.0, max_x: 7.0, max_y: 7.0)
      assert !polygon.aabb_intersects(box, disjoint_box)
    }
    Error(_) -> panic as "AABB computation failed"
  }
}

pub fn polygon_area_and_winding_test() {
  // Counter-clockwise 2x3 rectangle
  let poly_ccw =
    polygon.new([
      Point2D(0.0, 0.0),
      Point2D(2.0, 0.0),
      Point2D(2.0, 3.0),
      Point2D(0.0, 3.0),
    ])

  assert polygon.is_counter_clockwise(poly_ccw)
  assert polygon.signed_area(poly_ccw) == 6.0
  assert polygon.area(poly_ccw) == 6.0

  // Clockwise orientation has negative signed area
  let poly_cw =
    polygon.new([
      Point2D(0.0, 0.0),
      Point2D(0.0, 3.0),
      Point2D(2.0, 3.0),
      Point2D(2.0, 0.0),
    ])

  assert !polygon.is_counter_clockwise(poly_cw)
  assert polygon.signed_area(poly_cw) == -6.0
  assert polygon.area(poly_cw) == 6.0
}

pub fn point_containment_and_centroid_test() {
  let poly =
    polygon.new([
      Point2D(0.0, 0.0),
      Point2D(10.0, 0.0),
      Point2D(10.0, 10.0),
      Point2D(0.0, 10.0),
    ])

  // Center point is inside
  assert polygon.contains_point(poly, Point2D(5.0, 5.0))
  // Outside points
  assert !polygon.contains_point(poly, Point2D(-1.0, 5.0))
  assert !polygon.contains_point(poly, Point2D(11.0, 5.0))
  assert !polygon.contains_point(poly, Point2D(5.0, 15.0))

  // Centroid of square is at (5.0, 5.0)
  case polygon.centroid(poly) {
    Ok(c) -> {
      assert point.is_approximately_equal(c, Point2D(5.0, 5.0), 0.0001)
    }
    Error(_) -> panic as "Centroid computation failed"
  }
}

pub fn sat_collision_detection_test() {
  // Robot A: centered at (0, 0), length 2.0, width 1.0, heading 0.0
  let obb_a =
    OrientedBoundingBox(
      center: Point2D(0.0, 0.0),
      width: 1.0,
      length: 2.0,
      heading: 0.0,
    )
  let poly_a = polygon.obb_to_polygon(obb_a)

  // Robot B: centered at (1.5, 0.0), overlapping with Robot A
  let obb_b =
    OrientedBoundingBox(
      center: Point2D(1.5, 0.0),
      width: 1.0,
      length: 2.0,
      heading: 0.0,
    )
  let poly_b = polygon.obb_to_polygon(obb_b)

  assert polygon.polygons_intersect_sat(poly_a, poly_b)

  // Robot C: centered at (5.0, 5.0), clearly disjoint
  let obb_c =
    OrientedBoundingBox(
      center: Point2D(5.0, 5.0),
      width: 1.0,
      length: 2.0,
      heading: 0.0,
    )
  let poly_c = polygon.obb_to_polygon(obb_c)

  assert !polygon.polygons_intersect_sat(poly_a, poly_c)
}
