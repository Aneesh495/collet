import collet/geometry/point

pub fn point_creation_and_origin_test() {
  let p = point.new(3.5, -2.1)
  assert p.x == 3.5
  assert p.y == -2.1

  let o = point.origin()
  assert o.x == 0.0
  assert o.y == 0.0
}

pub fn point_arithmetic_test() {
  let a = point.new(1.0, 2.0)
  let b = point.new(4.0, 6.0)

  let sum = point.add(a, b)
  assert sum.x == 5.0
  assert sum.y == 8.0

  let diff = point.sub(b, a)
  assert diff.x == 3.0
  assert diff.y == 4.0

  let scaled = point.scale(a, 2.5)
  assert scaled.x == 2.5
  assert scaled.y == 5.0

  let neg = point.negate(a)
  assert neg.x == -1.0
  assert neg.y == -2.0
}

pub fn dot_and_cross_product_test() {
  let v1 = point.new(1.0, 0.0)
  let v2 = point.new(0.0, 1.0)

  // Perpendicular vectors have dot product zero
  assert point.dot(v1, v2) == 0.0

  // Standard 2D cross product of unit X and unit Y is 1.0 (counter-clockwise)
  assert point.cross_2d(v1, v2) == 1.0
  assert point.cross_2d(v2, v1) == -1.0

  // Parallel vectors have cross product zero
  let v3 = point.new(2.0, 0.0)
  assert point.cross_2d(v1, v3) == 0.0
}

pub fn distance_and_norms_test() {
  let a = point.new(0.0, 0.0)
  let b = point.new(3.0, 4.0)

  assert point.squared_norm(b) == 25.0
  assert point.norm(b) == 5.0

  assert point.squared_distance(a, b) == 25.0
  assert point.distance(a, b) == 5.0

  assert point.manhattan_distance(a, b) == 7.0
  assert point.chebyshev_distance(a, b) == 4.0
}

pub fn vector_normalize_test() {
  let v = point.new(0.0, 5.0)
  case point.normalize(v) {
    Ok(unit_v) -> {
      assert point.is_approximately_equal(unit_v, point.new(0.0, 1.0), 0.0001)
    }
    Error(_) -> panic as "Normalization failed for non-zero vector"
  }

  // Zero vector should return Error
  let zero_v = point.new(0.0, 0.0)
  case point.normalize(zero_v) {
    Ok(_) -> panic as "Zero vector normalization should fail"
    Error(Nil) -> Nil
  }
}

pub fn lerp_and_midpoint_test() {
  let start = point.new(0.0, 10.0)
  let end = point.new(10.0, 20.0)

  let mid = point.midpoint(start, end)
  assert mid.x == 5.0
  assert mid.y == 15.0

  let at_zero = point.lerp(start, end, 0.0)
  assert at_zero.x == 0.0
  assert at_zero.y == 10.0

  let at_one = point.lerp(start, end, 1.0)
  assert at_one.x == 10.0
  assert at_one.y == 20.0

  let clamped_below = point.lerp(start, end, -0.5)
  assert clamped_below.x == 0.0

  let clamped_above = point.lerp(start, end, 1.5)
  assert clamped_above.x == 10.0
}

pub fn distance_to_segment_test() {
  let seg_start = point.new(0.0, 0.0)
  let seg_end = point.new(10.0, 0.0)

  // Point projecting onto the interior of the segment
  let p_interior = point.new(5.0, 3.0)
  let d_interior = point.distance_to_segment(p_interior, seg_start, seg_end)
  assert point.is_approximately_equal(
    point.new(d_interior, 0.0),
    point.new(3.0, 0.0),
    0.0001,
  )

  // Point closest to start endpoint
  let p_before = point.new(-3.0, 4.0)
  let d_before = point.distance_to_segment(p_before, seg_start, seg_end)
  assert point.is_approximately_equal(
    point.new(d_before, 0.0),
    point.new(5.0, 0.0),
    0.0001,
  )

  // Point closest to end endpoint
  let p_after = point.new(13.0, 4.0)
  let d_after = point.distance_to_segment(p_after, seg_start, seg_end)
  assert point.is_approximately_equal(
    point.new(d_after, 0.0),
    point.new(5.0, 0.0),
    0.0001,
  )

  // Degenerate segment: start equals end
  let p_deg = point.new(3.0, 4.0)
  let d_deg = point.distance_to_segment(p_deg, seg_start, seg_start)
  assert point.is_approximately_equal(
    point.new(d_deg, 0.0),
    point.new(5.0, 0.0),
    0.0001,
  )
}

pub fn lattice_point_conversion_test() {
  let cell_size = 1.0
  let pt = point.new(2.4, 3.8)
  let lat = point.to_lattice(pt, cell_size)
  assert lat.x == 2
  assert lat.y == 3

  let back_pt = point.from_lattice(lat, cell_size)
  assert back_pt.x == 2.5
  assert back_pt.y == 3.5

  let lat_a = point.LatticePoint(x: 1, y: 1)
  let lat_b = point.LatticePoint(x: 4, y: 5)
  assert point.lattice_manhattan_distance(lat_a, lat_b) == 7
}
