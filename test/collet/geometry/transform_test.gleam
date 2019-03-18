import collet/geometry/math_ffi
import collet/geometry/point
import collet/geometry/transform

pub fn identity_transform_test() {
  let id = transform.identity()
  let pt = point.new(4.2, -7.8)

  let transformed_pt = transform.apply_to_point(id, pt)
  assert point.is_approximately_equal(transformed_pt, pt, 0.0001)

  let vec = point.new(1.0, 3.0)
  let transformed_vec = transform.apply_to_vector(id, vec)
  assert point.is_approximately_equal(transformed_vec, vec, 0.0001)
}

pub fn translation_and_rotation_test() {
  let t_trans = transform.from_translation(10.0, 5.0)
  let pt = point.new(2.0, 3.0)

  let pt_trans = transform.apply_to_point(t_trans, pt)
  assert point.is_approximately_equal(pt_trans, point.new(12.0, 8.0), 0.0001)

  // Pure translation does not change vectors
  let vec = point.new(2.0, 3.0)
  let vec_trans = transform.apply_to_vector(t_trans, vec)
  assert point.is_approximately_equal(vec_trans, vec, 0.0001)

  // 90-degree counter-clockwise rotation (pi / 2)
  let pi_half = math_ffi.pi() /. 2.0
  let t_rot = transform.from_rotation(pi_half)
  let unit_x = point.new(1.0, 0.0)

  let rotated_x = transform.apply_to_point(t_rot, unit_x)
  assert point.is_approximately_equal(rotated_x, point.new(0.0, 1.0), 0.0001)
}

pub fn composition_and_inverse_test() {
  let t1 = transform.Transform2D(x: 5.0, y: 10.0, theta: 0.5)
  let t2 = transform.Transform2D(x: -2.0, y: 3.0, theta: -0.2)

  let comp = transform.compose(t1, t2)
  let pt = point.new(1.0, 1.0)

  // Composed application should match sequential application
  let pt_seq = transform.apply_to_point(t1, transform.apply_to_point(t2, pt))
  let pt_comp = transform.apply_to_point(comp, pt)
  assert point.is_approximately_equal(pt_seq, pt_comp, 0.0001)

  // Inverse application should recover original point
  let inv1 = transform.inverse(t1)
  let id_pt = transform.apply_to_point(inv1, transform.apply_to_point(t1, pt))
  assert point.is_approximately_equal(id_pt, pt, 0.0001)

  // World to local roundtrip
  let local_pt = transform.world_to_local(t1, pt_seq)
  let recovered_world = transform.apply_to_point(t1, local_pt)
  assert point.is_approximately_equal(recovered_world, pt_seq, 0.0001)
}

pub fn relative_transform_and_interpolation_test() {
  let pose_a = transform.Transform2D(x: 0.0, y: 0.0, theta: 0.0)
  let pose_b = transform.Transform2D(x: 10.0, y: 0.0, theta: 1.0)

  let rel = transform.relative_transform(pose_a, pose_b)
  assert rel.x == 10.0
  assert rel.y == 0.0
  assert point.is_approximately_equal(
    point.new(rel.theta, 0.0),
    point.new(1.0, 0.0),
    0.0001,
  )

  let mid_pose = transform.interpolate(pose_a, pose_b, 0.5)
  assert mid_pose.x == 5.0
  assert mid_pose.y == 0.0
  assert point.is_approximately_equal(
    point.new(mid_pose.theta, 0.0),
    point.new(0.5, 0.0),
    0.0001,
  )
}
