import collet/geometry/point
import collet/geometry/polygon.{type Polygon, OrientedBoundingBox, Polygon}
import collet/kinematics/trajectory.{type Trajectory, type TrajectoryKnot}
import collet/kinematics/vehicle.{type VehicleProfile}
import gleam/list

// Constructs a convex swept bounding polygon representing the space swept by a vehicle
// between two consecutive trajectory knots [knot_start, knot_end].
// Prevents high-speed tunneling across discrete time steps.
pub fn swept_polygon(
  knot_start: TrajectoryKnot,
  knot_end: TrajectoryKnot,
  width: Float,
  length: Float,
) -> Polygon {
  let obb_start =
    OrientedBoundingBox(
      center: knot_start.position,
      width: width,
      length: length,
      heading: knot_start.heading_rad,
    )
  let obb_end =
    OrientedBoundingBox(
      center: knot_end.position,
      width: width,
      length: length,
      heading: knot_end.heading_rad,
    )

  let poly_start = polygon.obb_to_polygon(obb_start)
  let poly_end = polygon.obb_to_polygon(obb_end)

  // Swept envelope combines vertices of both footprints
  let all_verts = list.append(poly_start.vertices, poly_end.vertices)
  Polygon(vertices: all_verts)
}

// Checks if two swept trajectory intervals collide using the Separating Axis Theorem.
pub fn swept_intervals_collide(
  knot_a_start: TrajectoryKnot,
  knot_a_end: TrajectoryKnot,
  width_a: Float,
  length_a: Float,
  knot_b_start: TrajectoryKnot,
  knot_b_end: TrajectoryKnot,
  width_b: Float,
  length_b: Float,
) -> Bool {
  // Check if time intervals overlap
  let time_disjoint =
    knot_a_end.time_sec <. knot_b_start.time_sec
    || knot_b_end.time_sec <. knot_a_start.time_sec

  case time_disjoint {
    True -> False
    False -> {
      // Broad-phase spatial distance check on centers
      let center_a = point.midpoint(knot_a_start.position, knot_a_end.position)
      let center_b = point.midpoint(knot_b_start.position, knot_b_end.position)
      let dist = point.distance(center_a, center_b)

      let radius_a = { width_a +. length_a } /. 2.0
      let radius_b = { width_b +. length_b } /. 2.0

      case dist >. radius_a +. radius_b +. 2.0 {
        // Broad phase eliminates distant swept segments
        True -> False
        False -> {
          // Narrow phase: construct exact swept polygons and apply SAT
          let swept_a =
            swept_polygon(knot_a_start, knot_a_end, width_a, length_a)
          let swept_b =
            swept_polygon(knot_b_start, knot_b_end, width_b, length_b)
          polygon.polygons_intersect_sat(swept_a, swept_b)
        }
      }
    }
  }
}

// Full continuous swept collision scan across two complete trajectories.
pub fn scan_trajectories(
  traj_a: Trajectory,
  prof_a: VehicleProfile,
  traj_b: Trajectory,
  prof_b: VehicleProfile,
) -> Bool {
  let segments_a = get_trajectory_segments(traj_a.knots)
  let segments_b = get_trajectory_segments(traj_b.knots)

  list.any(segments_a, fn(seg_a) {
    let #(k_a1, k_a2) = seg_a
    list.any(segments_b, fn(seg_b) {
      let #(k_b1, k_b2) = seg_b
      swept_intervals_collide(
        k_a1,
        k_a2,
        prof_a.width_meters,
        prof_a.length_meters,
        k_b1,
        k_b2,
        prof_b.width_meters,
        prof_b.length_meters,
      )
    })
  })
}

// Extracts pairwise sequential knot segments [(k_0, k_1), (k_1, k_2), ...]
fn get_trajectory_segments(
  knots: List(TrajectoryKnot),
) -> List(#(TrajectoryKnot, TrajectoryKnot)) {
  case knots {
    [] -> []
    [_single] -> []
    [_first, ..rest] -> {
      let pairs = list.zip(knots, rest)
      pairs
    }
  }
}
