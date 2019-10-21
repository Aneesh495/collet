import collet/geometry/point.{Point2D}
import collet/kinematics/trajectory
import collet/kinematics/vehicle
import collet/reservation/clearance
import collet/reservation/conflict
import collet/reservation/sweep
import collet/reservation/table.{TimeInterval}

pub fn interval_overlap_and_table_test() {
  let int_a = TimeInterval(0.0, 5.0)
  let int_b = TimeInterval(4.5, 8.0)
  let int_c = TimeInterval(6.0, 10.0)

  // A and B overlap between 4.5 and 5.0
  assert table.intervals_overlap(int_a, int_b, 0.01)
  // A and C do not overlap
  assert !table.intervals_overlap(int_a, int_c, 0.01)

  let tbl = table.new()

  // Reserve node N1 for AMR_1 from 0 to 5s
  case table.reserve_node(tbl, "AMR_1", "N1", int_a) {
    Ok(tbl2) -> {
      // N1 is not available for overlapping interval B
      assert !table.is_node_available(tbl2, "N1", int_b)
      // N1 is available for disjoint interval C
      assert table.is_node_available(tbl2, "N1", int_c)

      // Another robot attempting to book overlapping interval should fail
      case table.reserve_node(tbl2, "AMR_2", "N1", int_b) {
        Error(conflicting_id) -> {
          assert conflicting_id == "AMR_1"
          Nil
        }
        Ok(_) -> panic as "Overlapping reservation should have failed"
      }

      // Cancelling AMR_1 frees up N1
      let tbl_freed = table.cancel_vehicle(tbl2, "AMR_1")
      assert table.is_node_available(tbl_freed, "N1", int_b)
    }
    Error(_) -> panic as "Initial reservation failed"
  }
}

pub fn edge_reservation_and_pruning_test() {
  let tbl = table.new()
  let int_early = TimeInterval(0.0, 3.0)
  let int_late = TimeInterval(10.0, 15.0)

  case table.reserve_edge(tbl, "AMR_A", "E1", int_early) {
    Ok(tbl2) -> {
      case table.reserve_edge(tbl2, "AMR_B", "E2", int_late) {
        Ok(tbl3) -> {
          // At simulation time 5.0, E1 reservation should be pruned, E2 remains
          let pruned = table.prune_expired(tbl3, 5.0)
          assert table.is_edge_available(pruned, "E1", int_early)
          assert !table.is_edge_available(pruned, "E2", int_late)
        }
        Error(_) -> panic as "E2 reservation failed"
      }
    }
    Error(_) -> panic as "E1 reservation failed"
  }
}

pub fn dynamic_clearance_and_headway_test() {
  let amr = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")

  // At standstill (v = 0.0): length is base length 1.0m, width is base width 0.8 + 2*0.15 = 1.1m
  let #(w0, l0) = clearance.dynamic_clearance_dimensions(amr, 0.0)
  assert w0 >=. 1.09 && w0 <=. 1.11
  assert l0 >=. 0.99 && l0 <=. 1.01

  // At 1.8 m/s: length expands due to reaction distance (0.36m) + stopping distance (1.35m)
  let #(_, l_fast) = clearance.dynamic_clearance_dimensions(amr, 1.8)
  assert l_fast >=. 2.7 && l_fast <=. 2.72

  // Minimum safe headway
  let gap = clearance.minimum_safe_headway(1.5, 1.2, 0.5, 1.2, 0.5)
  assert gap >. 0.5
}

pub fn swept_continuous_collision_test() {
  let amr = vehicle.standard_differential_amr("AMR_1", "Hermes 1000")

  // Trajectory A: moves from (0, 0) to (10, 0) between t=0 and t=5s
  let traj_a =
    trajectory.generate_linear(
      Point2D(0.0, 0.0),
      Point2D(10.0, 0.0),
      0.0,
      0.0,
      amr,
      0.0,
    )

  // Trajectory B: orthogonal path crossing at (5, 0) from (5, -5) to (5, 5) at the exact same time
  let traj_b_colliding =
    trajectory.generate_linear(
      Point2D(5.0, -5.0),
      Point2D(5.0, 5.0),
      1.570796,
      1.570796,
      amr,
      0.0,
    )

  // Continuous sweep detects intersection
  let collides = sweep.scan_trajectories(traj_a, amr, traj_b_colliding, amr)
  assert collides

  // Conflict classifier also detects the footprint conflict
  case
    conflict.detect_trajectory_conflict(
      traj_a,
      "AMR_A",
      traj_b_colliding,
      "AMR_B",
      1.0,
    )
  {
    Ok(conf) -> {
      let #(a, b) = conflict.conflict_agents(conf)
      assert a == "AMR_A"
      assert b == "AMR_B"
      Nil
    }
    Error(_) -> panic as "Expected trajectory conflict not detected"
  }

  // Trajectory C: parallel path far away at Y=10.0 (no collision)
  let traj_c_disjoint =
    trajectory.generate_linear(
      Point2D(0.0, 10.0),
      Point2D(10.0, 10.0),
      0.0,
      0.0,
      amr,
      0.0,
    )
  assert !sweep.scan_trajectories(traj_a, amr, traj_c_disjoint, amr)
}
