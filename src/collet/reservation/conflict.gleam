import collet/geometry/point
import collet/kinematics/trajectory.{type Trajectory, type TrajectoryKnot}
import collet/reservation/table.{type VehicleId}
import collet/topology/edge.{type EdgeId}
import collet/topology/node.{type NodeId}

// Formal classification of spatio-temporal path conflicts between two agents.
pub type Conflict {
  // Vertex conflict: Two agents occupy the same topological node at overlapping times.
  VertexConflict(
    agent_a: VehicleId,
    agent_b: VehicleId,
    node_id: NodeId,
    time_sec: Float,
  )
  // Edge conflict: Two agents traverse the same directional edge with insufficient headway.
  EdgeConflict(
    agent_a: VehicleId,
    agent_b: VehicleId,
    edge_id: EdgeId,
    time_sec: Float,
  )
  // Swap conflict: Two agents traverse the same aisle segment in opposing directions (head-on).
  SwapConflict(
    agent_a: VehicleId,
    agent_b: VehicleId,
    node_u: NodeId,
    node_v: NodeId,
    time_sec: Float,
  )
  // Trailing conflict: Follower agent closes separation below safe dynamic stopping headway.
  TrailingConflict(
    follower_id: VehicleId,
    leader_id: VehicleId,
    edge_id: EdgeId,
    time_sec: Float,
  )
  // Continuous geometric footprint conflict detected by polygonal SAT sweep.
  FootprintConflict(
    agent_a: VehicleId,
    agent_b: VehicleId,
    time_sec: Float,
    separation_meters: Float,
  )
}

// Extracts the pair of conflicting vehicle IDs from a Conflict record.
pub fn conflict_agents(c: Conflict) -> #(VehicleId, VehicleId) {
  case c {
    VertexConflict(a, b, _, _) -> #(a, b)
    EdgeConflict(a, b, _, _) -> #(a, b)
    SwapConflict(a, b, _, _, _) -> #(a, b)
    TrailingConflict(f, l, _, _) -> #(f, l)
    FootprintConflict(a, b, _, _) -> #(a, b)
  }
}

// Extracts the critical timestamp when the conflict first materializes.
pub fn conflict_time(c: Conflict) -> Float {
  case c {
    VertexConflict(_, _, _, t) -> t
    EdgeConflict(_, _, _, t) -> t
    SwapConflict(_, _, _, _, t) -> t
    TrailingConflict(_, _, _, t) -> t
    FootprintConflict(_, _, t, _) -> t
  }
}

// Detects the earliest spatial collision between two executing trajectories.
pub fn detect_trajectory_conflict(
  traj_a: Trajectory,
  agent_a: VehicleId,
  traj_b: Trajectory,
  agent_b: VehicleId,
  safety_distance_meters: Float,
) -> Result(Conflict, Nil) {
  // Iterate through trajectory A knots and evaluate trajectory B at corresponding timestamps
  find_knot_collision(
    traj_a.knots,
    agent_a,
    traj_b,
    agent_b,
    safety_distance_meters,
  )
}

fn find_knot_collision(
  knots_a: List(TrajectoryKnot),
  agent_a: VehicleId,
  traj_b: Trajectory,
  agent_b: VehicleId,
  safety_dist: Float,
) -> Result(Conflict, Nil) {
  case knots_a {
    [] -> Error(Nil)
    [knot_a, ..rest] -> {
      case trajectory.evaluate_at_time(traj_b, knot_a.time_sec) {
        Error(Nil) ->
          find_knot_collision(
            rest,
            agent_a,
            traj_b,
            agent_b,
            safety_dist,
          )
        Ok(knot_b) -> {
          let dist = point.distance(knot_a.position, knot_b.position)
          case dist <. safety_dist {
            True ->
              Ok(
                FootprintConflict(
                  agent_a: agent_a,
                  agent_b: agent_b,
                  time_sec: knot_a.time_sec,
                  separation_meters: dist,
                ),
              )
            False ->
              find_knot_collision(
                rest,
                agent_a,
                traj_b,
                agent_b,
                safety_dist,
              )
          }
        }
      }
    }
  }
}
