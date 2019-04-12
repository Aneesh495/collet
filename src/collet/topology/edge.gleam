import collet/geometry/point.{type Point2D}
import collet/topology/node.{type NodeId}

// Unique identifier for an edge in the facility graph.
pub type EdgeId =
  String

// Geometric trajectory profile defining how the vehicle moves between nodes.
pub type TrajectoryType {
  // Direct Euclidean straight line segment.
  Linear
  // Smooth cubic spline with intermediate polynomial control points.
  CubicSpline(control_points: List(Point2D))
}

// Orientation constraint imposed on vehicles traversing this edge.
pub type OrientationConstraint {
  // Vehicle heading must align tangentially with trajectory velocity vector.
  Tangential
  // Vehicle maintains a fixed absolute heading throughout traversal (e.g. crabbing/mecanum).
  FixedHeading(radians: Float)
  // Vehicle heading is unconstrained (rotations allowed during translation).
  Unconstrained
}

// Directed topological connection between two nodes in the navigation graph.
pub type Edge {
  Edge(
    id: EdgeId,
    start_node_id: NodeId,
    end_node_id: NodeId,
    length_meters: Float,
    // Maximum permitted forward travel velocity in meters/second.
    max_forward_speed: Float,
    // Maximum permitted reverse travel velocity in meters/second (0.0 if reverse disallowed).
    max_reverse_speed: Float,
    // Maximum permitted angular yaw rate in radians/second.
    max_rotation_speed: Float,
    trajectory: TrajectoryType,
    orientation: OrientationConstraint,
    // Lateral aisle corridor clearance width in meters.
    corridor_width: Float,
    // Cost multiplier factor for weighted graph search (default 1.0).
    cost_weight: Float,
    // Dynamic temporary blockage flag (e.g. human obstruction or spill).
    is_blocked: Bool,
  )
}

// Constructs a standard linear transit edge.
pub fn new_linear(
  id: EdgeId,
  start_node_id: NodeId,
  end_node_id: NodeId,
  length_meters: Float,
  max_forward_speed: Float,
) -> Edge {
  Edge(
    id: id,
    start_node_id: start_node_id,
    end_node_id: end_node_id,
    length_meters: length_meters,
    max_forward_speed: max_forward_speed,
    max_reverse_speed: max_forward_speed *. 0.5,
    max_rotation_speed: 1.5,
    trajectory: Linear,
    orientation: Tangential,
    corridor_width: 1.6,
    cost_weight: 1.0,
    is_blocked: False,
  )
}

// Constructs a curved spline transit edge.
pub fn new_spline(
  id: EdgeId,
  start_node_id: NodeId,
  end_node_id: NodeId,
  length_meters: Float,
  max_forward_speed: Float,
  control_points: List(Point2D),
) -> Edge {
  Edge(
    id: id,
    start_node_id: start_node_id,
    end_node_id: end_node_id,
    length_meters: length_meters,
    max_forward_speed: max_forward_speed,
    max_reverse_speed: 0.0,
    max_rotation_speed: 1.0,
    trajectory: CubicSpline(control_points: control_points),
    orientation: Tangential,
    corridor_width: 1.8,
    cost_weight: 1.1,
    is_blocked: False,
  )
}

// Creates the opposite reverse directed edge (end -> start).
pub fn reverse(edge: Edge, reversed_id: EdgeId) -> Edge {
  Edge(
    ..edge,
    id: reversed_id,
    start_node_id: edge.end_node_id,
    end_node_id: edge.start_node_id,
  )
}

// Computes the nominal traversal duration in seconds assuming constant effective velocity.
pub fn nominal_duration(edge: Edge, vehicle_speed_limit: Float) -> Float {
  let effective_speed = case edge.max_forward_speed <. vehicle_speed_limit {
    True -> edge.max_forward_speed
    False -> vehicle_speed_limit
  }
  case effective_speed <. 0.01 {
    True -> 999999.0
    False -> edge.length_meters /. effective_speed
  }
}

// Sets the dynamic blockage status of an edge.
pub fn set_blocked(edge: Edge, blocked: Bool) -> Edge {
  Edge(..edge, is_blocked: blocked)
}
