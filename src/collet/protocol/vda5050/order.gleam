import collet/protocol/vda5050/header.{type Header}
import gleam/option.{type Option, None, Some}

// VDA 5050 spatial position record representing coordinates and orientation.
pub type NodePosition {
  NodePosition(x: Float, y: Float, theta: Option(Float), map_id: String)
}

// Control point for rational/non-rational polynomial splines (NURBS/Bezier).
pub type ControlPoint {
  ControlPoint(x: Float, y: Float, weight: Option(Float))
}

// Continuous trajectory description embedded in a VDA 5050 edge.
pub type TrajectoryDescription {
  TrajectoryDescription(
    degree: Int,
    knot_vector: List(Float),
    control_points: List(ControlPoint),
  )
}

// Action planned to trigger upon reaching a node or traversing an edge.
pub type OrderAction {
  OrderAction(
    action_type: String,
    action_id: String,
    action_description: Option(String),
    blocking_type: String,
  )
}

// Topological node entry within a VDA 5050 Order.
pub type OrderNode {
  OrderNode(
    node_id: String,
    sequence_id: Int,
    released: Bool,
    position: NodePosition,
    actions: List(OrderAction),
  )
}

// Directed edge entry within a VDA 5050 Order.
pub type OrderEdge {
  OrderEdge(
    edge_id: String,
    sequence_id: Int,
    released: Bool,
    start_node_id: String,
    end_node_id: String,
    max_speed_mps: Option(Float),
    trajectory: Option(TrajectoryDescription),
    actions: List(OrderAction),
  )
}

// Master VDA 5050 Order message sent from fleet controller to an individual AMR.
pub type Order {
  Order(
    header: Header,
    order_id: String,
    order_update_id: Int,
    zone_set_id: Option(String),
    nodes: List(OrderNode),
    edges: List(OrderEdge),
  )
}

// Helper to construct a base (released) order node.
pub fn new_released_node(
  node_id: String,
  sequence_id: Int,
  x: Float,
  y: Float,
  theta: Option(Float),
  map_id: String,
) -> OrderNode {
  OrderNode(
    node_id: node_id,
    sequence_id: sequence_id,
    released: True,
    position: NodePosition(x: x, y: y, theta: theta, map_id: map_id),
    actions: [],
  )
}

// Helper to construct a base (released) order edge.
pub fn new_released_edge(
  edge_id: String,
  sequence_id: Int,
  start_node_id: String,
  end_node_id: String,
  max_speed: Float,
) -> OrderEdge {
  OrderEdge(
    edge_id: edge_id,
    sequence_id: sequence_id,
    released: True,
    start_node_id: start_node_id,
    end_node_id: end_node_id,
    max_speed_mps: Some(max_speed),
    trajectory: None,
    actions: [],
  )
}
