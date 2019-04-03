import collet/geometry/orientation
import collet/geometry/point.{type Point2D}
import gleam/list

// Unique identifier for a topological node in the facility floor plan.
pub type NodeId =
  String

// Functional role and classification of a warehouse topological node.
pub type NodeType {
  // Standard transit intersection or aisle waypoint.
  TransitNode
  // Material pickup location where AMRs lift pallets or totes.
  PickupStation(station_id: String, payload_type: String)
  // Material drop-off location where AMRs deposit loads.
  DropoffStation(station_id: String, accepted_types: List(String))
  // Automated battery charging contact pad or induction coil.
  ChargingStation(charger_id: String, power_kw: Float, connector_type: String)
  // Safe parking buffer or side bay used for queuing and yield maneuvers.
  ParkBuffer(buffer_id: String)
  // Dedicated maintenance, calibration, or inspection cell.
  MaintenanceBay(bay_id: String)
}

// A discrete node in the topological navigation graph.
pub type Node {
  Node(
    id: NodeId,
    position: Point2D,
    elevation: Float,
    node_type: NodeType,
    // Allowed vehicle approach orientations in radians. Empty list allows any orientation.
    allowed_headings: List(Float),
    // Required spatial clearance radius in meters around the node center.
    clearance_radius: Float,
    // Maximum permitted dwell or waiting time in seconds (0.0 implies indefinite).
    max_wait_duration_sec: Float,
  )
}

// Constructs a standard transit junction node.
pub fn new_transit(id: NodeId, position: Point2D) -> Node {
  Node(
    id: id,
    position: position,
    elevation: 0.0,
    node_type: TransitNode,
    allowed_headings: [],
    clearance_radius: 0.8,
    max_wait_duration_sec: 0.0,
  )
}

// Constructs a pickup station node.
pub fn new_pickup(
  id: NodeId,
  position: Point2D,
  station_id: String,
  payload_type: String,
  required_heading: Float,
) -> Node {
  Node(
    id: id,
    position: position,
    elevation: 0.0,
    node_type: PickupStation(
      station_id: station_id,
      payload_type: payload_type,
    ),
    allowed_headings: [orientation.normalize(required_heading)],
    clearance_radius: 1.2,
    max_wait_duration_sec: 300.0,
  )
}

// Constructs a drop-off station node.
pub fn new_dropoff(
  id: NodeId,
  position: Point2D,
  station_id: String,
  accepted_types: List(String),
  required_heading: Float,
) -> Node {
  Node(
    id: id,
    position: position,
    elevation: 0.0,
    node_type: DropoffStation(
      station_id: station_id,
      accepted_types: accepted_types,
    ),
    allowed_headings: [orientation.normalize(required_heading)],
    clearance_radius: 1.2,
    max_wait_duration_sec: 300.0,
  )
}

// Constructs an automated charging pad node.
pub fn new_charger(
  id: NodeId,
  position: Point2D,
  charger_id: String,
  power_kw: Float,
  connector_type: String,
  docking_heading: Float,
) -> Node {
  Node(
    id: id,
    position: position,
    elevation: 0.0,
    node_type: ChargingStation(
      charger_id: charger_id,
      power_kw: power_kw,
      connector_type: connector_type,
    ),
    allowed_headings: [orientation.normalize(docking_heading)],
    clearance_radius: 1.0,
    max_wait_duration_sec: 0.0,
  )
}

// Constructs a park buffer node for holding idle vehicles or deadlock yield maneuvers.
pub fn new_park(id: NodeId, position: Point2D, buffer_id: String) -> Node {
  Node(
    id: id,
    position: position,
    elevation: 0.0,
    node_type: ParkBuffer(buffer_id: buffer_id),
    allowed_headings: [],
    clearance_radius: 0.9,
    max_wait_duration_sec: 0.0,
  )
}

// Computes Euclidean distance between two topological nodes.
pub fn distance(a: Node, b: Node) -> Float {
  point.distance(a.position, b.position)
}

// Checks if a candidate heading is permitted at the node within an angular tolerance.
pub fn is_heading_allowed(node: Node, heading: Float, tolerance: Float) -> Bool {
  case node.allowed_headings {
    // Empty list allows all orientations
    [] -> True
    headings -> {
      let target_norm = orientation.normalize(heading)
      list.any(headings, fn(h) {
        orientation.angular_distance(h, target_norm) <=. tolerance
      })
    }
  }
}

// Predicate checking if the node can serve as a parking or holding location.
pub fn allows_parking(node: Node) -> Bool {
  case node.node_type {
    ParkBuffer(_) -> True
    TransitNode -> True
    MaintenanceBay(_) -> True
    _ -> False
  }
}

// Predicate checking if the node possesses charging capability.
pub fn allows_charging(node: Node) -> Bool {
  case node.node_type {
    ChargingStation(_, _, _) -> True
    _ -> False
  }
}
