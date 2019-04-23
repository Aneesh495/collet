import collet/geometry/math_ffi
import collet/geometry/point.{type Point2D}
import collet/geometry/polygon.{type Polygon}
import collet/topology/node.{type Node}
import gleam/list

// Unique identifier for a floor operational zone.
pub type ZoneId =
  String

// Operational classification and behavioral policy of a floor zone.
pub type ZoneType {
  // Prohibited zone: vehicles may never enter under normal operations.
  KeepOutZone
  // Speed-restricted envelope enforcing a reduced velocity limit.
  SpeedRestrictedZone(max_speed: Float)
  // Mutual exclusion corridor restricting simultaneous vehicle occupancy.
  MutexCorridor(max_concurrent_vehicles: Int)
  // Critical emergency route that must remain unblocked.
  FireAisle
  // Temporary dynamic obstruction with an expiration timestamp.
  TemporaryObstacle(expires_at_timestamp: Float)
}

// Bounded polygonal operational zone on the facility floor.
pub type Zone {
  Zone(
    id: ZoneId,
    name: String,
    boundary: Polygon,
    zone_type: ZoneType,
    is_active: Bool,
  )
}

// Constructs a prohibited keep-out zone.
pub fn new_keep_out(id: ZoneId, name: String, boundary: Polygon) -> Zone {
  Zone(
    id: id,
    name: name,
    boundary: boundary,
    zone_type: KeepOutZone,
    is_active: True,
  )
}

// Constructs a speed-restricted operational zone.
pub fn new_speed_restricted(
  id: ZoneId,
  name: String,
  boundary: Polygon,
  max_speed: Float,
) -> Zone {
  Zone(
    id: id,
    name: name,
    boundary: boundary,
    zone_type: SpeedRestrictedZone(max_speed: max_speed),
    is_active: True,
  )
}

// Constructs a mutual-exclusion narrow corridor zone.
pub fn new_mutex_corridor(
  id: ZoneId,
  name: String,
  boundary: Polygon,
  capacity: Int,
) -> Zone {
  Zone(
    id: id,
    name: name,
    boundary: boundary,
    zone_type: MutexCorridor(max_concurrent_vehicles: capacity),
    is_active: True,
  )
}

// Checks if a spatial point falls within the zone boundary.
pub fn contains_point(zone: Zone, pt: Point2D) -> Bool {
  case zone.is_active {
    False -> False
    True -> polygon.contains_point(zone.boundary, pt)
  }
}

// Checks if a topological node is situated inside the zone boundary.
pub fn contains_node(zone: Zone, node: Node) -> Bool {
  contains_point(zone, node.position)
}

// Checks if a spatial point is located in any active keep-out zone.
pub fn is_point_in_keep_out(zones: List(Zone), pt: Point2D) -> Bool {
  list.any(zones, fn(z) {
    case z.zone_type {
      KeepOutZone -> contains_point(z, pt)
      _ -> False
    }
  })
}

// Computes the effective speed limit at a spatial position given all overlapping active zones.
pub fn effective_speed_limit(
  zones: List(Zone),
  pt: Point2D,
  default_speed: Float,
) -> Float {
  list.fold(zones, default_speed, fn(current_limit, z) {
    case contains_point(z, pt) {
      False -> current_limit
      True ->
        case z.zone_type {
          SpeedRestrictedZone(max_s) -> math_ffi.min_float(current_limit, max_s)
          _ -> current_limit
        }
    }
  })
}
