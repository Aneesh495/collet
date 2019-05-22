import collet/geometry/polygon.{type OrientedBoundingBox}
import collet/topology/edge.{type EdgeId}
import collet/topology/node.{type NodeId}
import gleam/dict.{type Dict}
import gleam/list

// Unique identifier of an operating vehicle or agent.
pub type VehicleId =
  String

// Bounded continuous-time interval [start_sec, end_sec].
pub type TimeInterval {
  TimeInterval(start_sec: Float, end_sec: Float)
}

// Spatio-temporal reservation locking a discrete topological node.
pub type NodeReservation {
  NodeReservation(
    vehicle_id: VehicleId,
    node_id: NodeId,
    interval: TimeInterval,
  )
}

// Spatio-temporal reservation locking a directed or bidirectional edge.
pub type EdgeReservation {
  EdgeReservation(
    vehicle_id: VehicleId,
    edge_id: EdgeId,
    interval: TimeInterval,
  )
}

// Fine-grained continuous-time geometric footprint reservation for SAT collision sweeps.
pub type FootprintReservation {
  FootprintReservation(
    vehicle_id: VehicleId,
    time_sec: Float,
    obb: OrientedBoundingBox,
  )
}

// Central dynamic reservation table tracking all active space-time bookings.
pub type ReservationTable {
  ReservationTable(
    node_reservations: Dict(NodeId, List(NodeReservation)),
    edge_reservations: Dict(EdgeId, List(EdgeReservation)),
    footprints: List(FootprintReservation),
  )
}

// Initializes an empty reservation table.
pub fn new() -> ReservationTable {
  ReservationTable(
    node_reservations: dict.new(),
    edge_reservations: dict.new(),
    footprints: [],
  )
}

// Checks if two time intervals overlap by more than a minimum epsilon duration.
pub fn intervals_overlap(
  a: TimeInterval,
  b: TimeInterval,
  epsilon_sec: Float,
) -> Bool {
  let disjoint =
    a.end_sec <=. b.start_sec +. epsilon_sec
    || b.end_sec <=. a.start_sec +. epsilon_sec
  !disjoint
}

// Checks if a candidate node is completely unreserved throughout the requested time interval.
pub fn is_node_available(
  table: ReservationTable,
  node_id: NodeId,
  interval: TimeInterval,
) -> Bool {
  case dict.get(table.node_reservations, node_id) {
    Error(Nil) -> True
    Ok(reservations) ->
      !list.any(reservations, fn(r) {
        intervals_overlap(r.interval, interval, 0.05)
      })
  }
}

// Checks if a candidate edge is completely unreserved throughout the requested time interval.
pub fn is_edge_available(
  table: ReservationTable,
  edge_id: EdgeId,
  interval: TimeInterval,
) -> Bool {
  case dict.get(table.edge_reservations, edge_id) {
    Error(Nil) -> True
    Ok(reservations) ->
      !list.any(reservations, fn(r) {
        intervals_overlap(r.interval, interval, 0.05)
      })
  }
}

// Commits a node reservation if no conflict exists; otherwise returns Error(conflicting_vehicle_id).
pub fn reserve_node(
  table: ReservationTable,
  vehicle_id: VehicleId,
  node_id: NodeId,
  interval: TimeInterval,
) -> Result(ReservationTable, VehicleId) {
  let existing = case dict.get(table.node_reservations, node_id) {
    Ok(list) -> list
    Error(Nil) -> []
  }

  // Check for any conflicting reservation
  let conflict_opt =
    list.find(existing, fn(r) { intervals_overlap(r.interval, interval, 0.05) })

  case conflict_opt {
    Ok(conflicting) -> Error(conflicting.vehicle_id)
    Error(Nil) -> {
      let new_res =
        NodeReservation(
          vehicle_id: vehicle_id,
          node_id: node_id,
          interval: interval,
        )
      let updated_map =
        dict.insert(table.node_reservations, node_id, [new_res, ..existing])
      Ok(ReservationTable(..table, node_reservations: updated_map))
    }
  }
}

// Commits an edge reservation if no conflict exists; otherwise returns Error(conflicting_vehicle_id).
pub fn reserve_edge(
  table: ReservationTable,
  vehicle_id: VehicleId,
  edge_id: EdgeId,
  interval: TimeInterval,
) -> Result(ReservationTable, VehicleId) {
  let existing = case dict.get(table.edge_reservations, edge_id) {
    Ok(list) -> list
    Error(Nil) -> []
  }

  let conflict_opt =
    list.find(existing, fn(r) { intervals_overlap(r.interval, interval, 0.05) })

  case conflict_opt {
    Ok(conflicting) -> Error(conflicting.vehicle_id)
    Error(Nil) -> {
      let new_res =
        EdgeReservation(
          vehicle_id: vehicle_id,
          edge_id: edge_id,
          interval: interval,
        )
      let updated_map =
        dict.insert(table.edge_reservations, edge_id, [new_res, ..existing])
      Ok(ReservationTable(..table, edge_reservations: updated_map))
    }
  }
}

// Revokes all active node, edge, and footprint reservations owned by a specific vehicle.
// Essential for dynamic rerouting, mission cancellation, and emergency preemption.
pub fn cancel_vehicle(
  table: ReservationTable,
  vehicle_id: VehicleId,
) -> ReservationTable {
  let filtered_nodes =
    dict.map_values(table.node_reservations, fn(_node_id, res_list) {
      list.filter(res_list, fn(r) { r.vehicle_id != vehicle_id })
    })

  let filtered_edges =
    dict.map_values(table.edge_reservations, fn(_edge_id, res_list) {
      list.filter(res_list, fn(r) { r.vehicle_id != vehicle_id })
    })

  let filtered_footprints =
    list.filter(table.footprints, fn(f) { f.vehicle_id != vehicle_id })

  ReservationTable(
    node_reservations: filtered_nodes,
    edge_reservations: filtered_edges,
    footprints: filtered_footprints,
  )
}

// Prunes historical reservations whose end timestamp is strictly prior to current simulation time.
pub fn prune_expired(
  table: ReservationTable,
  current_time_sec: Float,
) -> ReservationTable {
  let pruned_nodes =
    dict.map_values(table.node_reservations, fn(_nid, res_list) {
      list.filter(res_list, fn(r) { r.interval.end_sec >=. current_time_sec })
    })

  let pruned_edges =
    dict.map_values(table.edge_reservations, fn(_eid, res_list) {
      list.filter(res_list, fn(r) { r.interval.end_sec >=. current_time_sec })
    })

  let pruned_footprints =
    list.filter(table.footprints, fn(f) { f.time_sec >=. current_time_sec })

  ReservationTable(
    node_reservations: pruned_nodes,
    edge_reservations: pruned_edges,
    footprints: pruned_footprints,
  )
}
