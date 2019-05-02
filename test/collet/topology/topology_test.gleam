import collet/geometry/point.{Point2D}
import collet/geometry/polygon
import collet/topology/edge
import collet/topology/map
import collet/topology/node
import collet/topology/parser
import collet/topology/zone
import gleam/dict
import gleam/list
import gleam/string

pub fn node_types_and_predicates_test() {
  let p_transit = node.new_transit("N1", Point2D(0.0, 0.0))
  assert node.allows_parking(p_transit)
  assert !node.allows_charging(p_transit)

  let p_pickup = node.new_pickup("N2", Point2D(5.0, 0.0), "STN_A", "PALLET_EUR", 0.0)
  assert !node.allows_parking(p_pickup)
  assert node.is_heading_allowed(p_pickup, 0.05, 0.1)
  assert !node.is_heading_allowed(p_pickup, 1.5, 0.1)

  let p_charger = node.new_charger("N3", Point2D(10.0, 0.0), "CHG_1", 22.0, "INDUCTION", 0.0)
  assert node.allows_charging(p_charger)
  assert node.distance(p_transit, p_charger) == 10.0
}

pub fn edge_operations_test() {
  let e_fwd = edge.new_linear("E1", "N1", "N2", 10.0, 2.0)
  assert e_fwd.length_meters == 10.0
  assert e_fwd.max_forward_speed == 2.0
  assert edge.nominal_duration(e_fwd, 2.0) == 5.0
  assert edge.nominal_duration(e_fwd, 1.0) == 10.0

  let e_rev = edge.reverse(e_fwd, "E1_REV")
  assert e_rev.id == "E1_REV"
  assert e_rev.start_node_id == "N2"
  assert e_rev.end_node_id == "N1"

  let e_blocked = edge.set_blocked(e_fwd, True)
  assert e_blocked.is_blocked
}

pub fn warehouse_map_test() {
  let wmap = map.new("MAP_1", "Main Assembly Facility")
  let n1 = node.new_transit("N1", Point2D(0.0, 0.0))
  let n2 = node.new_transit("N2", Point2D(5.0, 0.0))
  let e1 = edge.new_linear("E1", "N1", "N2", 5.0, 2.0)

  let built_map =
    wmap
    |> map.add_node(n1)
    |> map.add_node(n2)
    |> map.add_edge(e1)

  case map.get_node(built_map, "N1") {
    Ok(retrieved) -> {
      assert retrieved.id == "N1"
      Nil
    }
    Error(_) -> panic as "Node N1 not found"
  }

  case map.get_edge(built_map, "E1") {
    Ok(retrieved_e) -> {
      assert retrieved_e.start_node_id == "N1"
      Nil
    }
    Error(_) -> panic as "Edge E1 not found"
  }

  let neighbors = map.get_neighbors(built_map, "N1")
  assert list.length(neighbors) == 1
  case list.first(neighbors) {
    Ok(#(neigh_node, _)) -> {
      assert neigh_node.id == "N2"
      Nil
    }
    Error(_) -> panic as "Missing neighbor"
  }

  // Nearest node query
  case map.nearest_node(built_map, Point2D(0.5, 0.2)) {
    Ok(closest) -> {
      assert closest.id == "N1"
      Nil
    }
    Error(_) -> panic as "Nearest node query failed"
  }

  // Validation passes on valid map
  assert list.is_empty(map.validate(built_map))

  // Validation flags broken edges
  let broken_edge = edge.new_linear("E_BAD", "N1", "NON_EXISTENT", 5.0, 2.0)
  let invalid_map = map.add_edge(built_map, broken_edge)
  let errors = map.validate(invalid_map)
  assert list.length(errors) == 1
}

pub fn operational_zones_test() {
  let poly_boundary =
    polygon.new([
      Point2D(0.0, 0.0),
      Point2D(10.0, 0.0),
      Point2D(10.0, 10.0),
      Point2D(0.0, 10.0),
    ])

  let zone_speed =
    zone.new_speed_restricted("Z_SLOW", "Slow Aisle", poly_boundary, 0.8)
  assert zone.contains_point(zone_speed, Point2D(5.0, 5.0))
  assert !zone.contains_point(zone_speed, Point2D(15.0, 5.0))

  let eff_speed =
    zone.effective_speed_limit([zone_speed], Point2D(5.0, 5.0), 2.5)
  assert eff_speed == 0.8

  let zone_keepout =
    zone.new_keep_out("Z_HAZARD", "Battery Pit", poly_boundary)
  assert zone.is_point_in_keep_out([zone_keepout], Point2D(5.0, 5.0))
  assert !zone.is_point_in_keep_out([zone_keepout], Point2D(15.0, 5.0))
}

pub fn grid_warehouse_generator_and_json_test() {
  let grid = parser.create_grid_warehouse("GRID_3x3", "Aisle Test 3x3", 3, 3, 2.5)
  // 3x3 = 9 nodes
  assert dict.size(grid.nodes) == 9
  // Horizontal edges: 3 rows * 2 columns * 2 directions = 12
  // Vertical edges: 3 cols * 2 rows * 2 directions = 12
  // Total edges = 24
  assert dict.size(grid.edges) == 24

  let json_str = parser.to_json(grid)
  assert string.contains(json_str, "GRID_3x3")
  assert string.contains(json_str, "Aisle Test 3x3")
  assert string.contains(json_str, "nodes")
  assert string.contains(json_str, "edges")
}
