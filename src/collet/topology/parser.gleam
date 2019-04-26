import collet/geometry/point.{Point2D}
import collet/topology/edge.{type Edge}
import collet/topology/map.{type WarehouseMap}
import collet/topology/node.{type Node}
import gleam/dict
import gleam/json
import gleam/list

// Serializes a WarehouseMap to a formatted JSON string representation.
pub fn to_json(wmap: WarehouseMap) -> String {
  let nodes_json =
    dict.values(wmap.nodes)
    |> list.map(encode_node)
    |> json.preprocessed_array

  let edges_json =
    dict.values(wmap.edges)
    |> list.map(encode_edge)
    |> json.preprocessed_array

  json.object([
    #("id", json.string(wmap.id)),
    #("name", json.string(wmap.name)),
    #("nodes", nodes_json),
    #("edges", edges_json),
  ])
  |> json.to_string
}

// Encodes an individual topological node to a JSON object.
fn encode_node(n: Node) -> json.Json {
  json.object([
    #("id", json.string(n.id)),
    #("x", json.float(n.position.x)),
    #("y", json.float(n.position.y)),
    #("clearance_radius", json.float(n.clearance_radius)),
    #("max_wait_sec", json.float(n.max_wait_duration_sec)),
  ])
}

// Encodes an individual topological edge to a JSON object.
fn encode_edge(e: Edge) -> json.Json {
  json.object([
    #("id", json.string(e.id)),
    #("start_node_id", json.string(e.start_node_id)),
    #("end_node_id", json.string(e.end_node_id)),
    #("length_meters", json.float(e.length_meters)),
    #("max_forward_speed", json.float(e.max_forward_speed)),
    #("corridor_width", json.float(e.corridor_width)),
    #("is_blocked", json.bool(e.is_blocked)),
  ])
}

// Programmatically generates a standardized NxM orthogonal grid layout warehouse.
// Grid spacing is specified in meters (e.g. 2.0 meters between aisle nodes).
pub fn create_grid_warehouse(
  id: String,
  name: String,
  cols: Int,
  rows: Int,
  spacing_meters: Float,
) -> WarehouseMap {
  let empty_map = map.new(id, name)

  // 1. Generate nodes
  let col_indices = int_range(0, cols - 1)
  let row_indices = int_range(0, rows - 1)

  let node_map =
    list.fold(row_indices, empty_map, fn(map_acc, r) {
      list.fold(col_indices, map_acc, fn(m, c) {
        let node_id = "N_" <> int_to_string(c) <> "_" <> int_to_string(r)
        let pos =
          Point2D(
            x: int_to_float_erl(c) *. spacing_meters,
            y: int_to_float_erl(r) *. spacing_meters,
          )
        let n = node.new_transit(node_id, pos)
        map.add_node(m, n)
      })
    })

  // 2. Generate horizontal bidirectional edges
  let with_horiz =
    list.fold(row_indices, node_map, fn(map_acc, r) {
      let c_pairs = int_range(0, cols - 2)
      list.fold(c_pairs, map_acc, fn(m, c) {
        let u_id = "N_" <> int_to_string(c) <> "_" <> int_to_string(r)
        let v_id = "N_" <> int_to_string(c + 1) <> "_" <> int_to_string(r)
        let fwd_id = "E_H_" <> int_to_string(c) <> "_" <> int_to_string(r)
        let rev_id = "E_H_REV_" <> int_to_string(c) <> "_" <> int_to_string(r)
        let fwd_edge = edge.new_linear(fwd_id, u_id, v_id, spacing_meters, 2.0)
        map.add_bidirectional_edge(m, fwd_edge, rev_id)
      })
    })

  // 3. Generate vertical bidirectional edges
  let full_map =
    list.fold(col_indices, with_horiz, fn(map_acc, c) {
      let r_pairs = int_range(0, rows - 2)
      list.fold(r_pairs, map_acc, fn(m, r) {
        let u_id = "N_" <> int_to_string(c) <> "_" <> int_to_string(r)
        let v_id = "N_" <> int_to_string(c) <> "_" <> int_to_string(r + 1)
        let fwd_id = "E_V_" <> int_to_string(c) <> "_" <> int_to_string(r)
        let rev_id = "E_V_REV_" <> int_to_string(c) <> "_" <> int_to_string(r)
        let fwd_edge = edge.new_linear(fwd_id, u_id, v_id, spacing_meters, 2.0)
        map.add_bidirectional_edge(m, fwd_edge, rev_id)
      })
    })

  full_map
}

// Generates an inclusive integer range [start, stop].
fn int_range(start: Int, stop: Int) -> List(Int) {
  case start > stop {
    True -> []
    False -> [start, ..int_range(start + 1, stop)]
  }
}

@external(erlang, "erlang", "float")
fn int_to_float_erl(x: Int) -> Float

@external(erlang, "erlang", "integer_to_binary")
fn int_to_string(x: Int) -> String
