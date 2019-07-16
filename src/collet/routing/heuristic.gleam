import collet/geometry/math_ffi
import collet/geometry/point.{type Point2D}
import collet/topology/map.{type WarehouseMap}
import collet/topology/node.{type NodeId}
import gleam/dict.{type Dict}
import gleam/list

// Admissible Euclidean heuristic estimating minimum travel duration to goal: h = dist / v_max.
pub fn euclidean(from: Point2D, to: Point2D, v_max: Float) -> Float {
  let safe_v = math_ffi.max_float(0.1, v_max)
  point.distance(from, to) /. safe_v
}

// Precomputes the exact, consistent reverse-Dijkstra heuristic lookup table from all graph
// nodes to the specified target node.
// Accounts for topology detours and one-way directional edges on the static map.
pub fn precompute_reverse_dijkstra(
  wmap: WarehouseMap,
  target_node_id: NodeId,
  v_max: Float,
) -> Dict(NodeId, Float) {
  let init_distances = dict.insert(dict.new(), target_node_id, 0.0)
  let unvisited = [#(target_node_id, 0.0)]

  run_reverse_dijkstra(wmap, init_distances, unvisited, v_max)
}

// Looks up precomputed heuristic value, falling back to a default if disconnected.
pub fn lookup(
  heuristic_table: Dict(NodeId, Float),
  node_id: NodeId,
  fallback_val: Float,
) -> Float {
  case dict.get(heuristic_table, node_id) {
    Ok(val) -> val
    Error(Nil) -> fallback_val
  }
}

// Iterative reverse-Dijkstra traversal backwards along incoming edges.
fn run_reverse_dijkstra(
  wmap: WarehouseMap,
  distances: Dict(NodeId, Float),
  open_set: List(#(NodeId, Float)),
  v_max: Float,
) -> Dict(NodeId, Float) {
  case open_set {
    [] -> distances
    [#(current_id, current_cost), ..rest] -> {
      // Find incoming edges to current_id
      let incoming_edge_ids = case dict.get(wmap.incoming, current_id) {
        Ok(ids) -> ids
        Error(Nil) -> []
      }

      let #(updated_dist, new_open) =
        list.fold(incoming_edge_ids, #(distances, rest), fn(acc, eid) {
          let #(curr_dist_map, curr_open_list) = acc
          case dict.get(wmap.edges, eid) {
            Error(Nil) -> acc
            Ok(e) -> {
              let pred_id = e.start_node_id
              let safe_speed =
                math_ffi.min_float(
                  math_ffi.max_float(0.1, e.max_forward_speed),
                  v_max,
                )
              let edge_time = e.length_meters /. safe_speed *. e.cost_weight
              let new_cost = current_cost +. edge_time

              let is_shorter = case dict.get(curr_dist_map, pred_id) {
                Error(Nil) -> True
                Ok(existing_cost) -> new_cost <. existing_cost
              }

              case is_shorter {
                True -> {
                  let next_dist_map =
                    dict.insert(curr_dist_map, pred_id, new_cost)
                  let next_open_list = [#(pred_id, new_cost), ..curr_open_list]
                  #(next_dist_map, next_open_list)
                }
                False -> acc
              }
            }
          }
        })

      run_reverse_dijkstra(wmap, updated_dist, new_open, v_max)
    }
  }
}
