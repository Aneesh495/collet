import collet/geometry/math_ffi
import collet/kinematics/vehicle.{type VehicleProfile}
import collet/reservation/table.{type ReservationTable, TimeInterval}
import collet/routing/heuristic
import collet/topology/edge.{type EdgeId}
import collet/topology/map.{type WarehouseMap}
import collet/topology/node.{type NodeId}
import gleam/dict.{type Dict}
import gleam/list
import gleam/option.{type Option, None, Some}

// Discrete state in the space-time search graph: position node u at timestamp t.
pub type SpaceTimeNode {
  SpaceTimeNode(node_id: NodeId, time_sec: Float)
}

// Scheduled waypoint in a planned space-time route.
pub type TimedRouteNode {
  TimedRouteNode(
    node_id: NodeId,
    arrival_time_sec: Float,
    departure_time_sec: Float,
    traversing_edge_id: Option(EdgeId),
  )
}

// Single-agent avoidance constraint passed from high-level CBS solver.
pub type AgentConstraint {
  // Agent forbidden from occupying node at time t
  AvoidNode(node_id: NodeId, time_sec: Float)
  // Agent forbidden from traversing edge at time t
  AvoidEdge(edge_id: EdgeId, time_sec: Float)
}

// Complete result of a single-agent space-time path planning query.
pub type PathResult {
  PathResult(
    route: List(TimedRouteNode),
    total_travel_time_sec: Float,
    nodes_expanded: Int,
  )
}

// Internal node representation inside the A* search tree.
type SearchNode {
  SearchNode(
    node_id: NodeId,
    time_sec: Float,
    g_cost: Float,
    f_cost: Float,
    parent: Option(SearchNode),
    edge_taken: Option(EdgeId),
  )
}

// Executes single-agent Space-Time A* search respecting reservation table and constraints.
pub fn search(
  wmap: WarehouseMap,
  veh_profile: VehicleProfile,
  res_table: ReservationTable,
  constraints: List(AgentConstraint),
  start_node_id: NodeId,
  start_time_sec: Float,
  goal_node_id: NodeId,
  max_horizon_sec: Float,
) -> Result(PathResult, Nil) {
  // Target node position for heuristic
  case map.get_node(wmap, goal_node_id) {
    Error(Nil) -> Error(Nil)
    Ok(goal_node) -> {
      let h_table =
        heuristic.precompute_reverse_dijkstra(
          wmap,
          goal_node_id,
          veh_profile.max_linear_velocity,
        )
      let initial_h =
        heuristic.lookup(
          h_table,
          start_node_id,
          heuristic.euclidean(
            goal_node.position,
            goal_node.position,
            veh_profile.max_linear_velocity,
          ),
        )

      let start_search_node =
        SearchNode(
          node_id: start_node_id,
          time_sec: start_time_sec,
          g_cost: 0.0,
          f_cost: initial_h,
          parent: None,
          edge_taken: None,
        )

      let open_set = [start_search_node]
      let closed_set = dict.new()

      run_sta_star(
        wmap,
        veh_profile,
        res_table,
        constraints,
        h_table,
        goal_node_id,
        start_time_sec +. max_horizon_sec,
        open_set,
        closed_set,
        0,
      )
    }
  }
}

// Recursive Space-Time A* loop.
fn run_sta_star(
  wmap: WarehouseMap,
  veh: VehicleProfile,
  res_table: ReservationTable,
  constraints: List(AgentConstraint),
  h_table: Dict(NodeId, Float),
  goal_node_id: NodeId,
  horizon_cutoff_sec: Float,
  open_set: List(SearchNode),
  closed_set: Dict(String, Float),
  expansions: Int,
) -> Result(PathResult, Nil) {
  case open_set {
    [] -> Error(Nil)
    [best, ..rest_open] -> {
      // Check goal condition
      case best.node_id == goal_node_id {
        True -> {
          let route = reconstruct_path(best, [])
          Ok(
            PathResult(
              route: route,
              total_travel_time_sec: best.g_cost,
              nodes_expanded: expansions,
            ),
          )
        }
        False -> {
          case best.time_sec >. horizon_cutoff_sec || expansions > 5000 {
            True -> Error(Nil)
            False -> {
              let state_key =
                best.node_id <> "@" <> float_to_quantized_str(best.time_sec)
              let already_closed = case dict.get(closed_set, state_key) {
                Ok(prev_g) -> prev_g <=. best.g_cost
                Error(Nil) -> False
              }

              case already_closed {
                True ->
                  run_sta_star(
                    wmap,
                    veh,
                    res_table,
                    constraints,
                    h_table,
                    goal_node_id,
                    horizon_cutoff_sec,
                    rest_open,
                    closed_set,
                    expansions + 1,
                  )
                False -> {
                  let updated_closed =
                    dict.insert(closed_set, state_key, best.g_cost)
                  let successors =
                    generate_successors(
                      wmap,
                      veh,
                      res_table,
                      constraints,
                      h_table,
                      best,
                    )
                  let next_open = merge_open_set(rest_open, successors)

                  run_sta_star(
                    wmap,
                    veh,
                    res_table,
                    constraints,
                    h_table,
                    goal_node_id,
                    horizon_cutoff_sec,
                    next_open,
                    updated_closed,
                    expansions + 1,
                  )
                }
              }
            }
          }
        }
      }
    }
  }
}

// Generates valid transition actions: moving along an outgoing edge or waiting at current node.
fn generate_successors(
  wmap: WarehouseMap,
  veh: VehicleProfile,
  res_table: ReservationTable,
  constraints: List(AgentConstraint),
  h_table: Dict(NodeId, Float),
  current: SearchNode,
) -> List(SearchNode) {
  // 1. Traverse outgoing edges
  let neighbors = map.get_neighbors(wmap, current.node_id)
  let move_successors =
    list.filter_map(neighbors, fn(neighbor_pair) {
      let #(target_node, edge_obj) = neighbor_pair
      let effective_speed =
        math_ffi.min_float(
          edge_obj.max_forward_speed,
          veh.max_linear_velocity,
        )
      let travel_time =
        edge_obj.length_meters /. effective_speed *. edge_obj.cost_weight
      let arrival_time = current.time_sec +. travel_time
      let interval = TimeInterval(current.time_sec, arrival_time)

      // Check reservation table availability for edge and target node
      let edge_free =
        table.is_edge_available(res_table, edge_obj.id, interval)
      let node_free =
        table.is_node_available(
          res_table,
          target_node.id,
          TimeInterval(arrival_time, arrival_time +. 0.5),
        )

      // Check external CBS constraints
      let edge_constrained =
        list.any(constraints, fn(c) {
          case c {
            AvoidEdge(eid, t) ->
              eid == edge_obj.id
              && math_ffi.abs_float(t -. current.time_sec) <=. 0.5
            _ -> False
          }
        })

      let node_constrained =
        list.any(constraints, fn(c) {
          case c {
            AvoidNode(nid, t) ->
              nid == target_node.id
              && math_ffi.abs_float(t -. arrival_time) <=. 0.5
            _ -> False
          }
        })

      case
        edge_free
        && node_free
        && !edge_constrained
        && !node_constrained
      {
        True -> {
          let g = current.g_cost +. travel_time
          let h = heuristic.lookup(h_table, target_node.id, 0.0)
          Ok(
            SearchNode(
              node_id: target_node.id,
              time_sec: arrival_time,
              g_cost: g,
              f_cost: g +. h,
              parent: Some(current),
              edge_taken: Some(edge_obj.id),
            ),
          )
        }
        False -> Error(Nil)
      }
    })

  // 2. Wait at current node for delta_t (e.g. 1.0 second)
  let wait_dt = 1.0
  let wait_until = current.time_sec +. wait_dt
  let wait_interval = TimeInterval(current.time_sec, wait_until)
  let wait_node_free =
    table.is_node_available(res_table, current.node_id, wait_interval)
  let wait_constrained =
    list.any(constraints, fn(c) {
      case c {
        AvoidNode(nid, t) ->
          nid == current.node_id
          && math_ffi.abs_float(t -. wait_until) <=. 0.5
        _ -> False
      }
    })

  let wait_successor = case wait_node_free && !wait_constrained {
    True -> {
      let g = current.g_cost +. wait_dt
      let h = heuristic.lookup(h_table, current.node_id, 0.0)
      [
        SearchNode(
          node_id: current.node_id,
          time_sec: wait_until,
          g_cost: g,
          f_cost: g +. h,
          parent: Some(current),
          edge_taken: None,
        ),
      ]
    }
    False -> []
  }

  list.append(move_successors, wait_successor)
}

// Merges new successors into priority open set sorted by f_cost ascending.
fn merge_open_set(
  open: List(SearchNode),
  new_nodes: List(SearchNode),
) -> List(SearchNode) {
  list.fold(new_nodes, open, insert_sorted)
}

fn insert_sorted(list: List(SearchNode), node: SearchNode) -> List(SearchNode) {
  case list {
    [] -> [node]
    [head, ..tail] -> {
      case node.f_cost <. head.f_cost {
        True -> [node, head, ..tail]
        False -> [head, ..insert_sorted(tail, node)]
      }
    }
  }
}

// Backtracks through parent pointers to construct the sequential timed route.
fn reconstruct_path(
  node: SearchNode,
  acc: List(TimedRouteNode),
) -> List(TimedRouteNode) {
  let timed_node =
    TimedRouteNode(
      node_id: node.node_id,
      arrival_time_sec: node.time_sec,
      departure_time_sec: node.time_sec,
      traversing_edge_id: node.edge_taken,
    )
  let new_acc = [timed_node, ..acc]
  case node.parent {
    None -> new_acc
    Some(p) -> reconstruct_path(p, new_acc)
  }
}

// Quantizes continuous time into discrete string buckets (100ms precision) for closed set hash.
fn float_to_quantized_str(t: Float) -> String {
  let quantized = math_ffi.floor(t *. 10.0)
  float_to_str_erl(quantized)
}

@external(erlang, "erlang", "float_to_binary")
fn float_to_str_erl(x: Float) -> String
