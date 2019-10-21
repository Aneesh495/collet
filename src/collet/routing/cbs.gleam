import collet/kinematics/vehicle.{type VehicleProfile}
import collet/reservation/conflict.{type Conflict, VertexConflict}
import collet/reservation/table.{type VehicleId}
import collet/routing/space_time_astar.{
  type AgentConstraint, type PathResult, AvoidNode,
}
import collet/topology/map.{type WarehouseMap}
import collet/topology/node.{type NodeId}
import gleam/dict.{type Dict}
import gleam/list

// Initial assignment specifying an AMR's origin, destination, and physical profile.
pub type AgentTask {
  AgentTask(
    vehicle_id: VehicleId,
    profile: VehicleProfile,
    start_node_id: NodeId,
    goal_node_id: NodeId,
    start_time_sec: Float,
  )
}

// Complete multi-agent conflict-free solution returned by the CBS solver.
pub type CbsSolution {
  CbsSolution(
    paths: Dict(VehicleId, PathResult),
    sum_of_costs_sec: Float,
    makespan_sec: Float,
    tree_nodes_explored: Int,
  )
}

// Node in the high-level Constraint Tree (CT).
type ConstraintTreeNode {
  ConstraintTreeNode(
    constraints: Dict(VehicleId, List(AgentConstraint)),
    paths: Dict(VehicleId, PathResult),
    cost: Float,
  )
}

// High-level Conflict-Based Search (CBS) solver.
// Finds an optimal, collision-free joint schedule minimizing Sum-of-Costs.
pub fn solve(
  wmap: WarehouseMap,
  tasks: List(AgentTask),
  max_iterations: Int,
) -> Result(CbsSolution, String) {
  // 1. Root of the Constraint Tree: zero constraints for all agents
  let empty_constraints =
    list.fold(tasks, dict.new(), fn(acc, task) {
      dict.insert(acc, task.vehicle_id, [])
    })

  // Plan independent initial paths for all agents
  case plan_all_paths(wmap, tasks, empty_constraints, dict.new()) {
    Error(err) -> Error("Failed to find initial paths: " <> err)
    Ok(init_paths) -> {
      let root_cost = compute_sum_of_costs(init_paths)
      let root_ct =
        ConstraintTreeNode(
          constraints: empty_constraints,
          paths: init_paths,
          cost: root_cost,
        )

      let open_tree = [root_ct]
      search_constraint_tree(wmap, tasks, open_tree, max_iterations, 0)
    }
  }
}

// Recursive high-level Constraint Tree branch-and-bound search.
fn search_constraint_tree(
  wmap: WarehouseMap,
  tasks: List(AgentTask),
  open_tree: List(ConstraintTreeNode),
  max_iterations: Int,
  iterations: Int,
) -> Result(CbsSolution, String) {
  case open_tree {
    [] -> Error("Constraint tree exhausted; no collision-free solution exists")
    [best_ct, ..rest_tree] -> {
      case iterations >= max_iterations {
        True -> Error("CBS solver exceeded maximum search iterations")
        False -> {
          // Detect the first conflict among current joint paths
          case find_first_conflict(best_ct.paths) {
            // No conflicts: optimal solution found!
            Error(Nil) -> {
              let makespan = compute_makespan(best_ct.paths)
              Ok(CbsSolution(
                paths: best_ct.paths,
                sum_of_costs_sec: best_ct.cost,
                makespan_sec: makespan,
                tree_nodes_explored: iterations + 1,
              ))
            }
            // Conflict identified: branch into two child nodes
            Ok(conf) -> {
              let #(agent_a, agent_b) = conflict.conflict_agents(conf)
              let time = conflict.conflict_time(conf)

              let node_a_id =
                get_agent_node_at_time(best_ct.paths, agent_a, time)
              let constraint_a = AvoidNode(node_id: node_a_id, time_sec: time)

              let node_b_id =
                get_agent_node_at_time(best_ct.paths, agent_b, time)
              let constraint_b = AvoidNode(node_id: node_b_id, time_sec: time)

              // Branch Child A: impose constraint on agent A
              let child_a_opt =
                create_child_ct(wmap, tasks, best_ct, agent_a, constraint_a)
              // Branch Child B: impose constraint on agent B
              let child_b_opt =
                create_child_ct(wmap, tasks, best_ct, agent_b, constraint_b)

              let new_children =
                list.filter_map([child_a_opt, child_b_opt], fn(opt) { opt })
              let next_open = merge_ct_nodes(rest_tree, new_children)

              search_constraint_tree(
                wmap,
                tasks,
                next_open,
                max_iterations,
                iterations + 1,
              )
            }
          }
        }
      }
    }
  }
}

// Creates a child CT node by adding a constraint and re-planning only the affected agent.
fn create_child_ct(
  wmap: WarehouseMap,
  tasks: List(AgentTask),
  parent_ct: ConstraintTreeNode,
  replan_agent_id: VehicleId,
  new_constraint: AgentConstraint,
) -> Result(ConstraintTreeNode, Nil) {
  let current_agent_constraints = case
    dict.get(parent_ct.constraints, replan_agent_id)
  {
    Ok(c_list) -> [new_constraint, ..c_list]
    Error(Nil) -> [new_constraint]
  }

  let updated_constraints =
    dict.insert(
      parent_ct.constraints,
      replan_agent_id,
      current_agent_constraints,
    )

  // Find task for replan_agent
  case list.find(tasks, fn(t) { t.vehicle_id == replan_agent_id }) {
    Error(Nil) -> Error(Nil)
    Ok(task) -> {
      // Re-plan agent path with new constraints
      case
        space_time_astar.search(
          wmap,
          task.profile,
          table.new(),
          current_agent_constraints,
          task.start_node_id,
          task.start_time_sec,
          task.goal_node_id,
          100.0,
        )
      {
        Error(Nil) -> Error(Nil)
        Ok(new_path) -> {
          let updated_paths =
            dict.insert(parent_ct.paths, replan_agent_id, new_path)
          let new_cost = compute_sum_of_costs(updated_paths)
          Ok(ConstraintTreeNode(
            constraints: updated_constraints,
            paths: updated_paths,
            cost: new_cost,
          ))
        }
      }
    }
  }
}

// Computes Sum of Costs: sum of total travel times of all agents.
fn compute_sum_of_costs(paths: Dict(VehicleId, PathResult)) -> Float {
  dict.values(paths)
  |> list.fold(0.0, fn(acc, p) { acc +. p.total_travel_time_sec })
}

// Computes Makespan: maximum arrival time across all agents.
fn compute_makespan(paths: Dict(VehicleId, PathResult)) -> Float {
  dict.values(paths)
  |> list.fold(0.0, fn(max_m, p) {
    case p.total_travel_time_sec >. max_m {
      True -> p.total_travel_time_sec
      False -> max_m
    }
  })
}

// Plans paths for all tasks independently.
fn plan_all_paths(
  wmap: WarehouseMap,
  tasks: List(AgentTask),
  constraints: Dict(VehicleId, List(AgentConstraint)),
  acc_paths: Dict(VehicleId, PathResult),
) -> Result(Dict(VehicleId, PathResult), String) {
  case tasks {
    [] -> Ok(acc_paths)
    [task, ..rest] -> {
      let agent_c = case dict.get(constraints, task.vehicle_id) {
        Ok(list) -> list
        Error(Nil) -> []
      }
      case
        space_time_astar.search(
          wmap,
          task.profile,
          table.new(),
          agent_c,
          task.start_node_id,
          task.start_time_sec,
          task.goal_node_id,
          100.0,
        )
      {
        Error(Nil) ->
          Error("Could not find path for vehicle " <> task.vehicle_id)
        Ok(path) -> {
          let next_paths = dict.insert(acc_paths, task.vehicle_id, path)
          plan_all_paths(wmap, rest, constraints, next_paths)
        }
      }
    }
  }
}

// Finds the first pairwise collision between any two planned agent routes.
fn find_first_conflict(
  paths: Dict(VehicleId, PathResult),
) -> Result(Conflict, Nil) {
  let agent_ids = dict.keys(paths)
  check_agent_pairs(agent_ids, paths)
}

fn check_agent_pairs(
  remaining_agents: List(VehicleId),
  paths: Dict(VehicleId, PathResult),
) -> Result(Conflict, Nil) {
  case remaining_agents {
    [] -> Error(Nil)
    [agent_a, ..other_agents] -> {
      let conflict_with_others =
        list.find_map(other_agents, fn(agent_b) {
          case dict.get(paths, agent_a), dict.get(paths, agent_b) {
            Ok(path_a), Ok(path_b) ->
              check_route_conflict(agent_a, path_a, agent_b, path_b)
            _, _ -> Error(Nil)
          }
        })

      case conflict_with_others {
        Ok(c) -> Ok(c)
        Error(Nil) -> check_agent_pairs(other_agents, paths)
      }
    }
  }
}

// Compares two routes for vertex and edge collisions.
fn check_route_conflict(
  agent_a: VehicleId,
  path_a: PathResult,
  agent_b: VehicleId,
  path_b: PathResult,
) -> Result(Conflict, Nil) {
  // Check vertex collisions: overlapping occupancy at same node
  list.find_map(path_a.route, fn(node_a) {
    list.find_map(path_b.route, fn(node_b) {
      case node_a.node_id == node_b.node_id {
        True -> {
          // Overlap in time window
          let t_diff = case
            node_a.arrival_time_sec >=. node_b.arrival_time_sec
          {
            True -> node_a.arrival_time_sec -. node_b.arrival_time_sec
            False -> node_b.arrival_time_sec -. node_a.arrival_time_sec
          }
          case t_diff <. 0.8 {
            True ->
              Ok(VertexConflict(
                agent_a: agent_a,
                agent_b: agent_b,
                node_id: node_a.node_id,
                time_sec: node_a.arrival_time_sec,
              ))
            False -> Error(Nil)
          }
        }
        False -> Error(Nil)
      }
    })
  })
}

// Retrieves the node ID occupied by an agent at time t.
fn get_agent_node_at_time(
  paths: Dict(VehicleId, PathResult),
  agent_id: VehicleId,
  time_sec: Float,
) -> NodeId {
  case dict.get(paths, agent_id) {
    Error(Nil) -> "UNKNOWN"
    Ok(path) -> {
      let candidate =
        list.find(path.route, fn(n) {
          n.arrival_time_sec <=. time_sec
          && time_sec <=. n.departure_time_sec +. 1.0
        })
      case candidate {
        Ok(node_entry) -> node_entry.node_id
        Error(Nil) ->
          case list.last(path.route) {
            Ok(last_n) -> last_n.node_id
            Error(Nil) -> "UNKNOWN"
          }
      }
    }
  }
}

// Merges CT nodes sorted by cost ascending.
fn merge_ct_nodes(
  existing: List(ConstraintTreeNode),
  new_nodes: List(ConstraintTreeNode),
) -> List(ConstraintTreeNode) {
  list.fold(new_nodes, existing, insert_ct_sorted)
}

fn insert_ct_sorted(
  list: List(ConstraintTreeNode),
  node: ConstraintTreeNode,
) -> List(ConstraintTreeNode) {
  case list {
    [] -> [node]
    [head, ..tail] -> {
      case node.cost <. head.cost {
        True -> [node, head, ..tail]
        False -> [head, ..insert_ct_sorted(tail, node)]
      }
    }
  }
}
