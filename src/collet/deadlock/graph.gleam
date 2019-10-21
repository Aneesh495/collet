import collet/reservation/table.{type VehicleId}
import gleam/dict.{type Dict}
import gleam/list

// Directed dependency edge in the fleet Wait-For Graph (WFG).
// Represents an agent waiting for a resource occupied by another agent.
pub type DependencyEdge {
  DependencyEdge(
    waiting_agent: VehicleId,
    blocking_agent: VehicleId,
    resource_id: String,
    wait_duration_sec: Float,
  )
}

// Global Wait-For Graph tracking inter-agent blocking relationships.
pub type WaitForGraph {
  WaitForGraph(
    // Adjacency: waiting_agent -> List(DependencyEdge)
    dependencies: Dict(VehicleId, List(DependencyEdge)),
  )
}

// Circular dependency cycle representing an active deadlock state.
pub type DeadlockCycle {
  DeadlockCycle(cycle_agents: List(VehicleId), longest_wait_sec: Float)
}

// Initializes an empty Wait-For Graph.
pub fn new() -> WaitForGraph {
  WaitForGraph(dependencies: dict.new())
}

// Records a dependency: waiting_agent is blocked by blocking_agent.
pub fn add_dependency(
  wfg: WaitForGraph,
  waiting: VehicleId,
  blocking: VehicleId,
  resource: String,
  duration_sec: Float,
) -> WaitForGraph {
  let edge =
    DependencyEdge(
      waiting_agent: waiting,
      blocking_agent: blocking,
      resource_id: resource,
      wait_duration_sec: duration_sec,
    )
  let current_list = case dict.get(wfg.dependencies, waiting) {
    Ok(list) -> list
    Error(Nil) -> []
  }
  let updated = dict.insert(wfg.dependencies, waiting, [edge, ..current_list])
  WaitForGraph(dependencies: updated)
}

// Detects all directed cycles in the Wait-For Graph using depth-first search.
pub fn find_deadlocks(wfg: WaitForGraph) -> List(DeadlockCycle) {
  let all_agents = dict.keys(wfg.dependencies)
  list.filter_map(all_agents, fn(start_agent) {
    case detect_cycle_from(wfg, start_agent, [start_agent], 0.0) {
      Error(Nil) -> Error(Nil)
      Ok(#(cycle_path, max_wait)) ->
        Ok(DeadlockCycle(cycle_agents: cycle_path, longest_wait_sec: max_wait))
    }
  })
}

// Predicate checking if any circular deadlock condition exists across the fleet.
pub fn is_deadlocked(wfg: WaitForGraph) -> Bool {
  !list.is_empty(find_deadlocks(wfg))
}

// Recursive DFS to detect directed cycles.
fn detect_cycle_from(
  wfg: WaitForGraph,
  current: VehicleId,
  visited_stack: List(VehicleId),
  accumulated_max_wait: Float,
) -> Result(#(List(VehicleId), Float), Nil) {
  case dict.get(wfg.dependencies, current) {
    Error(Nil) -> Error(Nil)
    Ok(edges) ->
      list.find_map(edges, fn(e) {
        let neighbor = e.blocking_agent
        let max_wait = case e.wait_duration_sec >. accumulated_max_wait {
          True -> e.wait_duration_sec
          False -> accumulated_max_wait
        }

        case list.contains(visited_stack, neighbor) {
          // Cycle found!
          True -> Ok(#([neighbor, ..visited_stack], max_wait))
          False ->
            detect_cycle_from(
              wfg,
              neighbor,
              [neighbor, ..visited_stack],
              max_wait,
            )
        }
      })
  }
}
