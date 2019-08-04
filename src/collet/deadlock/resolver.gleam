import collet/deadlock/graph.{type DeadlockCycle}
import collet/reservation/table.{type ReservationTable, type VehicleId}
import collet/topology/map.{type WarehouseMap}
import collet/topology/node.{type NodeId}
import gleam/dict.{type Dict}
import gleam/list

// Tactical maneuver commanded to break a circular gridlock.
pub type ResolutionAction {
  // Pull aside into an adjacent parking buffer or holding station.
  YieldToBuffer(buffer_node_id: NodeId)
  // Reverse along current edge into preceding junction to yield right-of-way.
  ReverseToJunction(junction_node_id: NodeId)
  // Re-plan a detour route avoiding the congested bottleneck completely.
  RerouteDetour(detour_node_id: NodeId)
}

// Coordinated fleet resolution directive resolving a detected deadlock cycle.
pub type ResolutionPlan {
  ResolutionPlan(
    yield_agent_id: VehicleId,
    action: ResolutionAction,
    beneficiary_agent_id: VehicleId,
    reason: String,
  )
}

// Selects the optimal agent to yield and synthesizes an escape maneuver to dissolve the deadlock.
pub fn resolve(
  cycle: DeadlockCycle,
  priorities: Dict(VehicleId, Int),
  wmap: WarehouseMap,
  res_table: ReservationTable,
  current_agent_nodes: Dict(VehicleId, NodeId),
) -> Result(ResolutionPlan, String) {
  case cycle.cycle_agents {
    [] -> Error("Empty deadlock cycle cannot be resolved")
    [single] -> Error("Cycle must contain at least 2 distinct agents: " <> single)
    agents -> {
      // 1. Identify victim agent with the lowest operational priority (lowest integer value)
      let victim_id = select_lowest_priority_agent(agents, priorities)
      let other_agents = list.filter(agents, fn(a) { a != victim_id })
      let beneficiary_id = case list.first(other_agents) {
        Ok(b) -> b
        Error(Nil) -> "UNKNOWN"
      }

      // 2. Locate current position of the victim agent
      case dict.get(current_agent_nodes, victim_id) {
        Error(Nil) ->
          Error("Cannot locate current topological node for yield agent " <> victim_id)
        Ok(curr_node_id) -> {
          // 3. Search for adjacent parking buffer or vacant bypass node
          let neighbors = map.get_neighbors(wmap, curr_node_id)
          let buffer_candidate =
            list.find(neighbors, fn(pair) {
              let #(node_obj, _) = pair
              node.allows_parking(node_obj)
            })

          case buffer_candidate {
            // Found adjacent buffer: yield into buffer bay
            Ok(#(buf_node, _)) ->
              Ok(
                ResolutionPlan(
                  yield_agent_id: victim_id,
                  action: YieldToBuffer(buffer_node_id: buf_node.id),
                  beneficiary_agent_id: beneficiary_id,
                  reason: "Yielding lower priority vehicle into adjacent buffer bay " <> buf_node.id,
                ),
              )
            // No buffer: reverse into an available incoming neighbor junction
            Error(Nil) -> {
              let incoming_candidate =
                list.find(neighbors, fn(pair) {
                  let #(neighbor_node, _) = pair
                  table.is_node_available(
                    res_table,
                    neighbor_node.id,
                    table.TimeInterval(0.0, 10.0),
                  )
                })

              case incoming_candidate {
                Ok(#(free_node, _)) ->
                  Ok(
                    ResolutionPlan(
                      yield_agent_id: victim_id,
                      action: ReverseToJunction(junction_node_id: free_node.id),
                      beneficiary_agent_id: beneficiary_id,
                      reason: "Reversing into unreserved neighbor junction " <> free_node.id,
                    ),
                  )
                Error(Nil) ->
                  Ok(
                    ResolutionPlan(
                      yield_agent_id: victim_id,
                      action: RerouteDetour(detour_node_id: curr_node_id),
                      beneficiary_agent_id: beneficiary_id,
                      reason: "Forcing full dynamic reroute around congested aisle",
                    ),
                  )
              }
            }
          }
        }
      }
    }
  }
}

// Finds the vehicle in the cycle with the lowest priority rank.
fn select_lowest_priority_agent(
  agents: List(VehicleId),
  priorities: Dict(VehicleId, Int),
) -> VehicleId {
  case agents {
    [] -> "UNKNOWN"
    [first, ..rest] -> {
      let init_p = case dict.get(priorities, first) {
        Ok(p) -> p
        Error(Nil) -> 0
      }
      let #(victim, _) =
        list.fold(rest, #(first, init_p), fn(acc, agent_id) {
          let #(_curr_victim, min_p) = acc
          let p = case dict.get(priorities, agent_id) {
            Ok(val) -> val
            Error(Nil) -> 0
          }
          case p < min_p {
            True -> #(agent_id, p)
            False -> acc
          }
        })
      victim
    }
  }
}
