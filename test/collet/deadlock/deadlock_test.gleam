import collet/deadlock/graph
import collet/deadlock/resolver.{ReverseToJunction, YieldToBuffer}
import collet/geometry/point.{Point2D}
import collet/reservation/table
import collet/topology/edge
import collet/topology/map
import collet/topology/node
import gleam/dict
import gleam/list

pub fn wfg_acyclic_no_deadlock_test() {
  let wfg =
    graph.new()
    |> graph.add_dependency("amr-1", "amr-2", "node-B", 3.5)

  assert graph.is_deadlocked(wfg) == False
  assert list.is_empty(graph.find_deadlocks(wfg)) == True
}

pub fn wfg_two_agent_head_on_deadlock_test() {
  let wfg =
    graph.new()
    |> graph.add_dependency("amr-1", "amr-2", "corridor-edge-1", 5.0)
    |> graph.add_dependency("amr-2", "amr-1", "corridor-edge-1", 5.2)

  assert graph.is_deadlocked(wfg) == True
  let deadlocks = graph.find_deadlocks(wfg)
  assert list.is_empty(deadlocks) == False

  let assert Ok(first_cycle) = list.first(deadlocks)
  assert list.contains(first_cycle.cycle_agents, "amr-1") == True
  assert list.contains(first_cycle.cycle_agents, "amr-2") == True
}

pub fn wfg_three_agent_circular_deadlock_test() {
  let wfg =
    graph.new()
    |> graph.add_dependency("amr-1", "amr-2", "junction-A", 4.0)
    |> graph.add_dependency("amr-2", "amr-3", "junction-B", 6.5)
    |> graph.add_dependency("amr-3", "amr-1", "junction-C", 2.1)

  assert graph.is_deadlocked(wfg) == True
  let deadlocks = graph.find_deadlocks(wfg)
  let assert Ok(cycle) = list.first(deadlocks)

  assert list.contains(cycle.cycle_agents, "amr-1") == True
  assert list.contains(cycle.cycle_agents, "amr-2") == True
  assert list.contains(cycle.cycle_agents, "amr-3") == True
  assert cycle.longest_wait_sec >=. 6.5
}

pub fn deadlock_resolution_yield_to_buffer_test() {
  let n1 = node.new_transit("n1", Point2D(0.0, 0.0))
  let n2 = node.new_transit("n2", Point2D(5.0, 0.0))
  let n_buf = node.new_park("n_buf", Point2D(0.0, 5.0), "buf-1")

  let e1_2 = edge.new_linear("e1_2", "n1", "n2", 5.0, 1.8)
  let e1_buf = edge.new_linear("e1_buf", "n1", "n_buf", 5.0, 1.0)

  let wmap =
    map.new("MAP_DEADLOCK", "Deadlock Test Facility")
    |> map.add_node(n1)
    |> map.add_node(n2)
    |> map.add_node(n_buf)
    |> map.add_edge(e1_2)
    |> map.add_edge(e1_buf)

  let priorities =
    dict.new()
    |> dict.insert("amr-low", 10)
    |> dict.insert("amr-high", 90)

  let current_nodes =
    dict.new()
    |> dict.insert("amr-low", "n1")
    |> dict.insert("amr-high", "n2")

  let cycle =
    graph.DeadlockCycle(
      cycle_agents: ["amr-low", "amr-high"],
      longest_wait_sec: 12.0,
    )

  let assert Ok(res_table) =
    table.new()
    |> table.reserve_node("amr-high", "n2", table.TimeInterval(0.0, 10.0))

  let assert Ok(plan) =
    resolver.resolve(cycle, priorities, wmap, res_table, current_nodes)

  assert plan.yield_agent_id == "amr-low"
  assert plan.beneficiary_agent_id == "amr-high"
  assert plan.action == YieldToBuffer("n_buf")
}

pub fn deadlock_resolution_reverse_to_junction_test() {
  let n0 = node.new_transit("n0", Point2D(-5.0, 0.0))
  let n1 = node.new_transit("n1", Point2D(0.0, 0.0))
  let n2 = node.new_transit("n2", Point2D(5.0, 0.0))

  let e1_0 = edge.new_linear("e1_0", "n1", "n0", 5.0, 1.8)
  let e1_2 = edge.new_linear("e1_2", "n1", "n2", 5.0, 1.8)

  let wmap =
    map.new("MAP_DEADLOCK_2", "Deadlock Reverse Facility")
    |> map.add_node(n0)
    |> map.add_node(n1)
    |> map.add_node(n2)
    |> map.add_edge(e1_0)
    |> map.add_edge(e1_2)

  let priorities =
    dict.new()
    |> dict.insert("amr-low", 5)
    |> dict.insert("amr-high", 50)

  let current_nodes =
    dict.new()
    |> dict.insert("amr-low", "n1")
    |> dict.insert("amr-high", "n2")

  let cycle =
    graph.DeadlockCycle(
      cycle_agents: ["amr-low", "amr-high"],
      longest_wait_sec: 8.0,
    )

  let assert Ok(res_table) =
    table.new()
    |> table.reserve_node("amr-high", "n2", table.TimeInterval(0.0, 10.0))

  let assert Ok(plan) =
    resolver.resolve(cycle, priorities, wmap, res_table, current_nodes)

  assert plan.yield_agent_id == "amr-low"
  assert plan.beneficiary_agent_id == "amr-high"
  assert plan.action == ReverseToJunction("n0")
}
