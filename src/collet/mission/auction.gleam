import collet/mission/task.{type MissionTask, TransportMission}
import collet/reservation/table.{type VehicleId}
import collet/topology/map.{type WarehouseMap}
import collet/topology/node.{type NodeId}
import gleam/int
import gleam/list

// Candidate fleet robot participating in the task auction.
pub type Bidder {
  Bidder(
    vehicle_id: VehicleId,
    current_node_id: NodeId,
    battery_soc: Float,
    is_loaded: Bool,
    max_payload_kg: Float,
    queued_task_count: Int,
  )
}

// Formal competitive bid submitted by an AMR for an advertised task.
pub type Bid {
  Bid(
    vehicle_id: VehicleId,
    task_id: String,
    marginal_cost: Float,
    estimated_arrival_sec: Float,
  )
}

// Minimum battery state-of-charge required to bid on transport tasks.
const min_dispatch_soc = 0.20

// Computes the marginal cost bid for an individual AMR to service a task.
pub fn compute_bid(
  bidder: Bidder,
  task: MissionTask,
  wmap: WarehouseMap,
) -> Result(Bid, String) {
  case task {
    TransportMission(
      task_id: tid,
      pickup_node_id: pickup_id,
      dropoff_node_id: dropoff_id,
      payload: payload_spec,
      priority: _,
      deadline_sec: _,
      created_at_sec: _,
      status: _,
    ) -> {
      // 1. Check physical capability and payload capacity
      case bidder.is_loaded {
        True -> Error("Robot " <> bidder.vehicle_id <> " is currently loaded")
        False ->
          case bidder.max_payload_kg <. payload_spec.weight_kg {
            True ->
              Error(
                "Robot "
                <> bidder.vehicle_id
                <> " payload limit exceeded for task "
                <> tid,
              )
            False ->
              // 2. Check battery reserve
              case bidder.battery_soc <. min_dispatch_soc {
                True ->
                  Error(
                    "Robot "
                    <> bidder.vehicle_id
                    <> " has insufficient battery reserve ("
                    <> "SoC < 0.20)",
                  )
                False ->
                  // 3. Compute topological distances
                  case
                    map.get_node(wmap, bidder.current_node_id),
                    map.get_node(wmap, pickup_id),
                    map.get_node(wmap, dropoff_id)
                  {
                    Ok(curr_n), Ok(pick_n), Ok(drop_n) -> {
                      let dist_to_pickup = node.distance(curr_n, pick_n)
                      let dist_transport = node.distance(pick_n, drop_n)

                      // Cost model:
                      // Marginal Cost = distance_to_pickup + 0.5 * distance_transport
                      //                 + 20.0 * (1.0 - battery_soc)
                      //                 + 15.0 * queued_tasks
                      let battery_penalty = { 1.0 -. bidder.battery_soc } *. 20.0
                      let queue_penalty =
                        int.to_float(bidder.queued_task_count) *. 15.0
                      let total_cost =
                        dist_to_pickup
                        +. { dist_transport *. 0.5 }
                        +. battery_penalty
                        +. queue_penalty

                      // Assume nominal average speed of 1.5 m/s
                      let eta_sec = dist_to_pickup /. 1.5

                      Ok(
                        Bid(
                          vehicle_id: bidder.vehicle_id,
                          task_id: tid,
                          marginal_cost: total_cost,
                          estimated_arrival_sec: eta_sec,
                        ),
                      )
                    }
                    _, _, _ ->
                      Error(
                        "Topological nodes not found in map for auction evaluation",
                      )
                  }
              }
          }
      }
    }
    _ ->
      Error("Auction currently supports point-to-point TransportMission tasks")
  }
}

// Conducts a contract net auction across all eligible fleet bidders.
// Returns the winning bid with the lowest marginal cost.
pub fn run_auction(
  task: MissionTask,
  bidders: List(Bidder),
  wmap: WarehouseMap,
) -> Result(Bid, String) {
  let valid_bids =
    list.filter_map(bidders, fn(b) { compute_bid(b, task, wmap) })

  case valid_bids {
    [] ->
      Error("No eligible robots submitted a valid bid for task " <> task.id(task))
    [first, ..rest] -> {
      let winning_bid =
        list.fold(rest, first, fn(best, candidate) {
          case candidate.marginal_cost <. best.marginal_cost {
            True -> candidate
            False -> best
          }
        })
      Ok(winning_bid)
    }
  }
}

// Allocates an ordered batch of tasks to the fleet by sequentially running auctions.
pub fn auction_batch(
  tasks: List(MissionTask),
  initial_bidders: List(Bidder),
  wmap: WarehouseMap,
) -> List(#(MissionTask, Result(Bid, String))) {
  // Sort tasks by priority descending (highest priority auctioned first)
  let sorted_tasks =
    list.sort(tasks, fn(a, b) {
      let pa = task.priority_to_int(task.priority(a))
      let pb = task.priority_to_int(task.priority(b))
      int.compare(pb, pa)
    })

  let #(_, results) =
    list.fold(sorted_tasks, #(initial_bidders, []), fn(acc, t) {
      let #(current_bidders, assigned_list) = acc
      case run_auction(t, current_bidders, wmap) {
        Ok(win_bid) -> {
          // Update winning bidder's queued task count so subsequent tasks penalize them
          let updated_bidders =
            list.map(current_bidders, fn(b) {
              case b.vehicle_id == win_bid.vehicle_id {
                True ->
                  Bidder(
                    vehicle_id: b.vehicle_id,
                    current_node_id: b.current_node_id,
                    battery_soc: b.battery_soc,
                    is_loaded: b.is_loaded,
                    max_payload_kg: b.max_payload_kg,
                    queued_task_count: b.queued_task_count + 1,
                  )
                False -> b
              }
            })
          #(updated_bidders, [#(t, Ok(win_bid)), ..assigned_list])
        }
        Error(err) -> #(current_bidders, [#(t, Error(err)), ..assigned_list])
      }
    })

  list.reverse(results)
}
