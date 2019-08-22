import collet/mission/auction.{type Bidder}
import collet/mission/task.{
  type MissionTask, type TaskStatus, Assigned, Cancelled, Completed, Failed,
}
import collet/reservation/table.{type VehicleId}
import collet/topology/map.{type WarehouseMap}
import gleam/dict.{type Dict}
import gleam/int
import gleam/list
import gleam/option.{None, Some}

// Global mission dispatcher maintaining task queues and allocation lifecycle.
pub type Dispatcher {
  Dispatcher(
    pending_tasks: List(MissionTask),
    active_missions: Dict(String, MissionTask),
    completed_missions: Dict(String, MissionTask),
    robot_queues: Dict(VehicleId, List(String)),
  )
}

// Initializes a fresh, empty fleet mission dispatcher.
pub fn new() -> Dispatcher {
  Dispatcher(
    pending_tasks: [],
    active_missions: dict.new(),
    completed_missions: dict.new(),
    robot_queues: dict.new(),
  )
}

// Enqueues a new pending mission task for auction allocation.
pub fn submit_task(disp: Dispatcher, task: MissionTask) -> Dispatcher {
  Dispatcher(
    ..disp,
    pending_tasks: list.append(disp.pending_tasks, [task]),
  )
}

// Dispatches pending tasks to available AMRs using market auctioning.
// Returns the updated dispatcher and list of new assignments.
pub fn dispatch_pending(
  disp: Dispatcher,
  bidders: List(Bidder),
  wmap: WarehouseMap,
  now_sec: Float,
) -> #(Dispatcher, List(#(VehicleId, MissionTask))) {
  case disp.pending_tasks {
    [] -> #(disp, [])
    tasks -> {
      let auction_results = auction.auction_batch(tasks, bidders, wmap)

      list.fold(
        auction_results,
        #(disp, []),
        fn(acc, pair) {
          let #(curr_disp, assigned_acc) = acc
          let #(t, bid_res) = pair

          case bid_res {
            Ok(bid) -> {
              let tid = task.id(t)
              case task.assign(t, bid.vehicle_id, now_sec) {
                Ok(assigned_task) -> {
                  // Remove from pending_tasks
                  let remaining_pending =
                    list.filter(curr_disp.pending_tasks, fn(item) {
                      task.id(item) != tid
                    })

                  // Add to active_missions
                  let new_active =
                    dict.insert(curr_disp.active_missions, tid, assigned_task)

                  // Append to robot task queue
                  let curr_robot_q = case
                    dict.get(curr_disp.robot_queues, bid.vehicle_id)
                  {
                    Ok(q) -> q
                    Error(Nil) -> []
                  }
                  let new_robot_queues =
                    dict.insert(
                      curr_disp.robot_queues,
                      bid.vehicle_id,
                      list.append(curr_robot_q, [tid]),
                    )

                  let updated_disp =
                    Dispatcher(
                      pending_tasks: remaining_pending,
                      active_missions: new_active,
                      completed_missions: curr_disp.completed_missions,
                      robot_queues: new_robot_queues,
                    )

                  #(
                    updated_disp,
                    [#(bid.vehicle_id, assigned_task), ..assigned_acc],
                  )
                }
                Error(_) -> acc
              }
            }
            Error(_) -> acc
          }
        },
      )
    }
  }
}

// Updates an active mission status (e.g. from telemetry feedback).
pub fn update_status(
  disp: Dispatcher,
  task_id: String,
  new_status: TaskStatus,
) -> Result(Dispatcher, String) {
  case dict.get(disp.active_missions, task_id) {
    Error(Nil) -> Error("Task " <> task_id <> " not found in active missions")
    Ok(active_task) -> {
      case new_status {
        Completed(_) | Failed(_) | Cancelled -> {
          // Terminal state: remove from active and robot queue, move to completed
          let updated_active = dict.delete(disp.active_missions, task_id)
          let finished_task = case new_status {
            Completed(t) -> task.complete(active_task, t)
            Failed(r) -> task.fail(active_task, r)
            _ -> task.cancel(active_task)
          }
          let updated_completed =
            dict.insert(disp.completed_missions, task_id, finished_task)

          // Clean up robot queue
          let updated_queues = case task.assigned_vehicle(active_task) {
            Some(vid) -> {
              case dict.get(disp.robot_queues, vid) {
                Ok(q) -> {
                  let cleaned = list.filter(q, fn(id) { id != task_id })
                  dict.insert(disp.robot_queues, vid, cleaned)
                }
                Error(Nil) -> disp.robot_queues
              }
            }
            None -> disp.robot_queues
          }

          Ok(
            Dispatcher(
              pending_tasks: disp.pending_tasks,
              active_missions: updated_active,
              completed_missions: updated_completed,
              robot_queues: updated_queues,
            ),
          )
        }
        Assigned(vid, at) -> {
          case task.assign(active_task, vid, at) {
            Ok(updated) -> {
              let updated_active =
                dict.insert(disp.active_missions, task_id, updated)
              Ok(Dispatcher(..disp, active_missions: updated_active))
            }
            Error(err) -> Error(err)
          }
        }
        _ -> {
          // In transit or loading updates
          case task.start_transit_to_pickup(active_task, "ROBOT", 0.0) {
            Ok(updated) -> {
              let updated_active =
                dict.insert(disp.active_missions, task_id, updated)
              Ok(Dispatcher(..disp, active_missions: updated_active))
            }
            Error(_) -> Ok(disp)
          }
        }
      }
    }
  }
}

// Cancels a task whether pending or active.
pub fn cancel_task(disp: Dispatcher, task_id: String) -> Dispatcher {
  let in_pending =
    list.any(disp.pending_tasks, fn(t) { task.id(t) == task_id })
  case in_pending {
    True -> {
      let filtered =
        list.filter(disp.pending_tasks, fn(t) { task.id(t) != task_id })
      Dispatcher(..disp, pending_tasks: filtered)
    }
    False -> {
      case update_status(disp, task_id, Cancelled) {
        Ok(updated) -> updated
        Error(_) -> disp
      }
    }
  }
}

// Computes fleet allocation utilization ratio (active busy robots / total robots).
pub fn fleet_utilization(disp: Dispatcher, total_fleet_size: Int) -> Float {
  case total_fleet_size <= 0 {
    True -> 0.0
    False -> {
      let active_robot_count =
        dict.fold(disp.robot_queues, 0, fn(acc, _vid, q) {
          case list.is_empty(q) {
            True -> acc
            False -> acc + 1
          }
        })
      int.to_float(active_robot_count) /. int.to_float(total_fleet_size)
    }
  }
}

// Returns all active missions that have exceeded their SLA deadline.
pub fn get_tardy_missions(
  disp: Dispatcher,
  now_sec: Float,
) -> List(MissionTask) {
  let all_active = dict.values(disp.active_missions)
  list.filter(all_active, fn(t) { task.is_tardy(t, now_sec) })
}
