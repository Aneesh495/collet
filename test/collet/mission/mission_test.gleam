import collet/geometry/point.{Point2D}
import collet/mission/auction.{Bidder}
import collet/mission/dispatcher
import collet/mission/task.{Assigned, Critical, Express, InTransitToPickup, Standard}
import collet/topology/edge
import collet/topology/map
import collet/topology/node
import gleam/dict
import gleam/list

pub fn task_creation_and_lifecycle_test() {
  let t =
    task.new_transport(
      "TASK_001",
      "PICKUP_A",
      "DROPOFF_B",
      "PALLET_EUR",
      350.0,
      Express,
      120.0,
      0.0,
    )

  assert task.id(t) == "TASK_001"
  assert task.priority(t) == Express
  assert task.is_active(t) == True

  let assert Ok(assigned_t) = task.assign(t, "amr-01", 5.0)
  assert task.status(assigned_t) == Assigned("amr-01", 5.0)

  let assert Ok(transit_t) =
    task.start_transit_to_pickup(assigned_t, "amr-01", 6.0)
  assert task.status(transit_t) == InTransitToPickup("amr-01", 6.0)

  let completed_t = task.complete(transit_t, 45.0)
  assert task.is_active(completed_t) == False
}

pub fn task_sla_tardiness_test() {
  let t =
    task.new_transport(
      "TASK_URGENT",
      "P1",
      "D1",
      "BOX",
      10.0,
      Critical,
      100.0,
      0.0,
    )

  assert task.is_tardy(t, 50.0) == False
  assert task.is_tardy(t, 99.9) == False
  assert task.is_tardy(t, 100.1) == True

  let completed_early = task.complete(t, 95.0)
  assert task.is_tardy(completed_early, 120.0) == False

  let completed_late = task.complete(t, 105.0)
  assert task.is_tardy(completed_late, 120.0) == True
}

pub fn auction_bidding_and_winner_selection_test() {
  let n_a = node.new_transit("N_A", Point2D(0.0, 0.0))
  let n_b = node.new_transit("N_B", Point2D(50.0, 0.0))
  let n_p = node.new_pickup("N_P", Point2D(5.0, 0.0), "STN_1", "BOX", 0.0)
  let n_d = node.new_dropoff("N_D", Point2D(10.0, 0.0), "STN_2", ["BOX"], 0.0)

  let wmap =
    map.new("AUCTION_MAP", "Auction Facility")
    |> map.add_node(n_a)
    |> map.add_node(n_b)
    |> map.add_node(n_p)
    |> map.add_node(n_d)
    |> map.add_edge(edge.new_linear("E_AP", "N_A", "N_P", 5.0, 1.5))
    |> map.add_edge(edge.new_linear("E_BP", "N_B", "N_P", 45.0, 1.5))
    |> map.add_edge(edge.new_linear("E_PD", "N_P", "N_D", 5.0, 1.5))

  let task_item =
    task.new_transport(
      "JOB_AUCT",
      "N_P",
      "N_D",
      "BOX",
      50.0,
      Standard,
      200.0,
      0.0,
    )

  let robot_near =
    Bidder(
      vehicle_id: "robot-near",
      current_node_id: "N_A",
      battery_soc: 0.95,
      is_loaded: False,
      max_payload_kg: 500.0,
      queued_task_count: 0,
    )

  let robot_far =
    Bidder(
      vehicle_id: "robot-far",
      current_node_id: "N_B",
      battery_soc: 0.95,
      is_loaded: False,
      max_payload_kg: 500.0,
      queued_task_count: 0,
    )

  let assert Ok(win_bid) =
    auction.run_auction(task_item, [robot_near, robot_far], wmap)

  assert win_bid.vehicle_id == "robot-near"
  assert win_bid.marginal_cost <. 20.0
}

pub fn auction_rejection_criteria_test() {
  let n_curr = node.new_transit("N_C", Point2D(0.0, 0.0))
  let n_p = node.new_pickup("N_P", Point2D(10.0, 0.0), "S1", "PALLET", 0.0)
  let n_d = node.new_dropoff("N_D", Point2D(20.0, 0.0), "S2", ["PALLET"], 0.0)

  let wmap =
    map.new("MAP_REJ", "Rejection Test")
    |> map.add_node(n_curr)
    |> map.add_node(n_p)
    |> map.add_node(n_d)

  let heavy_task =
    task.new_transport(
      "HEAVY",
      "N_P",
      "N_D",
      "STEEL_COIL",
      1200.0,
      Standard,
      300.0,
      0.0,
    )

  let weak_robot =
    Bidder(
      vehicle_id: "amr-light",
      current_node_id: "N_C",
      battery_soc: 0.90,
      is_loaded: False,
      max_payload_kg: 500.0,
      queued_task_count: 0,
    )

  assert auction.compute_bid(weak_robot, heavy_task, wmap) == Error(
    "Robot amr-light payload limit exceeded for task HEAVY",
  )

  let low_battery_robot =
    Bidder(
      vehicle_id: "amr-low-bat",
      current_node_id: "N_C",
      battery_soc: 0.15,
      is_loaded: False,
      max_payload_kg: 1500.0,
      queued_task_count: 0,
    )

  assert auction.compute_bid(low_battery_robot, heavy_task, wmap) == Error(
    "Robot amr-low-bat has insufficient battery reserve (SoC < 0.20)",
  )
}

pub fn dispatcher_batch_allocation_and_utilization_test() {
  let n1 = node.new_transit("N1", Point2D(0.0, 0.0))
  let n2 = node.new_transit("N2", Point2D(10.0, 0.0))
  let n3 = node.new_transit("N3", Point2D(20.0, 0.0))

  let wmap =
    map.new("DISPATCH_FACILITY", "Dispatch Center")
    |> map.add_node(n1)
    |> map.add_node(n2)
    |> map.add_node(n3)
    |> map.add_edge(edge.new_linear("E1_2", "N1", "N2", 10.0, 2.0))
    |> map.add_edge(edge.new_linear("E2_3", "N2", "N3", 10.0, 2.0))

  let t_critical =
    task.new_transport(
      "CRIT_01",
      "N1",
      "N2",
      "URGENT_DRUG",
      20.0,
      Critical,
      50.0,
      0.0,
    )

  let t_standard =
    task.new_transport(
      "STD_02",
      "N2",
      "N3",
      "NORMAL_BOX",
      50.0,
      Standard,
      200.0,
      0.0,
    )

  let disp =
    dispatcher.new()
    |> dispatcher.submit_task(t_standard)
    |> dispatcher.submit_task(t_critical)

  let r1 =
    Bidder(
      vehicle_id: "r1",
      current_node_id: "N1",
      battery_soc: 0.9,
      is_loaded: False,
      max_payload_kg: 500.0,
      queued_task_count: 0,
    )

  let r2 =
    Bidder(
      vehicle_id: "r2",
      current_node_id: "N2",
      battery_soc: 0.85,
      is_loaded: False,
      max_payload_kg: 500.0,
      queued_task_count: 0,
    )

  let #(updated_disp, assignments) =
    dispatcher.dispatch_pending(disp, [r1, r2], wmap, 10.0)

  // Both tasks should be dispatched
  assert list.length(assignments) == 2
  assert list.is_empty(updated_disp.pending_tasks) == True
  assert dict.size(updated_disp.active_missions) == 2

  // Fleet utilization across 2 robots should be 1.0 (100%)
  assert dispatcher.fleet_utilization(updated_disp, 2) == 1.0

  // Finish critical mission
  let assert Ok(disp_after_comp) =
    dispatcher.update_status(
      updated_disp,
      "CRIT_01",
      task.Completed(completion_time: 25.0),
    )

  assert dict.size(disp_after_comp.active_missions) == 1
  assert dict.size(disp_after_comp.completed_missions) == 1
}
