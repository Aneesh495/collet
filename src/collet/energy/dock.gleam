import collet/energy/battery.{type BatteryModel}
import collet/reservation/table.{type VehicleId}
import collet/topology/map.{type WarehouseMap}
import collet/topology/node.{type NodeId}
import gleam/list

// Hardware operational status of an automated charging station.
pub type DockStatus {
  DockAvailable
  DockOccupied(vehicle_id: VehicleId, docked_at_sec: Float, target_soc: Float)
  DockFaulted(reason: String)
}

// Physical charging dock hardware descriptor.
pub type ChargingDock {
  ChargingDock(
    dock_id: String,
    node_id: NodeId,
    power_kw: Float,
    connector_type: String,
    status: DockStatus,
    queue: List(VehicleId),
  )
}

// Constructs a new available charging dock.
pub fn new(
  dock_id: String,
  node_id: NodeId,
  power_kw: Float,
  connector: String,
) -> ChargingDock {
  ChargingDock(
    dock_id: dock_id,
    node_id: node_id,
    power_kw: power_kw,
    connector_type: connector,
    status: DockAvailable,
    queue: [],
  )
}

// Docks a robot into the charging station.
pub fn dock(
  dock_station: ChargingDock,
  vehicle_id: VehicleId,
  target_soc: Float,
  now_sec: Float,
) -> Result(ChargingDock, String) {
  case dock_station.status {
    DockAvailable -> {
      // Remove vehicle from waiting queue if present
      let updated_queue =
        list.filter(dock_station.queue, fn(vid) { vid != vehicle_id })

      Ok(
        ChargingDock(
          ..dock_station,
          status: DockOccupied(
            vehicle_id: vehicle_id,
            docked_at_sec: now_sec,
            target_soc: target_soc,
          ),
          queue: updated_queue,
        ),
      )
    }
    DockOccupied(curr_vid, _, _) ->
      Error(
        "Dock "
        <> dock_station.dock_id
        <> " is currently occupied by vehicle "
        <> curr_vid,
      )
    DockFaulted(reason) ->
      Error("Dock " <> dock_station.dock_id <> " is faulted: " <> reason)
  }
}

// Undocks the current robot upon completing charge cycle or preemption.
pub fn undock(dock_station: ChargingDock) -> Result(ChargingDock, String) {
  case dock_station.status {
    DockOccupied(..) -> Ok(ChargingDock(..dock_station, status: DockAvailable))
    _ ->
      Error(
        "Dock "
        <> dock_station.dock_id
        <> " cannot undock because it is not occupied",
      )
  }
}

// Enqueues an AMR waiting for this specific charging dock.
pub fn enqueue(
  dock_station: ChargingDock,
  vehicle_id: VehicleId,
) -> ChargingDock {
  case list.contains(dock_station.queue, vehicle_id) {
    True -> dock_station
    False ->
      ChargingDock(
        ..dock_station,
        queue: list.append(dock_station.queue, [vehicle_id]),
      )
  }
}

// Estimates the time required in seconds to charge a battery to target SoC.
pub fn estimate_charge_time_sec(
  battery: BatteryModel,
  dock_power_kw: Float,
  target_soc: Float,
) -> Float {
  let curr_soc = battery.state_of_charge(battery)
  case target_soc <=. curr_soc || dock_power_kw <=. 0.0 {
    True -> 0.0
    False -> {
      let delta_soc = target_soc -. curr_soc
      let total_energy_kwh = battery.total_energy_wh(battery) /. 1000.0
      let energy_needed_kwh = total_energy_kwh *. delta_soc

      // 90% average charging efficiency
      let charging_efficiency = 0.9
      let hours_needed =
        energy_needed_kwh /. { dock_power_kw *. charging_efficiency }
      hours_needed *. 3600.0
    }
  }
}

// Selects the closest available charging dock for a vehicle based on topological distance.
pub fn find_best_dock(
  docks: List(ChargingDock),
  vehicle_node_id: NodeId,
  wmap: WarehouseMap,
) -> Result(ChargingDock, String) {
  let available_docks =
    list.filter(docks, fn(d) {
      case d.status {
        DockAvailable -> True
        _ -> False
      }
    })

  case available_docks {
    [] -> Error("No available charging docks in facility")
    [first, ..rest] -> {
      case map.get_node(wmap, vehicle_node_id) {
        Error(Nil) ->
          Error("Vehicle node " <> vehicle_node_id <> " not found in map")
        Ok(v_node) -> {
          let best =
            list.fold(rest, first, fn(candidate_best, candidate_dock) {
              case
                map.get_node(wmap, candidate_best.node_id),
                map.get_node(wmap, candidate_dock.node_id)
              {
                Ok(best_n), Ok(cand_n) -> {
                  let dist_best = node.distance(v_node, best_n)
                  let dist_cand = node.distance(v_node, cand_n)
                  case dist_cand <. dist_best {
                    True -> candidate_dock
                    False -> candidate_best
                  }
                }
                _, _ -> candidate_best
              }
            })
          Ok(best)
        }
      }
    }
  }
}
