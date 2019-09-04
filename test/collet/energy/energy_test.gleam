import collet/energy/battery
import collet/energy/dock.{DockAvailable, DockOccupied}
import collet/geometry/point.{Point2D}
import collet/topology/map
import collet/topology/node

pub fn battery_initialization_and_capacity_test() {
  let bat = battery.new_lifepo4_48v_50ah()

  assert battery.total_energy_wh(bat) == 2400.0
  assert battery.current_energy_wh(bat) == 2400.0
  assert battery.state_of_charge(bat) == 1.0
  assert battery.is_low(bat, 0.20) == False
}

pub fn battery_tractive_power_and_discharge_test() {
  let bat = battery.new_lifepo4_48v_50ah()

  // 200 kg robot + 300 kg payload at 1.5 m/s
  let power =
    battery.compute_motion_power_watts(bat, 200.0, 300.0, 1.5, 0.0)

  // Power should exceed auxiliary 45W load
  assert power >. 45.0
  assert power <. 300.0

  // Discharge at 600 Watts for 1 hour (3600 seconds)
  // 600 Wh consumed from 2400 Wh capacity -> 1800 Wh remaining (SoC 0.75)
  let discharged = battery.discharge(bat, 3600.0, 600.0)

  assert battery.current_energy_wh(discharged) == 1800.0
  assert battery.state_of_charge(discharged) == 0.75
}

pub fn battery_cc_cv_charging_simulation_test() {
  let bat = battery.new_lifepo4_48v_50ah()
  // Discharge 1800 Wh -> remaining 600 Wh (SoC 0.25)
  let low_bat = battery.discharge(bat, 3600.0, 1800.0)
  assert battery.state_of_charge(low_bat) == 0.25

  // Charge at 10 kW for 300 seconds (5 minutes)
  let charged = battery.charge(low_bat, 10.0, 300.0)
  assert battery.state_of_charge(charged) >. 0.25
  assert battery.state_of_charge(charged) <. 1.0
}

pub fn charging_dock_lifecycle_and_queue_test() {
  let d = dock.new("DOCK_A", "NODE_CHG_1", 22.0, "INDUCTION")
  assert d.status == DockAvailable

  // Dock AMR-01
  let assert Ok(docked) = dock.dock(d, "amr-01", 0.90, 100.0)
  assert docked.status == DockOccupied("amr-01", 100.0, 0.90)

  // Second AMR cannot dock concurrently
  assert dock.dock(docked, "amr-02", 0.90, 110.0) == Error(
    "Dock DOCK_A is currently occupied by vehicle amr-01",
  )

  // Enqueue AMR-02
  let enqueued = dock.enqueue(docked, "amr-02")
  assert enqueued.queue == ["amr-02"]

  // Undock AMR-01
  let assert Ok(freed) = dock.undock(enqueued)
  assert freed.status == DockAvailable
}

pub fn dock_selection_and_time_estimation_test() {
  let n_robot = node.new_transit("N_ROBOT", Point2D(0.0, 0.0))
  let n_close = node.new_charger("N_CHG_CLOSE", Point2D(10.0, 0.0), "D1", 22.0, "PAD", 0.0)
  let n_far = node.new_charger("N_CHG_FAR", Point2D(100.0, 0.0), "D2", 22.0, "PAD", 0.0)

  let wmap =
    map.new("MAP_ENERGY", "Energy Map")
    |> map.add_node(n_robot)
    |> map.add_node(n_close)
    |> map.add_node(n_far)

  let dock_close = dock.new("DOCK_1", "N_CHG_CLOSE", 22.0, "PAD")
  let dock_far = dock.new("DOCK_2", "N_CHG_FAR", 22.0, "PAD")

  let assert Ok(best_dock) =
    dock.find_best_dock([dock_close, dock_far], "N_ROBOT", wmap)

  assert best_dock.dock_id == "DOCK_1"

  let bat = battery.new_lifepo4_48v_50ah()
  let low_bat = battery.discharge(bat, 3600.0, 1200.0) // SoC 0.50

  let estimated_time =
    dock.estimate_charge_time_sec(low_bat, 22.0, 0.90)

  // Charging 40% of 2.4 kWh = 0.96 kWh at ~20 kW net takes ~170-195 seconds
  assert estimated_time >. 150.0
  assert estimated_time <. 250.0
}
