import collet/actor/fleet_manager
import collet/topology/map.{type WarehouseMap}
import gleam/erlang/process.{type Subject}
import gleam/otp/actor
import gleam/otp/static_supervisor.{type Supervisor}
import gleam/otp/supervision

// Starts the top-level OTP supervisor managing the fleet management system.
pub fn start_fleet_system(
  wmap: WarehouseMap,
) -> Result(actor.Started(Supervisor), actor.StartError) {
  let fleet_manager_spec =
    supervision.worker(fn() { fleet_manager.start(wmap) })

  static_supervisor.new(static_supervisor.OneForOne)
  |> static_supervisor.add(fleet_manager_spec)
  |> static_supervisor.start
}

// Builds a child specification for the fleet manager under an OTP supervisor.
pub fn make_fleet_worker(
  wmap: WarehouseMap,
) -> supervision.ChildSpecification(Subject(fleet_manager.FleetMessage)) {
  supervision.worker(fn() { fleet_manager.start(wmap) })
}
