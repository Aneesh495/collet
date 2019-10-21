import collet/actor/robot_session.{type RobotMessage, type VehicleTelemetry}
import collet/mission/auction.{Bidder}
import collet/mission/dispatcher.{type Dispatcher}
import collet/mission/task.{type MissionTask}
import collet/reservation/table.{type ReservationTable, type VehicleId}
import collet/topology/map.{type WarehouseMap}
import gleam/dict.{type Dict}
import gleam/erlang/process.{type Subject}
import gleam/list
import gleam/otp/actor

// Fleet management status overview telemetry.
pub type FleetStatusSnapshot {
  FleetStatusSnapshot(
    total_robots: Int,
    active_missions_count: Int,
    pending_tasks_count: Int,
    utilization_ratio: Float,
    telemetries: List(VehicleTelemetry),
  )
}

// Internal state of the centralized fleet manager actor.
pub type ManagerState {
  ManagerState(
    wmap: WarehouseMap,
    res_table: ReservationTable,
    dispatcher: Dispatcher,
    robot_sessions: Dict(VehicleId, Subject(RobotMessage)),
    robot_telemetries: Dict(VehicleId, VehicleTelemetry),
    system_time_sec: Float,
  )
}

// Messages accepted by the central fleet manager actor.
pub type FleetMessage {
  RegisterRobot(
    vehicle_id: VehicleId,
    session: Subject(RobotMessage),
    reply_to: Subject(Nil),
  )
  UnregisterRobot(vehicle_id: VehicleId, reply_to: Subject(Nil))
  SubmitMission(task: MissionTask, reply_to: Subject(String))
  UpdateRobotTelemetry(telemetry: VehicleTelemetry)
  TickDispatch(reply_to: Subject(Int))
  StepSimulation(delta_sec: Float, reply_to: Subject(Nil))
  GetFleetStatus(reply_to: Subject(FleetStatusSnapshot))
  Stop
}

// Starts the central fleet manager actor process.
pub fn start(
  wmap: WarehouseMap,
) -> Result(actor.Started(Subject(FleetMessage)), actor.StartError) {
  let initial_state =
    ManagerState(
      wmap: wmap,
      res_table: table.new(),
      dispatcher: dispatcher.new(),
      robot_sessions: dict.new(),
      robot_telemetries: dict.new(),
      system_time_sec: 0.0,
    )

  actor.new(initial_state)
  |> actor.on_message(handle_message)
  |> actor.start
}

// Handles incoming fleet management messages.
fn handle_message(
  state: ManagerState,
  message: FleetMessage,
) -> actor.Next(ManagerState, FleetMessage) {
  case message {
    RegisterRobot(vid, session, client) -> {
      let updated_sessions = dict.insert(state.robot_sessions, vid, session)
      process.send(client, Nil)
      actor.continue(ManagerState(..state, robot_sessions: updated_sessions))
    }

    UnregisterRobot(vid, client) -> {
      let updated_sessions = dict.delete(state.robot_sessions, vid)
      let updated_telems = dict.delete(state.robot_telemetries, vid)
      process.send(client, Nil)
      actor.continue(
        ManagerState(
          ..state,
          robot_sessions: updated_sessions,
          robot_telemetries: updated_telems,
        ),
      )
    }

    SubmitMission(t, client) -> {
      let tid = task.id(t)
      let updated_disp = dispatcher.submit_task(state.dispatcher, t)
      process.send(client, tid)
      actor.continue(ManagerState(..state, dispatcher: updated_disp))
    }

    UpdateRobotTelemetry(telem) -> {
      let updated_telems =
        dict.insert(state.robot_telemetries, telem.vehicle_id, telem)
      actor.continue(ManagerState(..state, robot_telemetries: updated_telems))
    }

    TickDispatch(client) -> {
      // 1. Gather bidders from active registered robot telemetries
      let bidders =
        dict.fold(state.robot_telemetries, [], fn(acc, vid, telem) {
          case telem.is_e_stopped {
            True -> acc
            False -> {
              let b =
                Bidder(
                  vehicle_id: vid,
                  current_node_id: telem.current_node_id,
                  battery_soc: telem.battery_soc,
                  is_loaded: False,
                  max_payload_kg: 500.0,
                  queued_task_count: 0,
                )
              [b, ..acc]
            }
          }
        })

      // 2. Run dispatch across pending tasks
      let #(new_disp, assignments) =
        dispatcher.dispatch_pending(
          state.dispatcher,
          bidders,
          state.wmap,
          state.system_time_sec,
        )

      let assigned_count = list.length(assignments)
      process.send(client, assigned_count)
      actor.continue(ManagerState(..state, dispatcher: new_disp))
    }

    StepSimulation(delta, client) -> {
      let new_time = state.system_time_sec +. delta
      process.send(client, Nil)
      actor.continue(ManagerState(..state, system_time_sec: new_time))
    }

    GetFleetStatus(client) -> {
      let total = dict.size(state.robot_sessions)
      let telems = dict.values(state.robot_telemetries)
      let active_count = dict.size(state.dispatcher.active_missions)
      let pending_count = list.length(state.dispatcher.pending_tasks)
      let util = dispatcher.fleet_utilization(state.dispatcher, total)

      let snapshot =
        FleetStatusSnapshot(
          total_robots: total,
          active_missions_count: active_count,
          pending_tasks_count: pending_count,
          utilization_ratio: util,
          telemetries: telems,
        )
      process.send(client, snapshot)
      actor.continue(state)
    }

    Stop -> actor.stop()
  }
}

// Client API: Registers a robot session actor with the fleet manager.
pub fn register_robot(
  manager: Subject(FleetMessage),
  vehicle_id: VehicleId,
  session: Subject(RobotMessage),
  timeout_ms: Int,
) -> Nil {
  process.call(manager, timeout_ms, fn(reply_to) {
    RegisterRobot(vehicle_id, session, reply_to)
  })
}

// Client API: Submits a new mission task to the fleet manager.
pub fn submit_task(
  manager: Subject(FleetMessage),
  task: MissionTask,
  timeout_ms: Int,
) -> String {
  process.call(manager, timeout_ms, fn(reply_to) {
    SubmitMission(task, reply_to)
  })
}

// Client API: Ticks dispatch cycle.
pub fn tick_dispatch(manager: Subject(FleetMessage), timeout_ms: Int) -> Int {
  process.call(manager, timeout_ms, TickDispatch)
}

// Client API: Queries complete fleet status snapshot.
pub fn get_fleet_status(
  manager: Subject(FleetMessage),
  timeout_ms: Int,
) -> FleetStatusSnapshot {
  process.call(manager, timeout_ms, GetFleetStatus)
}
