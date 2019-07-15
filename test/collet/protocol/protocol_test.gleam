import collet/protocol/vda5050/action.{InstantActionsMessage}
import collet/protocol/vda5050/codec
import collet/protocol/vda5050/connection.{ConnectionBroken, ConnectionMessage, Online}
import collet/protocol/vda5050/header
import collet/protocol/vda5050/order.{Order}
import collet/protocol/vda5050/state.{Automatic, Manual}
import gleam/list
import gleam/option.{None, Some}
import gleam/string

pub fn header_encoding_test() {
  let h = header.new(101, "2026-11-16T15:30:00Z", "KUKA_AMR", "SERIAL_998877")
  assert h.header_id == 101
  assert h.manufacturer == "KUKA_AMR"
  assert h.version == "2.0.0"

  let _json_obj = header.encode(h)
  let json_str = codec.encode_connection(ConnectionMessage(h, Online))
  assert string.contains(json_str, "101")
  assert string.contains(json_str, "KUKA_AMR")
  assert string.contains(json_str, "ONLINE")
}

pub fn order_encoding_and_validation_test() {
  let h = header.new(1, "2026-11-16T15:30:00Z", "KUKA_AMR", "SERIAL_998877")
  let node_0 = order.new_released_node("N0", 0, 0.0, 0.0, Some(0.0), "MAP_1")
  let edge_1 = order.new_released_edge("E1", 1, "N0", "N2", 1.5)
  let node_2 = order.new_released_node("N2", 2, 5.0, 0.0, Some(0.0), "MAP_1")

  let ord =
    Order(
      header: h,
      order_id: "ORD_ALPHA_001",
      order_update_id: 0,
      zone_set_id: None,
      nodes: [node_0, node_2],
      edges: [edge_1],
    )

  // Validation passes on valid alternating sequence IDs (0, 1, 2)
  let errors = codec.validate_order_topology(ord)
  assert list.is_empty(errors)

  // Encoding produces valid VDA 5050 JSON payload
  let json_str = codec.encode_order(ord)
  assert string.contains(json_str, "ORD_ALPHA_001")
  assert string.contains(json_str, "N0")
  assert string.contains(json_str, "N2")
  assert string.contains(json_str, "E1")
  assert string.contains(json_str, "nodePosition")

  // Invalid sequence IDs are flagged
  let bad_node = order.new_released_node("N_BAD", 3, 10.0, 0.0, None, "MAP_1")
  let bad_order = Order(..ord, nodes: [bad_node])
  let bad_errors = codec.validate_order_topology(bad_order)
  assert list.length(bad_errors) == 1
}

pub fn instant_actions_encoding_test() {
  let h = header.new(42, "2026-11-16T15:35:00Z", "KUKA_AMR", "SERIAL_998877")
  let estop = action.new_emergency_stop("ACT_ESTOP_1")
  let cancel = action.new_cancel_order("ACT_CANCEL_1", "ORD_ALPHA_001")

  let msg = InstantActionsMessage(header: h, actions: [estop, cancel])
  let json_str = codec.encode_instant_actions(msg)

  assert string.contains(json_str, "emergencyStop")
  assert string.contains(json_str, "cancelOrder")
  assert string.contains(json_str, "HARD")
  assert string.contains(json_str, "ACT_ESTOP_1")
}

pub fn connection_and_state_enums_test() {
  assert connection.connection_state_to_string(Online) == "ONLINE"
  assert connection.connection_state_to_string(ConnectionBroken) == "CONNECTIONBROKEN"
  assert connection.string_to_connection_state("ONLINE") == Ok(Online)

  assert action.status_to_string(action.Waiting) == "WAITING"
  assert action.status_to_string(action.Running) == "RUNNING"
  assert action.status_to_string(action.Finished) == "FINISHED"

  assert state.operating_mode_to_string(Automatic) == "AUTOMATIC"
  assert state.operating_mode_to_string(Manual) == "MANUAL"
}
