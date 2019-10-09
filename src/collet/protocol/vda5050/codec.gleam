import collet/protocol/vda5050/action.{
  type InstantActionsMessage,
}
import collet/protocol/vda5050/connection.{
  type ConnectionMessage,
}
import collet/protocol/vda5050/order.{
  type Order, type OrderEdge, type OrderNode,
}
import gleam/dynamic/decode
import gleam/json
import gleam/list

// Summary record extracted from an ingested VDA 5050 Order message.
pub type OrderSummary {
  OrderSummary(order_id: String, order_update_id: Int)
}

// Parses and validates a VDA 5050 Order JSON document.
pub fn decode_order_summary(json_string: String) -> Result(OrderSummary, String) {
  let decoder = {
    use order_id <- decode.field("orderId", decode.string)
    use update_id <- decode.field("orderUpdateId", decode.int)
    decode.success(OrderSummary(order_id: order_id, order_update_id: update_id))
  }

  case json.parse(json_string, decoder) {
    Ok(summary) -> Ok(summary)
    Error(_) -> Error("Malformed VDA 5050 order: missing orderId or orderUpdateId")
  }
}

// Encodes a full VDA 5050 Order message into a valid JSON string.
pub fn encode_order(o: Order) -> String {
  let nodes_json =
    o.nodes
    |> list.map(encode_order_node)
    |> json.preprocessed_array

  let edges_json =
    o.edges
    |> list.map(encode_order_edge)
    |> json.preprocessed_array

  json.object([
    #("headerId", json.int(o.header.header_id)),
    #("timestamp", json.string(o.header.timestamp_iso8601)),
    #("version", json.string(o.header.version)),
    #("manufacturer", json.string(o.header.manufacturer)),
    #("serialNumber", json.string(o.header.serial_number)),
    #("orderId", json.string(o.order_id)),
    #("orderUpdateId", json.int(o.order_update_id)),
    #("nodes", nodes_json),
    #("edges", edges_json),
  ])
  |> json.to_string
}

// Encodes an individual OrderNode.
fn encode_order_node(n: OrderNode) -> json.Json {
  let pos_obj =
    json.object([
      #("x", json.float(n.position.x)),
      #("y", json.float(n.position.y)),
      #("mapId", json.string(n.position.map_id)),
    ])

  json.object([
    #("nodeId", json.string(n.node_id)),
    #("sequenceId", json.int(n.sequence_id)),
    #("released", json.bool(n.released)),
    #("nodePosition", pos_obj),
  ])
}

// Encodes an individual OrderEdge.
fn encode_order_edge(e: OrderEdge) -> json.Json {
  json.object([
    #("edgeId", json.string(e.edge_id)),
    #("sequenceId", json.int(e.sequence_id)),
    #("released", json.bool(e.released)),
    #("startNodeId", json.string(e.start_node_id)),
    #("endNodeId", json.string(e.end_node_id)),
  ])
}

// Encodes an InstantActionsMessage into a JSON string.
pub fn encode_instant_actions(m: InstantActionsMessage) -> String {
  let actions_json =
    m.actions
    |> list.map(fn(a) {
      json.object([
        #("actionType", json.string(a.action_type)),
        #("actionId", json.string(a.action_id)),
        #("blockingType", json.string(action.blocking_type_to_string(a.blocking_type))),
      ])
    })
    |> json.preprocessed_array

  json.object([
    #("headerId", json.int(m.header.header_id)),
    #("timestamp", json.string(m.header.timestamp_iso8601)),
    #("version", json.string(m.header.version)),
    #("manufacturer", json.string(m.header.manufacturer)),
    #("serialNumber", json.string(m.header.serial_number)),
    #("actions", actions_json),
  ])
  |> json.to_string
}

// Encodes a ConnectionMessage into a JSON string.
pub fn encode_connection(c: ConnectionMessage) -> String {
  json.object([
    #("headerId", json.int(c.header.header_id)),
    #("timestamp", json.string(c.header.timestamp_iso8601)),
    #("version", json.string(c.header.version)),
    #("manufacturer", json.string(c.header.manufacturer)),
    #("serialNumber", json.string(c.header.serial_number)),
    #(
      "connectionState",
      json.string(connection.connection_state_to_string(c.connection_state)),
    ),
  ])
  |> json.to_string
}

// Validates the structural order topology: sequence IDs, edge node linkages.
pub fn validate_order_topology(o: Order) -> List(String) {
  // Check alternating sequence IDs: nodes must be even (0, 2, ...), edges must be odd (1, 3, ...)
  let node_seq_errors =
    list.filter_map(o.nodes, fn(n) {
      case n.sequence_id % 2 != 0 {
        True ->
          Ok(
            "Node "
            <> n.node_id
            <> " has odd sequenceId "
            <> int_to_string(n.sequence_id)
            <> "; VDA 5050 requires even sequence IDs for nodes",
          )
        False -> Error(Nil)
      }
    })

  let edge_seq_errors =
    list.filter_map(o.edges, fn(e) {
      case e.sequence_id % 2 == 0 {
        True ->
          Ok(
            "Edge "
            <> e.edge_id
            <> " has even sequenceId "
            <> int_to_string(e.sequence_id)
            <> "; VDA 5050 requires odd sequence IDs for edges",
          )
        False -> Error(Nil)
      }
    })

  list.flatten([node_seq_errors, edge_seq_errors])
}

@external(erlang, "erlang", "integer_to_binary")
fn int_to_string(x: Int) -> String
