import collet/protocol/vda5050/header.{type Header}

// MQTT / TCP connection health state as mandated by VDA 5050 specification.
// Typically communicated via MQTT Last Will and Testament (LWT).
pub type ConnectionState {
  Online
  Offline
  ConnectionBroken
}

// Complete VDA 5050 connection message envelope.
pub type ConnectionMessage {
  ConnectionMessage(header: Header, connection_state: ConnectionState)
}

// Converts ConnectionState to standard VDA 5050 string.
pub fn connection_state_to_string(state: ConnectionState) -> String {
  case state {
    Online -> "ONLINE"
    Offline -> "OFFLINE"
    ConnectionBroken -> "CONNECTIONBROKEN"
  }
}

// Parses standard VDA 5050 connection state string.
pub fn string_to_connection_state(str: String) -> Result(ConnectionState, Nil) {
  case str {
    "ONLINE" -> Ok(Online)
    "OFFLINE" -> Ok(Offline)
    "CONNECTIONBROKEN" -> Ok(ConnectionBroken)
    _ -> Error(Nil)
  }
}
