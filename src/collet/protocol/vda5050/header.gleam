import gleam/dynamic/decode
import gleam/json

// Standard VDA 5050 message envelope header included in all protocol interactions.
// Strictly conforms to the German VDA 5050 v2.0 / v2.1 standard specification.
pub type Header {
  Header(
    header_id: Int,
    timestamp_iso8601: String,
    version: String,
    manufacturer: String,
    serial_number: String,
  )
}

// Constructs a standard VDA 5050 header with defaults.
pub fn new(
  header_id: Int,
  timestamp_iso8601: String,
  manufacturer: String,
  serial_number: String,
) -> Header {
  Header(
    header_id: header_id,
    timestamp_iso8601: timestamp_iso8601,
    version: "2.0.0",
    manufacturer: manufacturer,
    serial_number: serial_number,
  )
}

// Serializes the VDA 5050 header to a JSON object.
pub fn encode(h: Header) -> json.Json {
  json.object([
    #("headerId", json.int(h.header_id)),
    #("timestamp", json.string(h.timestamp_iso8601)),
    #("version", json.string(h.version)),
    #("manufacturer", json.string(h.manufacturer)),
    #("serialNumber", json.string(h.serial_number)),
  ])
}

// Gleam dynamic decoder for parsing VDA 5050 headers from raw JSON payloads.
pub fn decoder() -> decode.Decoder(Header) {
  use header_id <- decode.field("headerId", decode.int)
  use timestamp <- decode.field("timestamp", decode.string)
  use version <- decode.field("version", decode.string)
  use manufacturer <- decode.field("manufacturer", decode.string)
  use serial_number <- decode.field("serialNumber", decode.string)
  decode.success(Header(
    header_id: header_id,
    timestamp_iso8601: timestamp,
    version: version,
    manufacturer: manufacturer,
    serial_number: serial_number,
  ))
}
