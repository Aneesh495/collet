import gleam/int

// Operational mode chosen via command line invocation.
pub type CliMode {
  SimulateMode
  DaemonMode
  ValidateTopologyMode
  BenchmarkMode
  HelpMode
  VersionMode
}

// Configuration options parsed from command line arguments.
pub type CliConfig {
  CliConfig(
    mode: CliMode,
    map_filepath: String,
    fleet_size: Int,
    duration_sec: Float,
    time_step_sec: Float,
    api_port: Int,
    enable_cbs_solver: Bool,
  )
}

// Returns the default CLI runtime configuration.
pub fn default_config() -> CliConfig {
  CliConfig(
    mode: SimulateMode,
    map_filepath: "data/maps/warehouse_grid_5x5.json",
    fleet_size: 4,
    duration_sec: 60.0,
    time_step_sec: 0.1,
    api_port: 8080,
    enable_cbs_solver: True,
  )
}

// Parses a list of raw CLI string arguments into a typed CliConfig.
pub fn parse(args: List(String)) -> CliConfig {
  let init_cfg = default_config()
  parse_flags(args, init_cfg)
}

fn parse_flags(args: List(String), cfg: CliConfig) -> CliConfig {
  case args {
    [] -> cfg
    ["--help", ..] | ["-h", ..] -> CliConfig(..cfg, mode: HelpMode)
    ["--version", ..] | ["-v", ..] -> CliConfig(..cfg, mode: VersionMode)
    ["sim", ..rest] -> parse_flags(rest, CliConfig(..cfg, mode: SimulateMode))
    ["daemon", ..rest] -> parse_flags(rest, CliConfig(..cfg, mode: DaemonMode))
    ["validate", ..rest] ->
      parse_flags(rest, CliConfig(..cfg, mode: ValidateTopologyMode))
    ["benchmark", ..rest] ->
      parse_flags(rest, CliConfig(..cfg, mode: BenchmarkMode))

    ["--map", path, ..rest] | ["-m", path, ..rest] ->
      parse_flags(rest, CliConfig(..cfg, map_filepath: path))

    ["--fleet-size", size_str, ..rest] | ["-n", size_str, ..rest] -> {
      let size = case int.parse(size_str) {
        Ok(s) -> s
        Error(_) -> cfg.fleet_size
      }
      parse_flags(rest, CliConfig(..cfg, fleet_size: size))
    }

    ["--duration", dur_str, ..rest] | ["-d", dur_str, ..rest] -> {
      let dur = case int.parse(dur_str) {
        Ok(d) -> int.to_float(d)
        Error(_) -> cfg.duration_sec
      }
      parse_flags(rest, CliConfig(..cfg, duration_sec: dur))
    }

    ["--port", port_str, ..rest] | ["-p", port_str, ..rest] -> {
      let p = case int.parse(port_str) {
        Ok(val) -> val
        Error(_) -> cfg.api_port
      }
      parse_flags(rest, CliConfig(..cfg, api_port: p))
    }

    ["--no-cbs", ..rest] ->
      parse_flags(rest, CliConfig(..cfg, enable_cbs_solver: False))

    [_, ..rest] -> parse_flags(rest, cfg)
  }
}

// Formats CLI help message text.
pub fn help_text() -> String {
  "Collet: Autonomous Mobile Robot & AGV Fleet Orchestration Platform

USAGE:
    collet <COMMAND> [OPTIONS]

COMMANDS:
    sim         Execute multi-agent warehouse simulation
    daemon      Start production BEAM fleet coordinator & REST/Prometheus API
    validate    Perform topology sanity validation on facility JSON map
    benchmark   Run kinodynamic trajectory and CBS solver stress benchmarks
    help        Display this help information
    version     Display version information

OPTIONS:
    -m, --map <PATH>          Path to facility topology JSON floor plan
    -n, --fleet-size <INT>    Number of concurrent AMRs (default: 4)
    -d, --duration <SEC>      Simulation duration in seconds (default: 60)
    -p, --port <INT>          HTTP REST and Prometheus metrics port (default: 8080)
        --no-cbs              Disable high-order CBS solver; fall back to Space-Time A*
"
}
