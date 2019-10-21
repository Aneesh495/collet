// Battery chemistry type influencing discharge curve and cycle degradation.
pub type BatteryChemistry {
  LithiumIronPhosphate
  NickelManganeseCobalt
  LithiumTitanate
}

// Physical battery configuration and current state-of-charge.
pub type BatteryModel {
  BatteryModel(
    chemistry: BatteryChemistry,
    nominal_voltage_v: Float,
    capacity_amp_hours: Float,
    current_charge_amp_hours: Float,
    // Internal resistance in Ohms
    internal_resistance_ohm: Float,
    // Ambient operating temperature in degrees Celsius
    ambient_temperature_c: Float,
    // Continuous base auxiliary load (LiDAR, computers, safety scanners) in Watts
    auxiliary_load_watts: Float,
  )
}

// Gravitational acceleration constant (m/s^2)
const gravity = 9.80665

// Coefficient of rolling resistance for industrial polyurethane AMR wheels on smooth epoxy flooring
const rolling_resistance_coeff = 0.018

// Creates a standard 48V 50Ah LiFePO4 battery pack used in heavy industrial AMRs.
pub fn new_lifepo4_48v_50ah() -> BatteryModel {
  BatteryModel(
    chemistry: LithiumIronPhosphate,
    nominal_voltage_v: 48.0,
    capacity_amp_hours: 50.0,
    current_charge_amp_hours: 50.0,
    internal_resistance_ohm: 0.025,
    ambient_temperature_c: 20.0,
    auxiliary_load_watts: 45.0,
  )
}

// Creates a compact 24V 30Ah NMC battery pack for light agile mobile robots.
pub fn new_nmc_24v_30ah() -> BatteryModel {
  BatteryModel(
    chemistry: NickelManganeseCobalt,
    nominal_voltage_v: 24.0,
    capacity_amp_hours: 30.0,
    current_charge_amp_hours: 30.0,
    internal_resistance_ohm: 0.035,
    ambient_temperature_c: 20.0,
    auxiliary_load_watts: 30.0,
  )
}

// Total nominal energy storage capacity in Watt-hours (Wh).
pub fn total_energy_wh(battery: BatteryModel) -> Float {
  battery.nominal_voltage_v *. battery.capacity_amp_hours
}

// Current usable energy storage in Watt-hours (Wh).
pub fn current_energy_wh(battery: BatteryModel) -> Float {
  battery.nominal_voltage_v *. battery.current_charge_amp_hours
}

// State of charge (SoC) normalized between 0.0 (empty) and 1.0 (fully charged).
pub fn state_of_charge(battery: BatteryModel) -> Float {
  case battery.capacity_amp_hours <=. 0.0 {
    True -> 0.0
    False -> {
      let soc = battery.current_charge_amp_hours /. battery.capacity_amp_hours
      case soc >. 1.0 {
        True -> 1.0
        False ->
          case soc <. 0.0 {
            True -> 0.0
            False -> soc
          }
      }
    }
  }
}

// Computes instantaneous power draw in Watts during vehicle movement.
pub fn compute_motion_power_watts(
  battery: BatteryModel,
  vehicle_mass_kg: Float,
  payload_mass_kg: Float,
  velocity_mps: Float,
  accel_mps2: Float,
) -> Float {
  let total_mass = vehicle_mass_kg +. payload_mass_kg
  let abs_vel = case velocity_mps <. 0.0 {
    True -> 0.0 -. velocity_mps
    False -> velocity_mps
  }

  // Rolling resistance force: F_rr = total_mass * g * C_rr
  let f_rr = total_mass *. gravity *. rolling_resistance_coeff

  // Inertial acceleration force: F_inertial = total_mass * accel (only positive draw during acceleration)
  let f_accel = case accel_mps2 >. 0.0 {
    True -> total_mass *. accel_mps2
    False -> 0.0
  }

  // Mechanical tractive power: P_mech = (F_rr + F_accel) * velocity
  let p_mech = { f_rr +. f_accel } *. abs_vel

  // Inverter and motor drivetrain efficiency (~85%)
  let drivetrain_efficiency = 0.85
  let p_electrical_drive = p_mech /. drivetrain_efficiency

  // Total power = electrical tractive power + auxiliary compute/sensor payload
  p_electrical_drive +. battery.auxiliary_load_watts
}

// Simulates energy discharge over a specified time duration (seconds) under a constant power load (Watts).
pub fn discharge(
  battery: BatteryModel,
  duration_sec: Float,
  load_watts: Float,
) -> BatteryModel {
  // Energy consumed in Watt-hours = (load_watts * duration_sec) / 3600.0
  let energy_consumed_wh = { load_watts *. duration_sec } /. 3600.0

  // Effective Amp-hours consumed = energy_consumed_wh / nominal_voltage
  let ah_consumed = energy_consumed_wh /. battery.nominal_voltage_v

  let new_charge = battery.current_charge_amp_hours -. ah_consumed
  let clamped_charge = case new_charge <. 0.0 {
    True -> 0.0
    False -> new_charge
  }

  BatteryModel(..battery, current_charge_amp_hours: clamped_charge)
}

// Simulates battery charging at an automated docking station with CC-CV curve emulation.
pub fn charge(
  battery: BatteryModel,
  charger_power_kw: Float,
  duration_sec: Float,
) -> BatteryModel {
  let charger_watts = charger_power_kw *. 1000.0
  let curr_soc = state_of_charge(battery)

  // CC-CV charging efficiency taper: above 80% SoC, charging power derates to protect cells
  let taper_factor = case curr_soc >. 0.8 {
    True -> { 1.0 -. curr_soc } /. 0.2
    False -> 1.0
  }
  let effective_watts = charger_watts *. taper_factor *. 0.92

  // Energy added in Watt-hours
  let energy_added_wh = { effective_watts *. duration_sec } /. 3600.0
  let ah_added = energy_added_wh /. battery.nominal_voltage_v

  let new_charge = battery.current_charge_amp_hours +. ah_added
  let clamped_charge = case new_charge >. battery.capacity_amp_hours {
    True -> battery.capacity_amp_hours
    False -> new_charge
  }

  BatteryModel(..battery, current_charge_amp_hours: clamped_charge)
}

// Predicts remaining driving range in meters based on average cruising velocity and payload.
pub fn estimate_range_meters(
  battery: BatteryModel,
  vehicle_mass_kg: Float,
  payload_mass_kg: Float,
  cruise_velocity_mps: Float,
) -> Float {
  let power_watts =
    compute_motion_power_watts(
      battery,
      vehicle_mass_kg,
      payload_mass_kg,
      cruise_velocity_mps,
      0.0,
    )
  let usable_wh = current_energy_wh(battery)
  case power_watts <=. 0.0 {
    True -> 0.0
    False -> {
      let hours_remaining = usable_wh /. power_watts
      let seconds_remaining = hours_remaining *. 3600.0
      seconds_remaining *. cruise_velocity_mps
    }
  }
}

// Predicate checking whether the battery state of charge is below the safety threshold.
pub fn is_low(battery: BatteryModel, threshold_soc: Float) -> Bool {
  state_of_charge(battery) <. threshold_soc
}
