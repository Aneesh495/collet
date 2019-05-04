import collet/geometry/math_ffi

// Kinematic drive mechanism and actuator topology of the automated vehicle.
pub type DriveType {
  // Two independent powered drive wheels on a common lateral axis.
  // Capable of zero-radius in-place rotation.
  DifferentialDrive(wheel_radius_meters: Float, wheel_track_meters: Float)
  // Four mecanum or omni wheels enabling full holonomic translation in SE(2).
  OmnidirectionalMecanum(
    wheel_radius_meters: Float,
    wheel_base_meters: Float,
    wheel_track_meters: Float,
  )
  // Steered single traction wheel with passive casters (tricycle drive).
  TricycleSteer(wheel_base_meters: Float, max_steer_angle_rad: Float)
}

// Comprehensive physical and kinematic specification of an AMR / AGV model.
pub type VehicleProfile {
  VehicleProfile(
    id: String,
    name: String,
    drive_type: DriveType,
    length_meters: Float,
    width_meters: Float,
    height_meters: Float,
    curb_weight_kg: Float,
    max_payload_kg: Float,
    max_linear_velocity: Float,
    max_linear_acceleration: Float,
    max_linear_deceleration: Float,
    emergency_deceleration: Float,
    max_angular_velocity: Float,
    max_angular_acceleration: Float,
    min_turning_radius: Float,
  )
}

// Standard industrial differential drive autonomous mobile robot (AMR).
// Typical use case: automated warehouse shelf lifting and pallet staging.
pub fn standard_differential_amr(id: String, name: String) -> VehicleProfile {
  VehicleProfile(
    id: id,
    name: name,
    drive_type: DifferentialDrive(
      wheel_radius_meters: 0.1,
      wheel_track_meters: 0.6,
    ),
    length_meters: 1.0,
    width_meters: 0.8,
    height_meters: 0.35,
    curb_weight_kg: 180.0,
    max_payload_kg: 1000.0,
    max_linear_velocity: 1.8,
    max_linear_acceleration: 1.0,
    max_linear_deceleration: 1.2,
    emergency_deceleration: 3.5,
    max_angular_velocity: 1.5708,
    max_angular_acceleration: 2.0,
    min_turning_radius: 0.0,
  )
}

// Heavy-duty autonomous pallet transporter for European EUR-pallets.
pub fn heavy_pallet_mover(id: String, name: String) -> VehicleProfile {
  VehicleProfile(
    id: id,
    name: name,
    drive_type: DifferentialDrive(
      wheel_radius_meters: 0.125,
      wheel_track_meters: 0.9,
    ),
    length_meters: 1.6,
    width_meters: 1.1,
    height_meters: 0.45,
    curb_weight_kg: 450.0,
    max_payload_kg: 1500.0,
    max_linear_velocity: 1.2,
    max_linear_acceleration: 0.6,
    max_linear_deceleration: 0.8,
    emergency_deceleration: 2.5,
    max_angular_velocity: 1.0,
    max_angular_acceleration: 1.2,
    min_turning_radius: 0.0,
  )
}

// Agile, high-velocity tote carrier for order picking and shuttle sortation.
pub fn high_speed_tote_carrier(id: String, name: String) -> VehicleProfile {
  VehicleProfile(
    id: id,
    name: name,
    drive_type: OmnidirectionalMecanum(
      wheel_radius_meters: 0.075,
      wheel_base_meters: 0.5,
      wheel_track_meters: 0.5,
    ),
    length_meters: 0.75,
    width_meters: 0.55,
    height_meters: 0.3,
    curb_weight_kg: 85.0,
    max_payload_kg: 80.0,
    max_linear_velocity: 2.5,
    max_linear_acceleration: 1.8,
    max_linear_deceleration: 2.0,
    emergency_deceleration: 4.5,
    max_angular_velocity: 2.5,
    max_angular_acceleration: 3.5,
    min_turning_radius: 0.0,
  )
}

// Computes theoretical stopping distance in meters: d = v^2 / (2 * a).
pub fn stopping_distance(
  profile: VehicleProfile,
  current_velocity: Float,
  is_emergency: Bool,
) -> Float {
  let v = math_ffi.abs_float(current_velocity)
  let a_decel = case is_emergency {
    True -> profile.emergency_deceleration
    False -> profile.max_linear_deceleration
  }
  case a_decel <. 0.01 {
    True -> 999999.0
    False -> { v *. v } /. { 2.0 *. a_decel }
  }
}

// Predicate checking whether the vehicle drive architecture supports in-place rotation.
pub fn can_turn_in_place(profile: VehicleProfile) -> Bool {
  case profile.drive_type {
    DifferentialDrive(_, _) -> True
    OmnidirectionalMecanum(_, _, _) -> True
    TricycleSteer(_, _) -> False
  }
}

// Adjusts deceleration limit dynamically based on instantaneous payload mass.
// Heavier payloads reduce braking performance to prevent tip-over or cargo sliding.
pub fn payload_derated_deceleration(
  profile: VehicleProfile,
  payload_kg: Float,
) -> Float {
  let safe_payload = math_ffi.clamp_float(payload_kg, 0.0, profile.max_payload_kg)
  let payload_ratio = safe_payload /. profile.max_payload_kg
  // Derates deceleration down to a minimum of 60% at full payload capacity
  let derate_factor = 1.0 -. 0.4 *. payload_ratio
  profile.max_linear_deceleration *. derate_factor
}
