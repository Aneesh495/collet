import collet/geometry/math_ffi

// Structural timing and kinematic summary of a velocity motion profile.
pub type ProfilePhases {
  ProfilePhases(
    duration_accel_sec: Float,
    duration_cruise_sec: Float,
    duration_decel_sec: Float,
    total_duration_sec: Float,
    peak_velocity_mps: Float,
    distance_accel_meters: Float,
    distance_cruise_meters: Float,
    distance_decel_meters: Float,
  )
}

// Instantaneous kinematic sample evaluated at a specific time offset along the profile.
pub type ProfileSample {
  ProfileSample(
    time_sec: Float,
    distance_meters: Float,
    velocity_mps: Float,
    acceleration_mps2: Float,
  )
}

// Computes a time-optimal trapezoidal (or triangular) velocity profile across a path length.
// Properly resolves distance constraints when path length is insufficient to attain v_max.
pub fn plan_trapezoidal(
  distance_meters: Float,
  v_start: Float,
  v_max: Float,
  v_end: Float,
  a_accel: Float,
  a_decel: Float,
) -> ProfilePhases {
  let safe_dist = math_ffi.max_float(0.0, distance_meters)
  let safe_a_acc = math_ffi.max_float(0.01, a_accel)
  let safe_a_dec = math_ffi.max_float(0.01, a_decel)

  // Distance required to accelerate from v_start to v_max
  let d_accel_nominal =
    { v_max *. v_max -. v_start *. v_start } /. { 2.0 *. safe_a_acc }
  // Distance required to decelerate from v_max to v_end
  let d_decel_nominal =
    { v_max *. v_max -. v_end *. v_end } /. { 2.0 *. safe_a_dec }

  let d_critical = d_accel_nominal +. d_decel_nominal

  case safe_dist >=. d_critical {
    // Standard trapezoidal profile with a constant-velocity cruise phase
    True -> {
      let d_cruise = safe_dist -. d_critical
      let t_accel = { v_max -. v_start } /. safe_a_acc
      let t_decel = { v_max -. v_end } /. safe_a_dec
      let t_cruise = case v_max <. 0.01 {
        True -> 0.0
        False -> d_cruise /. v_max
      }
      ProfilePhases(
        duration_accel_sec: t_accel,
        duration_cruise_sec: t_cruise,
        duration_decel_sec: t_decel,
        total_duration_sec: t_accel +. t_cruise +. t_decel,
        peak_velocity_mps: v_max,
        distance_accel_meters: d_accel_nominal,
        distance_cruise_meters: d_cruise,
        distance_decel_meters: d_decel_nominal,
      )
    }
    // Triangular profile: path is too short to attain v_max
    False -> {
      // Calculate reachable peak velocity:
      // v_peak^2 = (2 * a_acc * a_dec * d + a_dec * v_start^2 + a_acc * v_end^2) / (a_acc + a_dec)
      let num =
        2.0
        *. safe_a_acc
        *. safe_a_dec
        *. safe_dist
        +. safe_a_dec
        *. v_start
        *. v_start
        +. safe_a_acc
        *. v_end
        *. v_end
      let denom = safe_a_acc +. safe_a_dec
      let v_peak_sq = num /. denom
      let v_peak = math_ffi.sqrt(math_ffi.max_float(0.0, v_peak_sq))

      let t_accel = math_ffi.max_float(0.0, { v_peak -. v_start } /. safe_a_acc)
      let t_decel = math_ffi.max_float(0.0, { v_peak -. v_end } /. safe_a_dec)
      let d_accel =
        math_ffi.max_float(
          0.0,
          { v_peak *. v_peak -. v_start *. v_start } /. { 2.0 *. safe_a_acc },
        )
      let d_decel = safe_dist -. d_accel

      ProfilePhases(
        duration_accel_sec: t_accel,
        duration_cruise_sec: 0.0,
        duration_decel_sec: t_decel,
        total_duration_sec: t_accel +. t_decel,
        peak_velocity_mps: v_peak,
        distance_accel_meters: d_accel,
        distance_cruise_meters: 0.0,
        distance_decel_meters: d_decel,
      )
    }
  }
}

// Samples instantaneous distance, velocity, and acceleration at time offset t.
pub fn sample(
  profile: ProfilePhases,
  t: Float,
  v_start: Float,
  v_end: Float,
  a_accel: Float,
  a_decel: Float,
) -> ProfileSample {
  let t_clamped = math_ffi.clamp_float(t, 0.0, profile.total_duration_sec)

  case t_clamped <=. profile.duration_accel_sec {
    // 1. Acceleration phase
    True -> {
      let v = v_start +. a_accel *. t_clamped
      let s = v_start *. t_clamped +. 0.5 *. a_accel *. t_clamped *. t_clamped
      ProfileSample(
        time_sec: t_clamped,
        distance_meters: s,
        velocity_mps: v,
        acceleration_mps2: a_accel,
      )
    }
    False -> {
      let t_after_accel = t_clamped -. profile.duration_accel_sec
      case t_after_accel <=. profile.duration_cruise_sec {
        // 2. Cruise phase
        True -> {
          let s =
            profile.distance_accel_meters
            +. profile.peak_velocity_mps
            *. t_after_accel
          ProfileSample(
            time_sec: t_clamped,
            distance_meters: s,
            velocity_mps: profile.peak_velocity_mps,
            acceleration_mps2: 0.0,
          )
        }
        // 3. Deceleration phase
        False -> {
          let t_in_decel = t_after_accel -. profile.duration_cruise_sec
          let v =
            math_ffi.max_float(
              v_end,
              profile.peak_velocity_mps -. a_decel *. t_in_decel,
            )
          let s =
            profile.distance_accel_meters
            +. profile.distance_cruise_meters
            +. profile.peak_velocity_mps
            *. t_in_decel
            -. 0.5
            *. a_decel
            *. t_in_decel
            *. t_in_decel
          ProfileSample(
            time_sec: t_clamped,
            distance_meters: s,
            velocity_mps: v,
            acceleration_mps2: 0.0 -. a_decel,
          )
        }
      }
    }
  }
}
