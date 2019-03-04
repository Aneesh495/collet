// Native Erlang math bindings for floating point geometry and kinematics.
// All functions map directly to the BEAM :math module with zero overhead.

@external(erlang, "math", "sqrt")
pub fn sqrt(x: Float) -> Float

@external(erlang, "math", "sin")
pub fn sin(x: Float) -> Float

@external(erlang, "math", "cos")
pub fn cos(x: Float) -> Float

@external(erlang, "math", "tan")
pub fn tan(x: Float) -> Float

@external(erlang, "math", "asin")
pub fn asin(x: Float) -> Float

@external(erlang, "math", "acos")
pub fn acos(x: Float) -> Float

@external(erlang, "math", "atan2")
pub fn atan2(y: Float, x: Float) -> Float

@external(erlang, "math", "pi")
pub fn pi() -> Float

@external(erlang, "math", "pow")
pub fn pow(base: Float, exponent: Float) -> Float

@external(erlang, "math", "exp")
pub fn exp(x: Float) -> Float

@external(erlang, "math", "log")
pub fn log(x: Float) -> Float

@external(erlang, "math", "floor")
pub fn floor(x: Float) -> Float

@external(erlang, "math", "ceil")
pub fn ceil(x: Float) -> Float

// Absolute value for floating point numbers
pub fn abs_float(x: Float) -> Float {
  case x <. 0.0 {
    True -> 0.0 -. x
    False -> x
  }
}

// Minimum of two floating point numbers
pub fn min_float(a: Float, b: Float) -> Float {
  case a <. b {
    True -> a
    False -> b
  }
}

// Maximum of two floating point numbers
pub fn max_float(a: Float, b: Float) -> Float {
  case a >. b {
    True -> a
    False -> b
  }
}

// Clamp a floating point value between lower and upper bounds
pub fn clamp_float(value: Float, lower: Float, upper: Float) -> Float {
  case value <. lower {
    True -> lower
    False ->
      case value >. upper {
        True -> upper
        False -> value
      }
  }
}
