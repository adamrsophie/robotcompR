#' robotcompR: robotics, kinematics, dynamics and motion planning in R
#'
#' robotcompR provides spatial mathematics on SO(2), SO(3), SE(2) and
#' SE(3), rigid-body tree robot models, Denavit-Hartenberg and URDF model
#' construction, forward and inverse kinematics, Jacobians, rigid-body
#' dynamics, collision checking, motion planning, trajectory generation
#' and mobile robotics.
#'
#' Conventions used throughout the package:
#'
#' * Units are SI: meters, radians, seconds, kilograms, newtons.
#' * Rotations are active and right-handed. A pose \eqn{{}^{a}T_{b}} is the
#'   pose of frame b in frame a and maps b coordinates to a coordinates.
#' * Quaternions are scalar-first `c(w, x, y, z)` with the Hamilton product.
#' * Twists, Jacobian rows and spatial vectors are ordered linear first,
#'   `c(v, w)`.
#' * Euler angles are intrinsic; `"ZYX"` gives yaw, pitch, roll.
#'
#' See `vignette("robotcompR")` and `vignette("spatial-math")` for details.
#'
#' The package is research and teaching software. It is not certified for
#' industrial, medical, automotive or any other safety-critical use, and it
#' never sends commands to physical hardware.
#'
#' @keywords internal
"_PACKAGE"
