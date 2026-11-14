# Planar mobile robots. Poses are c(x, y, theta) in the world frame with
# theta wrapped to [-pi, pi). Body-frame velocities are (v, omega): v along
# the body x axis, omega about z.

check_pose2 <- function(p, name = "pose") check_numeric_vector(p, 3, name)

#' Closed-form unicycle motion
#'
#' Integrates the unicycle model \eqn{\dot x = v\cos\theta},
#' \eqn{\dot y = v\sin\theta}, \eqn{\dot\theta = \omega} in closed form
#' for constant body velocities over `dt`: along a circular arc of radius
#' \eqn{v/\omega}, written in the chord form
#' \eqn{\Delta p = v\,dt\,\mathrm{sinc}(\omega dt/2)(\cos(\theta + \omega dt/2),
#' \sin(\theta + \omega dt/2))}, which stays accurate as \eqn{\omega \to 0}
#' (a straight line in the limit).
#' Equivalently the new pose is \eqn{X \exp(\widehat{(v, 0, \omega)}\,dt)}
#' on SE(2).
#'
#' @param pose Pose `c(x, y, theta)`.
#' @param v Forward speed (m/s).
#' @param omega Turn rate (rad/s).
#' @param dt Duration (s).
#' @return The new pose.
#' @examples
#' unicycle_step(c(0, 0, 0), v = 1, omega = pi / 2, dt = 1)
#' @export
unicycle_step <- function(pose, v, omega, dt) {
  p <- check_pose2(pose)
  v <- check_scalar(v, "v")
  w <- check_scalar(omega, "omega")
  dt <- check_scalar(dt, "dt", nonnegative = TRUE)
  m <- unicycle_motion(p, v, w, dt)
  out <- c(p[1:2] + m$delta, wrap_angle_internal(p[3] + w * dt))
  names(out) <- c("x", "y", "theta")
  out
}

# sin(x) / x and its derivative, with series near zero.
sinc_ <- function(x) if (abs(x) < 1e-4) 1 - x^2 / 6 + x^4 / 120 else sin(x) / x
dsinc_ <- function(x) {
  if (abs(x) < 1e-3) -x / 3 + x^3 / 30 else (x * cos(x) - sin(x)) / x^2
}

# Chord form of the exact arc: the displacement is v dt sinc(phi / 2) along
# the mean heading theta + phi / 2, with phi = omega dt. It is accurate for
# every omega, including omega near zero. Also returns the Jacobians of the
# new pose with respect to the pose (F) and to (v, omega) (G).
unicycle_motion <- function(p, v, w, dt) {
  phi <- w * dt
  h <- p[3] + phi / 2
  s <- sinc_(phi / 2)
  ds <- dsinc_(phi / 2)
  D <- v * dt
  ch <- cos(h)
  sh <- sin(h)
  delta <- D * s * c(ch, sh)
  F <- diag(3)
  F[1:2, 3] <- D * s * c(-sh, ch)
  dphi <- D * (ds / 2 * c(ch, sh) + s / 2 * c(-sh, ch))
  G <- cbind(c(dt * s * ch, dt * s * sh, 0), c(dphi * dt, dt))
  list(delta = delta, F = F, G = G)
}

#' Mobile robot models
#'
#' `robot_diffdrive()` describes a differential-drive base with wheel
#' radius \eqn{r} and track width \eqn{b} (distance between wheels):
#' \eqn{v = r(\omega_R + \omega_L)/2}, \eqn{\omega = r(\omega_R -
#' \omega_L)/b}. `robot_bicycle()` describes a kinematic bicycle (or
#' Ackermann car) with wheelbase \eqn{L}, referenced at the rear axle:
#' \eqn{\omega = v \tan\delta / L}. `robot_unicycle()` is the plain
#' unicycle. Optional limits are checked by [mobile_simulate()].
#'
#' @param wheel_radius,track_width Differential-drive geometry (m).
#' @param wheelbase Bicycle wheelbase (m).
#' @param max_steer Maximum steering angle (rad).
#' @param max_speed,max_turn_rate Optional limits.
#' @return A `robot_mobile` object.
#' @examples
#' dd <- robot_diffdrive(wheel_radius = 0.05, track_width = 0.3)
#' diffdrive_forward(dd, 10, 12)
#' @name mobile_models
NULL

#' @rdname mobile_models
#' @export
robot_unicycle <- function(max_speed = Inf, max_turn_rate = Inf) {
  structure(list(type = "unicycle", max_speed = max_speed,
                 max_turn_rate = max_turn_rate), class = "robot_mobile")
}

#' @rdname mobile_models
#' @export
robot_diffdrive <- function(wheel_radius = 0.05, track_width = 0.3,
                            max_speed = Inf, max_turn_rate = Inf) {
  structure(list(type = "diffdrive",
                 wheel_radius = check_scalar(wheel_radius, "wheel_radius",
                                             positive = TRUE),
                 track_width = check_scalar(track_width, "track_width",
                                            positive = TRUE),
                 max_speed = max_speed, max_turn_rate = max_turn_rate),
            class = "robot_mobile")
}

#' @rdname mobile_models
#' @export
robot_bicycle <- function(wheelbase = 2.5, max_steer = pi / 4,
                          max_speed = Inf) {
  structure(list(type = "bicycle",
                 wheelbase = check_scalar(wheelbase, "wheelbase", positive = TRUE),
                 max_steer = check_scalar(max_steer, "max_steer", positive = TRUE),
                 max_speed = max_speed, max_turn_rate = Inf),
            class = "robot_mobile")
}

#' @export
print.robot_mobile <- function(x, ...) {
  desc <- switch(x$type,
    unicycle = "unicycle",
    diffdrive = sprintf("differential drive, wheel radius %g m, track %g m",
                        x$wheel_radius, x$track_width),
    bicycle = sprintf("kinematic bicycle, wheelbase %g m, max steer %g rad",
                      x$wheelbase, x$max_steer))
  cat("<robot_mobile>", desc, "\n")
  invisible(x)
}

#' Differential-drive kinematics and odometry
#'
#' `diffdrive_forward()` converts wheel angular speeds (rad/s) into body
#' velocities `c(v, omega)`; `diffdrive_inverse()` does the reverse.
#' `diffdrive_odometry()` integrates wheel rotation increments (radians,
#' for example encoder ticks times radians per tick), treating each
#' increment as a constant-velocity arc, which is exact when the wheel
#' speeds are constant over each interval.
#'
#' @param model A differential-drive `robot_mobile`.
#' @param omega_left,omega_right Wheel angular speeds (rad/s).
#' @param v,omega Body velocities.
#' @param pose Initial pose.
#' @param delta_left,delta_right Vectors of wheel rotation increments.
#' @return A named vector, or a matrix of poses (first row is `pose`).
#' @examples
#' dd <- robot_diffdrive(0.05, 0.3)
#' diffdrive_inverse(dd, 0.5, 1)
#' diffdrive_odometry(dd, c(0, 0, 0), rep(1, 4), rep(1.2, 4))
#' @name diffdrive
NULL

#' @rdname diffdrive
#' @export
diffdrive_forward <- function(model, omega_left, omega_right) {
  stopifnot(inherits(model, "robot_mobile"), model$type == "diffdrive")
  r <- model$wheel_radius
  c(v = r * (omega_right + omega_left) / 2,
    omega = r * (omega_right - omega_left) / model$track_width)
}

#' @rdname diffdrive
#' @export
diffdrive_inverse <- function(model, v, omega) {
  stopifnot(inherits(model, "robot_mobile"), model$type == "diffdrive")
  r <- model$wheel_radius
  b <- model$track_width
  c(left = (v - omega * b / 2) / r, right = (v + omega * b / 2) / r)
}

#' @rdname diffdrive
#' @export
diffdrive_odometry <- function(model, pose, delta_left, delta_right) {
  stopifnot(inherits(model, "robot_mobile"), model$type == "diffdrive")
  p <- check_pose2(pose)
  dl <- check_numeric_vector(delta_left, name = "delta_left")
  dr <- check_numeric_vector(delta_right, length(dl), "delta_right")
  out <- matrix(0, length(dl) + 1, 3, dimnames = list(NULL, c("x", "y", "theta")))
  out[1, ] <- p
  for (i in seq_along(dl)) {
    vw <- diffdrive_forward(model, dl[i], dr[i])
    out[i + 1, ] <- unicycle_step(out[i, ], vw[["v"]], vw[["omega"]], 1)
  }
  out
}

#' Simulate a mobile robot
#'
#' Applies piecewise-constant controls with the closed-form arc of
#' [unicycle_step()]. For unicycles and
#' differential drives the controls are `c(v, omega)` per row; for
#' bicycles `c(v, steer)`, converted to \eqn{\omega = v\tan\delta/L}.
#' Controls outside the model limits are an error unless `saturate =
#' TRUE`, in which case they are clipped.
#'
#' @param model A `robot_mobile`.
#' @param pose Initial pose `c(x, y, theta)`.
#' @param controls Matrix with one control per row.
#' @param dt Step duration (scalar or one per row).
#' @param saturate Clip controls to the limits.
#' @return A matrix of poses with columns `time`, `x`, `y`, `theta`.
#' @examples
#' sim <- mobile_simulate(robot_unicycle(), c(0, 0, 0),
#'                        cbind(1, rep(pi / 2, 4)), dt = 1)
#' round(sim, 6)
#' @export
mobile_simulate <- function(model, pose, controls, dt, saturate = FALSE) {
  stopifnot(inherits(model, "robot_mobile"))
  p <- check_pose2(pose)
  U <- check_matrix(controls, NULL, 2, "controls")
  dt <- rep_len(check_numeric_vector(dt, name = "dt"), nrow(U))
  out <- matrix(0, nrow(U) + 1, 4, dimnames = list(NULL, c("time", "x", "y",
                                                            "theta")))
  out[1, ] <- c(0, p)
  for (i in seq_len(nrow(U))) {
    u <- U[i, ]
    # Limits are checked on the commanded quantities before conversion, so
    # clipping the speed of a bicycle keeps its steering geometry.
    if (abs(u[1]) > model$max_speed) {
      if (!saturate) stop(sprintf("Speed %g exceeds max_speed at step %d.",
                                  u[1], i), call. = FALSE)
      u[1] <- sign(u[1]) * model$max_speed
    }
    if (model$type == "bicycle") {
      if (abs(u[2]) > model$max_steer) {
        if (!saturate) stop(sprintf("Steering %g exceeds max_steer at step %d.",
                                    u[2], i), call. = FALSE)
        u[2] <- sign(u[2]) * model$max_steer
      }
      u <- c(u[1], u[1] * tan(u[2]) / model$wheelbase)
    } else if (abs(u[2]) > model$max_turn_rate) {
      if (!saturate) stop(sprintf("Turn rate %g exceeds max_turn_rate at step %d.",
                                  u[2], i), call. = FALSE)
      u[2] <- sign(u[2]) * model$max_turn_rate
    }
    out[i + 1, ] <- c(out[i, 1] + dt[i],
                      unicycle_step(out[i, 2:4], u[1], u[2], dt[i]))
  }
  out
}

#' Convert between body and world velocities
#'
#' For a planar robot at heading \eqn{\theta}, the world velocity of the
#' body origin is \eqn{(v\cos\theta, v\sin\theta)} for body velocity
#' \eqn{(v_x^b, v_y^b) = (v, 0)}; in general
#' \eqn{v^w = R(\theta) v^b}.
#'
#' @param velocity Length-2 velocity.
#' @param theta Heading (rad).
#' @return Length-2 velocity in the other frame.
#' @examples
#' body_to_world_velocity(c(1, 0), pi / 2)
#' @name planar_velocity
NULL

#' @rdname planar_velocity
#' @export
body_to_world_velocity <- function(velocity, theta) {
  as.numeric(so2_exp(theta) %*% check_numeric_vector(velocity, 2, "velocity"))
}

#' @rdname planar_velocity
#' @export
world_to_body_velocity <- function(velocity, theta) {
  as.numeric(t(so2_exp(theta)) %*% check_numeric_vector(velocity, 2,
                                                        "velocity"))
}

#' Pure pursuit path following
#'
#' Computes a unicycle command that steers toward the lookahead point: the
#' point of the polyline `path` an arclength `lookahead` beyond the
#' closest point to the robot. The commanded curvature is
#' \eqn{\kappa = 2 y_L / \ell^2}, the circle through the robot and the
#' target, where \eqn{(x_L, y_L)} is the target in the body frame and
#' \eqn{\ell^2 = x_L^2 + y_L^2} its squared distance (Coulter, 1992). On a
#' straight path \eqn{\ell} is close to `lookahead`, but it is shorter when
#' the robot is off the path. Near the end
#' of the path the last point is used as the goal.
#'
#' @param pose Current pose.
#' @param path Matrix of 2D waypoints (one per row).
#' @param lookahead Lookahead distance \eqn{L_d} (m).
#' @param speed Forward speed (m/s).
#' @return A list with `v`, `omega`, the `target` point and `done`
#'   (within `lookahead / 4` of the final point).
#' @examples
#' path <- cbind(seq(0, 5, 0.5), 1)
#' pure_pursuit(c(0, 0, 0), path, lookahead = 1, speed = 0.5)$omega
#' @export
pure_pursuit <- function(pose, path, lookahead, speed) {
  p <- check_pose2(pose)
  P <- check_matrix(path, NULL, 2, "path")
  Ld <- check_scalar(lookahead, "lookahead", positive = TRUE)
  speed <- check_scalar(speed, "speed")
  if (nrow(P) < 1 || anyNA(P)) {
    stop("`path` needs at least one finite waypoint.", call. = FALSE)
  }
  goal <- P[nrow(P), ]
  if (vnorm(goal - p[1:2]) < Ld / 4) {
    return(list(v = 0, omega = 0, target = goal, done = TRUE))
  }
  # Closest point and arclength along the path.
  seg_len <- if (nrow(P) > 1) sqrt(rowSums(diff(P)^2)) else 0
  s_cum <- c(0, cumsum(seg_len))
  best <- c(Inf, 0)
  for (i in seq_len(max(1, nrow(P) - 1))) {
    a <- P[i, ]
    b <- P[min(i + 1, nrow(P)), ]
    ab <- b - a
    t <- if (sum(ab^2) > 0) max(0, min(1, sum((p[1:2] - a) * ab) / sum(ab^2))) else 0
    d <- vnorm(a + t * ab - p[1:2])
    if (d < best[1]) best <- c(d, s_cum[i] + t * seg_len[min(i, length(seg_len))])
  }
  s_target <- best[2] + Ld
  target <- if (s_target >= max(s_cum)) goal else {
    i <- findInterval(s_target, s_cum)
    f <- (s_target - s_cum[i]) / seg_len[i]
    P[i, ] + f * (P[i + 1, ] - P[i, ])
  }
  local <- world_to_body_velocity(target - p[1:2], p[3])
  L2 <- sum(local^2)
  kappa <- if (L2 > 0) 2 * local[2] / L2 else 0
  list(v = speed, omega = speed * kappa, target = target, done = FALSE)
}
