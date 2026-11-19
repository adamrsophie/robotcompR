# Time-parameterized trajectories. A geometric path (a sequence of
# configurations) becomes a trajectory only after time parameterization.

new_trajectory <- function(time, q, qd, qdd, names = NULL, type = "",
                           evaluate = NULL) {
  q <- as.matrix(q)
  qd <- as.matrix(qd)
  qdd <- as.matrix(qdd)
  if (!is.null(names)) {
    colnames(q) <- colnames(qd) <- colnames(qdd) <- names
  }
  structure(list(time = as.numeric(time), q = q, qd = qd, qdd = qdd,
                 type = type, evaluate = evaluate),
            class = "robot_trajectory")
}

#' @export
print.robot_trajectory <- function(x, ...) {
  cat(sprintf("<robot_trajectory> %s: %d samples of %d coordinates over [%g, %g] s\n",
              x$type, length(x$time), ncol(x$q), min(x$time), max(x$time)))
  vmax <- apply(abs(x$qd), 2, max)
  cat("peak |velocity|:", paste(signif(vmax, 4), collapse = ", "), "\n")
  invisible(x)
}

#' @export
plot.robot_trajectory <- function(x, what = c("q", "qd", "qdd"), ...) {
  what <- match.arg(what)
  Y <- x[[what]]
  lab <- c(q = "position", qd = "velocity", qdd = "acceleration")[what]
  graphics::matplot(x$time, Y, type = "l", lty = 1, lwd = 2,
                    col = grDevices::hcl.colors(max(2, ncol(Y)), "Dark 3"),
                    xlab = "time (s)", ylab = lab, main = x$type, ...)
  if (!is.null(colnames(Y))) {
    graphics::legend("topright", colnames(Y), lty = 1, lwd = 2, bty = "n",
                     col = grDevices::hcl.colors(max(2, ncol(Y)), "Dark 3"))
  }
  invisible(x)
}

time_grid <- function(duration, times, n) {
  if (!is.null(times)) {
    times <- check_numeric_vector(times, name = "times")
    if (any(diff(times) < 0)) stop("`times` must be nondecreasing.",
                                   call. = FALSE)
    return(times)
  }
  seq(0, duration, length.out = check_count(n, "n", min = 2))
}

poly_coefficients <- function(q0, q1, T, v0, v1, a0, a1, order) {
  if (order == "cubic") {
    c(q0, v0,
      (3 * (q1 - q0) - (2 * v0 + v1) * T) / T^2,
      (-2 * (q1 - q0) + (v0 + v1) * T) / T^3, 0, 0)
  } else {
    h <- q1 - q0
    c(q0, v0, a0 / 2,
      (20 * h - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T^2) / (2 * T^3),
      (-30 * h + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T^2) / (2 * T^4),
      (12 * h - 6 * (v1 + v0) * T + (a1 - a0) * T^2) / (2 * T^5))
  }
}

poly_eval <- function(cf, t) {
  p <- cf[1] + t * (cf[2] + t * (cf[3] + t * (cf[4] + t * (cf[5] + t * cf[6]))))
  v <- cf[2] + t * (2 * cf[3] + t * (3 * cf[4] + t * (4 * cf[5] + t * 5 * cf[6])))
  a <- 2 * cf[3] + t * (6 * cf[4] + t * (12 * cf[5] + t * 20 * cf[6]))
  cbind(p, v, a)
}

#' Polynomial joint trajectories
#'
#' Cubic polynomials match endpoint positions and velocities; quintic
#' polynomials also match endpoint accelerations. Each joint gets its own
#' polynomial on \eqn{[0, T]}; outside this interval the trajectory is held
#' at rest at its endpoint positions. The boundary conditions are satisfied exactly.
#'
#' @param q0,q1 Start and end configurations.
#' @param duration Duration \eqn{T} in seconds.
#' @param times Optional sample times; otherwise `n` equally spaced samples.
#' @param n Number of samples.
#' @param qd0,qd1,qdd0,qdd1 Endpoint velocities and accelerations (zero by
#'   default). Accelerations are ignored by cubic trajectories.
#' @param order `"quintic"` or `"cubic"`.
#' @return A `robot_trajectory` whose `evaluate(t)` function returns
#'   positions, velocities and accelerations at arbitrary times.
#' @examples
#' tr <- robot_traj_polynomial(c(0, 0), c(1, -1), duration = 2, n = 5)
#' tr$q
#' @export
robot_traj_polynomial <- function(q0, q1, duration, times = NULL, n = 101,
                                  qd0 = NULL, qd1 = NULL, qdd0 = NULL,
                                  qdd1 = NULL,
                                  order = c("quintic", "cubic")) {
  order <- match.arg(order)
  q0 <- check_numeric_vector(q0, name = "q0")
  m <- length(q0)
  q1 <- check_numeric_vector(q1, m, "q1")
  z <- rep(0, m)
  qd0 <- if (is.null(qd0)) z else check_numeric_vector(qd0, m, "qd0")
  qd1 <- if (is.null(qd1)) z else check_numeric_vector(qd1, m, "qd1")
  qdd0 <- if (is.null(qdd0)) z else check_numeric_vector(qdd0, m, "qdd0")
  qdd1 <- if (is.null(qdd1)) z else check_numeric_vector(qdd1, m, "qdd1")
  T <- check_scalar(duration, "duration", positive = TRUE)
  cf <- lapply(seq_len(m), function(i)
    poly_coefficients(q0[i], q1[i], T, qd0[i], qd1[i], qdd0[i], qdd1[i], order))
  evaluate <- function(t) {
    tt <- pmin(pmax(t, 0), T)
    P <- V <- A <- matrix(0, length(t), m)
    for (i in seq_len(m)) {
      e <- poly_eval(cf[[i]], tt)
      P[, i] <- e[, 1]
      V[, i] <- ifelse(t < 0 | t > T, 0, e[, 2])
      A[, i] <- ifelse(t < 0 | t > T, 0, e[, 3])
    }
    # Outside [0, T] the trajectory is held at rest at its endpoints.
    if (any(t < 0)) P[t < 0, ] <- matrix(q0, sum(t < 0), m, byrow = TRUE)
    if (any(t > T)) P[t > T, ] <- matrix(q1, sum(t > T), m, byrow = TRUE)
    list(q = P, qd = V, qdd = A)
  }
  t <- time_grid(T, times, n)
  e <- evaluate(t)
  new_trajectory(t, e$q, e$qd, e$qdd, names(q0), paste(order, "polynomial"),
                 evaluate)
}

trapezoid_profile <- function(h, vmax, amax) {
  # Scalar time scaling s(t) from 0 to 1 with |s'| <= vmax, |s''| <= amax.
  if (vmax^2 / amax >= 1) {
    ta <- sqrt(1 / amax)
    list(ta = ta, tc = 0, T = 2 * ta, vpeak = amax * ta, amax = amax)
  } else {
    ta <- vmax / amax
    tc <- 1 / vmax - ta
    list(ta = ta, tc = tc, T = 2 * ta + tc, vpeak = vmax, amax = amax)
  }
}

trapezoid_eval <- function(pr, t) {
  ta <- pr$ta; tc <- pr$tc; T <- pr$T; a <- pr$amax; vp <- pr$vpeak
  outside <- t < 0 | t > T
  t <- pmin(pmax(t, 0), T)
  s <- ifelse(t < ta, 0.5 * a * t^2,
       ifelse(t < ta + tc, 0.5 * a * ta^2 + vp * (t - ta),
              1 - 0.5 * a * (T - t)^2))
  sd <- ifelse(t < ta, a * t, ifelse(t < ta + tc, vp, a * (T - t)))
  sdd <- ifelse(t < ta, a, ifelse(t < ta + tc, 0, -a))
  sdd[t >= T | outside] <- 0
  sd[t >= T | outside] <- 0
  cbind(s, sd, sdd)
}

#' Trapezoidal velocity profile trajectories
#'
#' Moves from `q0` to `q1` along the straight line in joint space with a
#' common time scaling \eqn{q(t) = q_0 + s(t)(q_1 - q_0)}, where
#' \eqn{s(t)} has a trapezoidal velocity profile (or a triangular one when
#' the peak velocity is not reached). The profile is the fastest one with
#' this shape that respects every joint's velocity and acceleration
#' limits, so all joints start and stop together. The motion is rest to
#' rest.
#'
#' @param q0,q1 Start and end configurations.
#' @param vmax,amax Positive velocity and acceleration limits (scalars or
#'   one per joint).
#' @param times Optional sample times.
#' @param n Number of samples.
#' @return A `robot_trajectory` with field `duration`.
#' @examples
#' tr <- robot_traj_trapezoidal(c(0, 0), c(1, 2), vmax = 1, amax = 2)
#' tr$duration
#' @export
robot_traj_trapezoidal <- function(q0, q1, vmax, amax, times = NULL,
                                   n = 101) {
  q0 <- check_numeric_vector(q0, name = "q0")
  m <- length(q0)
  q1 <- check_numeric_vector(q1, m, "q1")
  vmax <- check_per_item(vmax, m, "vmax")
  amax <- check_per_item(amax, m, "amax")
  if (any(vmax <= 0) || any(amax <= 0)) {
    stop("`vmax` and `amax` must be positive.", call. = FALSE)
  }
  h <- q1 - q0
  moving <- abs(h) > 0
  if (!any(moving)) {
    t <- if (is.null(times)) 0 else times
    Z <- matrix(0, length(t), m)
    tr <- new_trajectory(t, matrix(q0, length(t), m, byrow = TRUE), Z, Z,
                         names(q0), "trapezoidal")
    tr$duration <- 0
    tr$evaluate <- function(t) {
      Zt <- matrix(0, length(t), m)
      list(q = matrix(q0, length(t), m, byrow = TRUE), qd = Zt, qdd = Zt)
    }
    return(tr)
  }
  vs <- min(vmax[moving] / abs(h[moving]))
  as <- min(amax[moving] / abs(h[moving]))
  pr <- trapezoid_profile(1, vs, as)
  evaluate <- function(t) {
    e <- trapezoid_eval(pr, t)
    list(q = outer(e[, 1], h) + matrix(q0, length(t), m, byrow = TRUE),
         qd = outer(e[, 2], h), qdd = outer(e[, 3], h))
  }
  t <- time_grid(pr$T, times, n)
  e <- evaluate(t)
  tr <- new_trajectory(t, e$q, e$qd, e$qdd, names(q0), "trapezoidal", evaluate)
  tr$duration <- pr$T
  tr
}

#' Time-parameterize a geometric path
#'
#' Converts a path (one configuration per row, for example from
#' [robot_plan()]) into a rest-to-rest trajectory through every waypoint:
#' each segment uses [robot_traj_trapezoidal()] with the given limits and
#' the robot stops at each waypoint. The result is feasible for the stated
#' velocity and acceleration limits, but not time optimal; a collision-free
#' path is not automatically dynamically feasible for torque limits.
#'
#' @param path Matrix of waypoints or a `robot_plan`.
#' @param vmax,amax Velocity and acceleration limits.
#' @param dt Sample period in seconds.
#' @return A `robot_trajectory` with field `duration`.
#' @examples
#' path <- rbind(c(0, 0), c(1, 0), c(1, 1))
#' tr <- robot_traj_path(path, vmax = 1, amax = 2, dt = 0.05)
#' tr$duration
#' @export
robot_traj_path <- function(path, vmax, amax, dt = 0.01) {
  if (inherits(path, "robot_plan")) path <- path$path
  P <- check_matrix(path, name = "path")
  if (nrow(P) < 2) stop("`path` needs at least two waypoints.", call. = FALSE)
  dt <- check_scalar(dt, "dt", positive = TRUE)
  segs <- lapply(seq_len(nrow(P) - 1), function(i)
    robot_traj_trapezoidal(P[i, ], P[i + 1, ], vmax, amax, times = 0))
  durations <- vapply(segs, `[[`, 0, "duration")
  starts <- c(0, cumsum(durations))
  total <- sum(durations)
  evaluate <- function(t) {
    m <- ncol(P)
    out <- list(q = matrix(0, length(t), m), qd = matrix(0, length(t), m),
                qdd = matrix(0, length(t), m))
    idx <- findInterval(t, starts, rightmost.closed = TRUE)
    idx <- pmin(pmax(idx, 1), length(segs))
    for (s in unique(idx)) {
      sel <- idx == s
      e <- if (durations[s] > 0) segs[[s]]$evaluate(t[sel] - starts[s])
           else list(q = matrix(P[s, ], sum(sel), m, byrow = TRUE),
                     qd = matrix(0, sum(sel), m), qdd = matrix(0, sum(sel), m))
      out$q[sel, ] <- e$q
      out$qd[sel, ] <- e$qd
      out$qdd[sel, ] <- e$qdd
    }
    out
  }
  t <- unique(c(seq(0, total, by = dt), total))
  e <- evaluate(t)
  tr <- new_trajectory(t, e$q, e$qd, e$qdd, colnames(P), "waypoint path",
                       evaluate)
  tr$duration <- total
  tr$waypoint_times <- starts
  tr
}

#' Task-space (Cartesian) trajectories
#'
#' Interpolates between two poses with a quintic time scaling
#' \eqn{s(t)}: the position moves along the straight line and the
#' orientation along the shortest great circle ([quaternion_slerp()]),
#' \eqn{R(s) = \exp(s\,\widehat{\phi}) R_0} with world-frame rotation
#' vector \eqn{\phi = \log(R_1 R_0^\top)}. The spatial velocity of the
#' moving frame, in world coordinates and linear part first, is therefore
#' \eqn{(v, \omega) = \dot s\,(p_1 - p_0, \phi)} and its acceleration
#' \eqn{\ddot s\,(p_1 - p_0, \phi)}; both vanish at the ends.
#'
#' When `robot` is given, the path is also converted to joints: inverse
#' kinematics is solved at each sample, warm-started from the previous
#' solution (from `initial` at the first sample), and joint velocities and
#' accelerations follow from the weighted least-squares solutions of
#' \eqn{J \dot q = V} and \eqn{J \ddot q = A - \dot J \dot q}, with the
#' world Jacobian of `frame` and \eqn{\dot J \dot q} from a central
#' difference of \eqn{J} along \eqn{\dot q}.
#'
#' @param from,to Start and end `robot_pose` objects (SE(3)).
#' @param duration Duration in seconds.
#' @param n Number of samples.
#' @param robot Optional `robot_model` for the joint-space conversion.
#' @param initial Initial guess for the first inverse kinematics solve.
#' @param frame End-effector frame; defaults to the robot's end effector.
#' @param weights Task weights passed to [robot_ik()] and used in the
#'   velocity solves (for example `c(1, 1, 0, 0, 0, 1)` for a planar arm).
#' @param ... Further arguments passed to [robot_ik()].
#' @return A list with `time`, `s`, `poses` (a list of `robot_pose`),
#'   `velocity` and `acceleration` (one row per sample, columns
#'   `vx, vy, vz, wx, wy, wz`), and, with `robot`, `joint` (a
#'   `robot_trajectory`) and `converged` (logical, one per sample).
#' @examples
#' ct <- robot_traj_cartesian(robot_pose(), robot_pose(rotz(1), c(1, 0, 0)),
#'                            duration = 1, n = 3)
#' pose_translation(ct$poses[[2]])
#' ct$velocity[2, ]
#' arm <- robot_example("planar2")
#' a <- robot_fk(arm, c(0.3, 1.2))
#' b <- robot_pose(translation = c(1.2, 0.6, 0))
#' jt <- robot_traj_cartesian(a, b, 1, n = 11, robot = arm, initial = c(0.3, 1.2),
#'                            weights = c(1, 1, 0, 0, 0, 0))
#' all(jt$converged)
#' @export
robot_traj_cartesian <- function(from, to, duration, n = 51, robot = NULL,
                                 initial = NULL, frame = NULL, weights = NULL,
                                 ...) {
  stopifnot(inherits(from, "robot_pose"), inherits(to, "robot_pose"))
  if (from$dim != 3 || to$dim != 3) stop("Poses must be SE(3).", call. = FALSE)
  sc <- robot_traj_polynomial(0, 1, duration, n = n)
  s <- pmin(pmax(as.numeric(sc$q), 0), 1)
  sd <- as.numeric(sc$qd)
  sdd <- as.numeric(sc$qdd)
  dp <- pose_translation_vector(to) - pose_translation_vector(from)
  phi <- rotation_delta(pose_rotation_matrix(from), pose_rotation_matrix(to))
  axis <- c(dp, phi)
  cols <- c("vx", "vy", "vz", "wx", "wy", "wz")
  V <- outer(sd, axis)
  A <- outer(sdd, axis)
  colnames(V) <- colnames(A) <- cols
  out <- list(time = sc$time, s = s,
              poses = lapply(s, function(si) robot_pose_interp(from, to, si)),
              velocity = V, acceleration = A)
  if (is.null(robot)) return(out)
  check_robot(robot)
  w <- if (is.null(weights)) rep(1, 6) else check_numeric_vector(weights, 6, "weights")
  sw <- sqrt(w)
  m <- length(s)
  Q <- Qd <- Qdd <- matrix(0, m, robot$dof)
  ok <- logical(m)
  q <- initial
  for (i in seq_len(m)) {
    sol <- robot_ik(robot, out$poses[[i]], initial = q, frame = frame,
                    weights = weights, ...)
    q <- sol$q
    ok[i] <- sol$converged
    J <- robot_jacobian(robot, q, frame)
    qd <- pinv_solve(sw * J, sw * V[i, ])
    h <- 1e-6
    Jdqd <- if (any(qd != 0)) {
      as.numeric((robot_jacobian(robot, q + h * qd, frame) -
                    robot_jacobian(robot, q - h * qd, frame)) %*% qd) / (2 * h)
    } else numeric(6)
    Q[i, ] <- q
    Qd[i, ] <- qd
    Qdd[i, ] <- pinv_solve(sw * J, sw * (A[i, ] - Jdqd))
  }
  out$joint <- new_trajectory(sc$time, Q, Qd, Qdd, robot_joint_names(robot),
                              "cartesian")
  out$converged <- ok
  out
}

#' Check a trajectory against robot limits
#'
#' Reports samples that violate joint position limits and, where the
#' joints declare them, velocity and acceleration limits.
#'
#' @param robot A `robot_model`.
#' @param trajectory A `robot_trajectory`.
#' @param tol Tolerance.
#' @return A list with logical `ok` and data frames of violations.
#' @examples
#' robot <- robot_example("planar2")
#' tr <- robot_traj_polynomial(c(0, 0), c(1, 1), duration = 1)
#' robot_traj_check(robot, tr)$ok
#' @export
robot_traj_check <- function(robot, trajectory, tol = 1e-9) {
  check_robot(robot)
  stopifnot(inherits(trajectory, "robot_trajectory"))
  check_trajectory_structure(trajectory, robot$dof)
  L <- robot_limits(robot)
  mv <- robot$joints[robot$movable]
  vl <- vapply(mv, function(j) j$velocity %||% Inf, 0)
  al <- vapply(mv, function(j) j$acceleration %||% Inf, 0)
  viol <- function(M, lo, hi, what) {
    bad <- which(sweep(M, 2, lo - tol, "<") | sweep(M, 2, hi + tol, ">"),
                 arr.ind = TRUE)
    if (!length(bad)) return(NULL)
    data.frame(quantity = what, time = trajectory$time[bad[, 1]],
               joint = robot_joint_names(robot)[bad[, 2]],
               value = M[bad], stringsAsFactors = FALSE)
  }
  v <- rbind(viol(trajectory$q, L[, 1], L[, 2], "position"),
             viol(trajectory$qd, -vl, vl, "velocity"),
             viol(trajectory$qdd, -al, al, "acceleration"))
  list(ok = is.null(v), violations = v)
}

# A malformed trajectory (missing or nonfinite samples, mismatched sizes,
# unordered time) is rejected before any limit is checked, so it can never
# be certified.
check_trajectory_structure <- function(tr, dof) {
  bad <- function(msg) stop(sprintf("Malformed trajectory: %s.", msg),
                            call. = FALSE)
  t <- tr$time
  if (!is.numeric(t) || !length(t) || any(!is.finite(t))) {
    bad("`time` must be a nonempty finite numeric vector")
  }
  if (is.unsorted(t)) bad("`time` must be nondecreasing")
  for (f in c("q", "qd", "qdd")) {
    M <- tr[[f]]
    if (!is.matrix(M) || !is.numeric(M)) bad(sprintf("`%s` must be a numeric matrix", f))
    if (nrow(M) != length(t)) bad(sprintf("`%s` needs one row per time", f))
    if (ncol(M) != dof) {
      stop("Trajectory dimension does not match the robot.", call. = FALSE)
    }
    if (any(!is.finite(M))) bad(sprintf("`%s` has missing or nonfinite values", f))
  }
  invisible(TRUE)
}
