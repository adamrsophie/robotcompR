# Rigid-body dynamics with spatial vector algebra.
#
# Spatial vectors are ordered linear first, matching twists and Jacobians:
# motion m = c(v, w), force f = c(F, n). Body quantities are expressed in
# link frame coordinates. With this ordering:
#   crm(v) = [w^, v^; 0, w^]       (motion cross product)
#   crf(v) = [w^, 0; v^, w^] = -t(crm(v))   (force cross product)
#   inertia at the frame origin: [m I, -m c^; m c^, Ic - m c^ c^]
# The algorithms follow Featherstone, "Rigid Body Dynamics Algorithms"
# (Springer, 2008), rewritten for this ordering.

crm <- function(v) {
  M <- matrix(0, 6, 6)
  W <- so3_hat(v[4:6])
  M[1:3, 1:3] <- W
  M[4:6, 4:6] <- W
  M[1:3, 4:6] <- so3_hat(v[1:3])
  M
}

crf <- function(v) -t(crm(v))

spatial_inertia <- function(link) {
  m <- link$mass
  C <- so3_hat(link$com)
  I <- matrix(0, 6, 6)
  I[1:3, 1:3] <- m * diag(3)
  I[1:3, 4:6] <- -m * C
  I[4:6, 1:3] <- m * C
  I[4:6, 4:6] <- link$inertia - m * C %*% C
  I
}

motion_subspace <- function(j) {
  S <- if (j$type == "prismatic") c(j$axis, 0, 0, 0)
       else if (j$type == "fixed") rep(0, 6) else c(0, 0, 0, j$axis)
  if (!is.null(j$post)) S <- as.numeric(se3_adjoint(pose_inverse_matrix(j$post)) %*% S)
  S
}

link_inertia <- function(robot, k) {
  robot$spatial_inertia[[k]] %||% spatial_inertia(robot$links[[k]])
}

pose_inverse_matrix <- function(M) {
  R <- M[1:3, 1:3]
  out <- diag(4)
  out[1:3, 1:3] <- t(R)
  out[1:3, 4] <- -t(R) %*% M[1:3, 4]
  out
}

# Motion transform from parent to child coordinates.
link_transforms <- function(robot, q) {
  nl <- length(robot$link_names)
  X <- vector("list", nl)
  S <- vector("list", nl)
  for (k in seq_len(nl)[-1]) {
    ji <- k - 1L
    j <- robot$joints[[ji]]
    qi <- if (robot$movable[ji]) q[robot$q_index[ji]] else 0
    T <- j$origin %*% joint_motion(j, qi)
    if (!is.null(j$post)) T <- T %*% j$post
    X[[k]] <- se3_adjoint(pose_inverse_matrix(T))
  }
  list(X = X, S = robot$motion_subspace %||% c(list(NULL),
                                               lapply(robot$joints, motion_subspace)))
}

check_gravity <- function(g) check_numeric_vector(g, 3, "gravity")

root_acceleration <- function(robot, gravity) {
  c(-as.numeric(crossprod(robot$base[1:3, 1:3], gravity)), 0, 0, 0)
}

external_forces <- function(robot, f_ext, q) {
  nl <- length(robot$link_names)
  F <- replicate(nl, rep(0, 6), simplify = FALSE)
  if (is.null(f_ext)) return(F)
  if (!is.list(f_ext) || is.null(names(f_ext))) {
    stop("`f_ext` must be a named list of length-6 wrenches.", call. = FALSE)
  }
  fk <- fk_internal(robot, q)
  for (nm in names(f_ext)) {
    k <- frame_index(robot, nm)
    w <- check_numeric_vector(f_ext[[nm]], 6, "f_ext")
    R <- fk$link[[k]][1:3, 1:3]
    F[[k]] <- c(crossprod(R, w[1:3]), crossprod(R, w[4:6]))
  }
  F
}

qvec <- function(robot, x, name) {
  if (is.null(x)) return(rep(0, robot$dof))
  check_q(robot, x, name)
}

#' Inverse dynamics with the recursive Newton-Euler algorithm
#'
#' Computes the joint forces and torques
#' \eqn{\tau = M(q)\ddot q + C(q, \dot q)\dot q + g(q) - J^\top f_{ext}}
#' for a fixed-base rigid-body tree in \eqn{O(n)} operations. Gravity is a
#' world-frame acceleration vector (default \eqn{(0, 0, -9.81)}
#' m/s^2). External wrenches are given per link as
#' `c(force, moment)` in world axes with the moment taken about the link
#' frame origin. Joint friction, damping, motor inertia and contact are
#' not modeled.
#'
#' @param robot A `robot_model` with inertial parameters.
#' @param q,qd,qdd Joint positions, velocities and accelerations.
#' @param gravity Gravity vector in world coordinates.
#' @param f_ext Optional named list of external wrenches.
#' @return Named numeric vector of joint efforts (N m or N).
#' @examples
#' robot <- robot_example("planar1")
#' # Hold a horizontal 1 kg, 1 m rod against gravity along -y.
#' robot_inverse_dynamics(robot, 0, gravity = c(0, -9.81, 0))
#' @export
robot_inverse_dynamics <- function(robot, q, qd = NULL, qdd = NULL,
                                   gravity = c(0, 0, -9.81), f_ext = NULL) {
  check_robot(robot)
  q <- check_q(robot, q)
  qd <- qvec(robot, qd, "qd")
  qdd <- qvec(robot, qdd, "qdd")
  tau <- rnea_internal(robot, q, qd, qdd, check_gravity(gravity),
                       external_forces(robot, f_ext, q))
  names(tau) <- robot_joint_names(robot)
  tau
}

rnea_internal <- function(robot, q, qd, qdd, gravity, Fext, tr = NULL) {
  nl <- length(robot$link_names)
  tr <- tr %||% link_transforms(robot, q)
  v <- vector("list", nl)
  a <- vector("list", nl)
  f <- vector("list", nl)
  v[[1]] <- rep(0, 6)
  a[[1]] <- root_acceleration(robot, gravity)
  for (k in seq_len(nl)[-1]) {
    ji <- k - 1L
    p <- robot$parent_index[k]
    X <- tr$X[[k]]
    S <- tr$S[[k]]
    if (robot$movable[ji]) {
      i <- robot$q_index[ji]
      vJ <- S * qd[i]
      v[[k]] <- as.numeric(X %*% v[[p]]) + vJ
      a[[k]] <- as.numeric(X %*% a[[p]]) + S * qdd[i] +
        as.numeric(crm(v[[k]]) %*% vJ)
    } else {
      v[[k]] <- as.numeric(X %*% v[[p]])
      a[[k]] <- as.numeric(X %*% a[[p]])
    }
    I <- link_inertia(robot, k)
    f[[k]] <- as.numeric(I %*% a[[k]] + crf(v[[k]]) %*% (I %*% v[[k]])) -
      Fext[[k]]
  }
  tau <- numeric(robot$dof)
  for (k in rev(seq_len(nl)[-1])) {
    ji <- k - 1L
    if (robot$movable[ji]) tau[robot$q_index[ji]] <- sum(tr$S[[k]] * f[[k]])
    p <- robot$parent_index[k]
    if (p > 1) f[[p]] <- f[[p]] + as.numeric(crossprod(tr$X[[k]], f[[k]]))
  }
  tau
}

#' Joint-space mass matrix with the composite rigid body algorithm
#'
#' Returns the symmetric positive semidefinite generalized inertia matrix
#' \eqn{M(q)} (positive definite when every degree of freedom moves some
#' mass).
#'
#' @inheritParams robot_inverse_dynamics
#' @param q Joint positions.
#' @return An n by n matrix.
#' @examples
#' robot_mass_matrix(robot_example("planar2"), c(0, 0.5))
#' @export
robot_mass_matrix <- function(robot, q) {
  check_robot(robot)
  q <- check_q(robot, q)
  M <- crba_internal(robot, q)
  dimnames(M) <- list(robot_joint_names(robot), robot_joint_names(robot))
  M
}

crba_internal <- function(robot, q, tr = NULL) {
  nl <- length(robot$link_names)
  tr <- tr %||% link_transforms(robot, q)
  Ic <- robot$spatial_inertia %||% lapply(robot$links, spatial_inertia)
  for (k in rev(seq_len(nl)[-1])) {
    p <- robot$parent_index[k]
    if (p > 1) Ic[[p]] <- Ic[[p]] + crossprod(tr$X[[k]], Ic[[k]] %*% tr$X[[k]])
  }
  n <- robot$dof
  M <- matrix(0, n, n)
  for (k in seq_len(nl)[-1]) {
    ji <- k - 1L
    if (!robot$movable[ji]) next
    i <- robot$q_index[ji]
    F <- as.numeric(Ic[[k]] %*% tr$S[[k]])
    M[i, i] <- sum(tr$S[[k]] * F)
    j <- k
    repeat {
      p <- robot$parent_index[j]
      if (p <= 1) break
      F <- as.numeric(crossprod(tr$X[[j]], F))
      j <- p
      if (robot$movable[j - 1L]) {
        jj <- robot$q_index[j - 1L]
        M[i, jj] <- M[jj, i] <- sum(F * tr$S[[j]])
      }
    }
  }
  M
}

#' Forward dynamics with the articulated body algorithm
#'
#' Solves \eqn{M(q)\ddot q = \tau - C(q, \dot q)\dot q - g(q) +
#' J^\top f_{ext}} for the joint accelerations in \eqn{O(n)} operations.
#' Every degree of freedom must move some mass or rotational inertia;
#' otherwise the articulated inertia is singular and the function stops.
#'
#' @inheritParams robot_inverse_dynamics
#' @param q,qd Joint positions and velocities.
#' @param tau Joint efforts (defaults to zero).
#' @return Named numeric vector of joint accelerations.
#' @examples
#' robot <- robot_example("planar2")
#' robot_forward_dynamics(robot, c(0, 0), gravity = c(0, -9.81, 0))
#' @export
robot_forward_dynamics <- function(robot, q, qd = NULL, tau = NULL,
                                   gravity = c(0, 0, -9.81), f_ext = NULL) {
  check_robot(robot)
  q <- check_q(robot, q)
  qd <- qvec(robot, qd, "qd")
  tau <- qvec(robot, tau, "tau")
  qdd <- aba_internal(robot, q, qd, tau, check_gravity(gravity),
                      external_forces(robot, f_ext, q))
  names(qdd) <- robot_joint_names(robot)
  qdd
}

aba_internal <- function(robot, q, qd, tau, gravity, Fext, tr = NULL) {
  nl <- length(robot$link_names)
  tr <- tr %||% link_transforms(robot, q)
  v <- vector("list", nl)
  cc <- vector("list", nl)
  IA <- vector("list", nl)
  pA <- vector("list", nl)
  v[[1]] <- rep(0, 6)
  for (k in seq_len(nl)[-1]) {
    ji <- k - 1L
    p <- robot$parent_index[k]
    vp <- as.numeric(tr$X[[k]] %*% v[[p]])
    if (robot$movable[ji]) {
      vJ <- tr$S[[k]] * qd[robot$q_index[ji]]
      v[[k]] <- vp + vJ
      cc[[k]] <- as.numeric(crm(v[[k]]) %*% vJ)
    } else {
      v[[k]] <- vp
      cc[[k]] <- rep(0, 6)
    }
    IA[[k]] <- link_inertia(robot, k)
    pA[[k]] <- as.numeric(crf(v[[k]]) %*% (IA[[k]] %*% v[[k]])) - Fext[[k]]
  }
  # Composite (rigid) subtree inertias give each pivot a reference of the
  # same physical units: the articulated inertia never exceeds it, and a
  # pivot that is tiny relative to it means the joint drives no inertia.
  IC <- IA
  U <- vector("list", nl)
  D <- numeric(nl)
  u <- numeric(nl)
  for (k in rev(seq_len(nl)[-1])) {
    ji <- k - 1L
    p <- robot$parent_index[k]
    if (robot$movable[ji]) {
      S <- tr$S[[k]]
      U[[k]] <- as.numeric(IA[[k]] %*% S)
      D[k] <- sum(S * U[[k]])
      Dref <- sum(S * (IC[[k]] %*% S))
      if (!(Dref > 0) || D[k] <= 1e-12 * Dref) {
        stop(sprintf(paste0("Joint '%s' drives no inertia; forward dynamics ",
                            "is undefined (add mass or inertia)."),
                     robot$joint_names[ji]), call. = FALSE)
      }
      u[k] <- tau[robot$q_index[ji]] - sum(S * pA[[k]])
      Ia <- IA[[k]] - tcrossprod(U[[k]]) / D[k]
      pa <- pA[[k]] + as.numeric(Ia %*% cc[[k]]) + U[[k]] * u[k] / D[k]
    } else {
      Ia <- IA[[k]]
      pa <- pA[[k]] + as.numeric(Ia %*% cc[[k]])
    }
    if (p > 1) {
      X <- tr$X[[k]]
      IA[[p]] <- IA[[p]] + crossprod(X, Ia %*% X)
      IC[[p]] <- IC[[p]] + crossprod(X, IC[[k]] %*% X)
      pA[[p]] <- pA[[p]] + as.numeric(crossprod(X, pa))
    }
  }
  a <- vector("list", nl)
  a[[1]] <- root_acceleration(robot, gravity)
  qdd <- numeric(robot$dof)
  for (k in seq_len(nl)[-1]) {
    ji <- k - 1L
    p <- robot$parent_index[k]
    ap <- as.numeric(tr$X[[k]] %*% a[[p]]) + cc[[k]]
    if (robot$movable[ji]) {
      i <- robot$q_index[ji]
      qdd[i] <- (u[k] - sum(U[[k]] * ap)) / D[k]
      a[[k]] <- ap + tr$S[[k]] * qdd[i]
    } else {
      a[[k]] <- ap
    }
  }
  qdd
}

#' Gravity, Coriolis and centrifugal terms
#'
#' `robot_gravity_torque()` returns \eqn{g(q)}, the efforts that hold the
#' robot still against gravity (static gravity compensation).
#' `robot_coriolis()` returns the velocity product terms
#' \eqn{C(q, \dot q)\dot q} without gravity. `robot_bias_forces()`
#' returns their sum, \eqn{C(q, \dot q)\dot q + g(q)}.
#'
#' @inheritParams robot_inverse_dynamics
#' @param q,qd Joint positions and velocities.
#' @return Named numeric vector.
#' @examples
#' robot <- robot_example("planar2")
#' robot_gravity_torque(robot, c(0, 0), gravity = c(0, -9.81, 0))
#' robot_coriolis(robot, c(0, 0.5), c(1, 0))
#' @name robot_bias
NULL

#' @rdname robot_bias
#' @export
robot_gravity_torque <- function(robot, q, gravity = c(0, 0, -9.81)) {
  robot_inverse_dynamics(robot, q, gravity = gravity)
}

#' @rdname robot_bias
#' @export
robot_coriolis <- function(robot, q, qd) {
  robot_inverse_dynamics(robot, q, qd, gravity = c(0, 0, 0))
}

#' @rdname robot_bias
#' @export
robot_bias_forces <- function(robot, q, qd, gravity = c(0, 0, -9.81)) {
  robot_inverse_dynamics(robot, q, qd, gravity = gravity)
}

#' Center of mass and mechanical energy
#'
#' `robot_com()` returns the total mass and the world position of the
#' center of mass. `robot_energy()` returns the kinetic energy
#' \eqn{\tfrac12 \dot q^\top M(q) \dot q}, the gravitational potential
#' energy \eqn{-\sum_i m_i g^\top c_i} (zero at the world origin), and
#' their sum.
#'
#' @inheritParams robot_inverse_dynamics
#' @param q,qd Joint positions and velocities.
#' @return A list (`robot_com`) or named numeric vector (`robot_energy`).
#' @examples
#' robot <- robot_example("planar2")
#' robot_com(robot, c(0, 0))
#' robot_energy(robot, c(0, 0), c(1, 0), gravity = c(0, -9.81, 0))
#' @name robot_energy
NULL

#' @rdname robot_energy
#' @export
robot_com <- function(robot, q) {
  check_robot(robot)
  q <- check_q(robot, q)
  fk <- fk_internal(robot, q)
  m <- vapply(robot$links, `[[`, 0, "mass")
  if (sum(m) <= 0) stop("The robot has no mass.", call. = FALSE)
  P <- vapply(seq_along(m), function(k)
    as.numeric(fk$link[[k]][1:3, 1:3] %*% robot$links[[k]]$com +
                 fk$link[[k]][1:3, 4]), numeric(3))
  list(mass = sum(m), position = as.numeric(P %*% m) / sum(m))
}

#' @rdname robot_energy
#' @export
robot_energy <- function(robot, q, qd, gravity = c(0, 0, -9.81)) {
  check_robot(robot)
  q <- check_q(robot, q)
  qd <- check_q(robot, qd, "qd")
  g <- check_gravity(gravity)
  M <- crba_internal(robot, q)
  com <- robot_com(robot, q)
  kin <- 0.5 * sum(qd * (M %*% qd))
  pot <- -com$mass * sum(g * com$position)
  c(kinetic = kin, potential = pot, total = kin + pot)
}

#' Simulate torque-driven rigid-body dynamics
#'
#' Integrates \eqn{\ddot q = ABA(q, \dot q, \tau(t, q, \dot q))} with
#' [deSolve::ode()]. This is a dynamic simulation of the rigid-body model
#' only: joint limits, friction, contact and impacts are not modeled, and
#' the result is not a high-fidelity physics simulation. Energy drift in an
#' unforced conservative simulation measures integrator error, not model
#' error; tighten `rtol` and `atol` to reduce it.
#'
#' @inheritParams robot_inverse_dynamics
#' @param q0,qd0 Initial positions and velocities.
#' @param times Output times (seconds), increasing.
#' @param torque A function `function(t, q, qd)` returning joint efforts,
#'   a constant numeric vector of efforts (one per joint), or `NULL` for
#'   zero torque.
#' @param method Integration method passed to [deSolve::ode()].
#' @param rtol,atol Integration tolerances.
#' @return A `robot_trajectory` with `time`, `q`, `qd` and `qdd`.
#' @examples
#' robot <- robot_example("planar1")
#' sim <- robot_simulate(robot, q0 = 0.5, qd0 = 0, times = seq(0, 1, 0.1),
#'                       gravity = c(0, -9.81, 0))
#' tail(sim$q, 1)
#' @export
robot_simulate <- function(robot, q0, qd0 = NULL, times, torque = NULL,
                           gravity = c(0, 0, -9.81), f_ext = NULL,
                           method = "lsoda", rtol = 1e-8, atol = 1e-10) {
  check_robot(robot)
  q0 <- check_q(robot, q0, "q0")
  qd0 <- qvec(robot, qd0, "qd0")
  times <- check_numeric_vector(times, name = "times")
  if (length(times) < 2 || any(diff(times) <= 0)) {
    stop("`times` must be increasing with at least two values.", call. = FALSE)
  }
  g <- check_gravity(gravity)
  n <- robot$dof
  tau_fun <- if (is.null(torque)) {
    function(t, q, qd) rep(0, n)
  } else if (is.function(torque)) {
    torque
  } else {
    tau_const <- check_numeric_vector(torque, n, "torque")
    function(t, q, qd) tau_const
  }
  rhs <- function(t, y, parms) {
    q <- y[seq_len(n)]
    qd <- y[n + seq_len(n)]
    tau <- check_numeric_vector(tau_fun(t, q, qd), n, "torque(t, q, qd)")
    qdd <- aba_internal(robot, q, qd, tau, g,
                        external_forces(robot, f_ext, q))
    list(c(qd, qdd))
  }
  out <- deSolve::ode(c(q0, qd0), times, rhs, NULL, method = method,
                      rtol = rtol, atol = atol)
  Y <- unclass(out)[, -1, drop = FALSE]
  Q <- Y[, seq_len(n), drop = FALSE]
  Qd <- Y[, n + seq_len(n), drop = FALSE]
  Qdd <- matrix(vapply(seq_len(nrow(Q)), function(i) {
    tau <- tau_fun(times[i], Q[i, ], Qd[i, ])
    aba_internal(robot, Q[i, ], Qd[i, ], tau, g,
                 external_forces(robot, f_ext, Q[i, ]))
  }, numeric(n)), nrow = nrow(Q), byrow = TRUE)
  new_trajectory(times, Q, Qd, Qdd, robot_joint_names(robot),
                 type = "dynamic simulation")
}
