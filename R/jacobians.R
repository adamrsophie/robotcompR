# Geometric and analytic Jacobians.

jacobian_internal <- function(robot, q, k, point = NULL,
                              reference = "world", fk = NULL) {
  fk <- fk %||% fk_internal(robot, q)
  Tt <- fk$link[[k]]
  p <- if (is.null(point)) Tt[1:3, 4] else Tt[1:3, 1:3] %*% point + Tt[1:3, 4]
  p <- as.numeric(p)
  J <- matrix(0, 6, robot$dof)
  for (ji in robot$ancestors[[k]]) {
    j <- robot$joints[[ji]]
    Tj <- fk$joint[[ji + 1L]]
    z <- as.numeric(Tj[1:3, 1:3] %*% j$axis)
    col <- robot$q_index[ji]
    if (j$type == "prismatic") {
      J[1:3, col] <- z
    } else {
      J[1:3, col] <- cross3(z, p - Tj[1:3, 4])
      J[4:6, col] <- z
    }
  }
  if (reference == "local") {
    R <- Tt[1:3, 1:3]
    J[1:3, ] <- crossprod(R, J[1:3, , drop = FALSE])
    J[4:6, ] <- crossprod(R, J[4:6, , drop = FALSE])
  } else if (reference == "space") {
    # Twist at the world origin: v_O = v_p + omega x (0 - p) = v_p + p x omega.
    J[1:3, ] <- J[1:3, , drop = FALSE] + so3_hat(p) %*% J[4:6, , drop = FALSE]
  }
  J
}

#' Geometric Jacobians
#'
#' Returns the 6 by n geometric Jacobian of `frame`, mapping joint
#' velocities to the spatial velocity of the frame. Rows are ordered
#' linear first, \eqn{(v_x, v_y, v_z, \omega_x, \omega_y, \omega_z)}, and
#' columns follow the configuration order of [robot_joint_names()].
#' Columns of joints that do not move the frame are zero.
#'
#' The `reference` argument fixes the coordinates:
#' * `"world"` (default): \eqn{v} is the velocity of the frame origin (or
#'   `point`) and \eqn{\omega} the angular velocity, both expressed in
#'   world coordinates. This is the Jacobian used for inverse kinematics.
#' * `"local"`: the same velocities expressed in the axes of `frame` (the
#'   body Jacobian).
#' * `"space"`: the spatial twist, whose linear part is the velocity of
#'   the body point that instantaneously coincides with the world origin
#'   (the space Jacobian of Lynch and Park).
#'
#' For a revolute joint with world axis \eqn{z} through point \eqn{o}, the
#' world column is \eqn{(z \times (p - o), z)}; for a prismatic joint it
#' is \eqn{(z, 0)}. The geometric Jacobian gives angular velocities; it is
#' not the derivative of Euler angles, which is given by
#' [robot_jacobian_analytic()].
#'
#' @param robot A `robot_model`.
#' @param q Configuration.
#' @param frame Frame name (`NULL` for the end effector).
#' @param reference `"world"`, `"local"`, or `"space"`.
#' @param point Optional point offset in the frame coordinates, for the
#'   Jacobian of a point fixed to the frame.
#' @return A 6 by n matrix.
#' @examples
#' robot <- robot_example("planar2")
#' robot_jacobian(robot, c(0.2, 0.4))[1:2, ]
#' @export
robot_jacobian <- function(robot, q, frame = NULL,
                           reference = c("world", "local", "space"),
                           point = NULL) {
  check_robot(robot)
  reference <- match.arg(reference)
  q <- check_q(robot, q)
  if (!is.null(point)) point <- check_numeric_vector(point, 3, "point")
  J <- jacobian_internal(robot, q, frame_index(robot, frame), point, reference)
  dimnames(J) <- list(c("vx", "vy", "vz", "wx", "wy", "wz"),
                      robot_joint_names(robot))
  J
}

#' Finite-difference Jacobian check
#'
#' Computes the world-frame Jacobian of `frame` by central differences on
#' the Lie group: column \eqn{i} is
#' \eqn{(p(q + h e_i) - p(q - h e_i)) / 2h} for the linear part and
#' \eqn{\log(R(q + h e_i) R(q - h e_i)^\top) / 2h} for the angular part.
#' This uses forward kinematics only and is independent of the analytic
#' Jacobian code, so it serves as a validation reference.
#'
#' @inheritParams robot_jacobian
#' @param h Step size.
#' @return A 6 by n matrix.
#' @examples
#' robot <- robot_example("planar3")
#' q <- c(0.1, 0.2, 0.3)
#' max(abs(robot_jacobian(robot, q) - robot_jacobian_numeric(robot, q)))
#' @export
robot_jacobian_numeric <- function(robot, q, frame = NULL, h = 1e-6) {
  check_robot(robot)
  q <- check_q(robot, q)
  k <- frame_index(robot, frame)
  J <- matrix(0, 6, robot$dof)
  for (i in seq_len(robot$dof)) {
    dq <- replace(numeric(robot$dof), i, h)
    Tp <- fk_internal(robot, q + dq)$link[[k]]
    Tm <- fk_internal(robot, q - dq)$link[[k]]
    J[1:3, i] <- (Tp[1:3, 4] - Tm[1:3, 4]) / (2 * h)
    J[4:6, i] <- so3_log(Tp[1:3, 1:3] %*% t(Tm[1:3, 1:3])) / (2 * h)
  }
  dimnames(J) <- list(c("vx", "vy", "vz", "wx", "wy", "wz"),
                      robot_joint_names(robot))
  J
}

#' Analytic Jacobians
#'
#' Maps joint velocities to the time derivative of the position and a
#' minimal orientation parameterization. With `representation = "rpy"` the
#' orientation is the roll-pitch-yaw triple \eqn{(r, p, y)} with
#' \eqn{R = R_z(y) R_y(p) R_x(r)}, and \eqn{\omega = E(r, p, y)
#' (\dot r, \dot p, \dot y)} with columns
#' \eqn{E = [R_z R_y e_x, R_z e_y, e_z]}; the analytic Jacobian is
#' \eqn{J_a = \mathrm{diag}(I, E^{-1}) J}. `E` is singular when
#' \eqn{\cos p = 0}; the function stops there. With
#' `representation = "rotvec"` the orientation is the rotation vector
#' \eqn{\phi} and \eqn{\dot\phi = J_l(\phi)^{-1}\omega}.
#'
#' @inheritParams robot_jacobian
#' @param representation `"rpy"` or `"rotvec"`.
#' @param tol Singularity tolerance on \eqn{|\cos p|}.
#' @return A 6 by n matrix.
#' @examples
#' robot <- robot_example("puma560")
#' robot_jacobian_analytic(robot, c(0, 0.3, -0.2, 0.1, 0.4, 0.2))
#' @export
robot_jacobian_analytic <- function(robot, q, frame = NULL,
                                    representation = c("rpy", "rotvec"),
                                    tol = 1e-8) {
  representation <- match.arg(representation)
  J <- robot_jacobian(robot, q, frame)
  T <- robot_fk(robot, q, frame)
  R <- pose_rotation_matrix(T)
  if (representation == "rpy") {
    e <- matrix_to_euler(R, "ZYX") # (yaw, pitch, roll)
    y <- e[1]
    p <- e[2]
    if (abs(cos(p)) < tol) {
      stop("The roll-pitch-yaw representation is singular at this pose ",
           "(pitch = +/- pi/2).", call. = FALSE)
    }
    E <- cbind(rotz(y) %*% roty(p) %*% c(1, 0, 0), rotz(y) %*% c(0, 1, 0),
               c(0, 0, 1))
    Binv <- solve(E)
    rn <- c("x", "y", "z", "roll", "pitch", "yaw")
  } else {
    Binv <- so3_jacobian_left_inverse(so3_log(R))
    rn <- c("x", "y", "z", "phi_x", "phi_y", "phi_z")
  }
  Ja <- rbind(J[1:3, , drop = FALSE], Binv %*% J[4:6, , drop = FALSE])
  dimnames(Ja) <- list(rn, colnames(J))
  Ja
}
