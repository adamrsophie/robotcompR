# Lie group and Lie algebra operations on SO(2), SO(3), SE(2) and SE(3).
#
# Conventions (see vignette("conventions")):
# * so(3) vectors are angular velocity or rotation vectors w = theta * u.
# * se(3) twists are ordered linear first: xi = c(v, w).
# * se(2) twists are ordered c(vx, vy, w).
# * All maps use closed forms away from the identity and Taylor series
#   for small angles, so that results stay accurate to machine precision.

small_angle <- 1e-5

as_plain_matrix <- function(x) {
  if (inherits(x, c("robot_rotation", "robot_pose"))) {
    return(unclass_matrix(x))
  }
  if (is.matrix(x)) {
    storage.mode(x) <- "double"
    return(x)
  }
  stop("Expected a matrix, robot_rotation or robot_pose.", call. = FALSE)
}

unclass_matrix <- function(x) {
  m <- x$matrix
  dimnames(m) <- NULL
  m
}

# Coefficients A = sin(t)/t, B = (1 - cos t)/t^2, C = (t - sin t)/t^3.
so3_coefficients <- function(theta) {
  t2 <- theta * theta
  if (theta < small_angle) {
    A <- 1 - t2 / 6 + t2 * t2 / 120
    B <- 0.5 - t2 / 24 + t2 * t2 / 720
    C <- 1 / 6 - t2 / 120 + t2 * t2 / 5040
  } else {
    A <- sin(theta) / theta
    B <- (1 - cos(theta)) / t2
    C <- (theta - sin(theta)) / (t2 * theta)
  }
  list(A = A, B = B, C = C)
}

#' Hat and vee operators
#'
#' `so3_hat()` maps a 3-vector to the skew-symmetric matrix with
#' `so3_hat(w) %*% v == cross(w, v)`; `so3_vee()` is its inverse.
#' `se3_hat()` maps a linear-first twist `c(v, w)` to the 4 by 4 matrix
#' `[[hat(w), v], [0, 0]]`, and `se3_vee()` inverts it. `so2_hat()`,
#' `se2_hat()` and their vee operators are the planar analogues, with
#' planar twists ordered `c(vx, vy, w)`.
#'
#' @param x Numeric vector (hat) or matrix (vee).
#' @param tol Tolerance used by vee operators to check skew symmetry.
#' @return A matrix (hat) or numeric vector (vee).
#' @examples
#' W <- so3_hat(c(1, 2, 3))
#' so3_vee(W)
#' se3_vee(se3_hat(c(1, 0, 0, 0, 0, 1)))
#' @name hat_vee
NULL

#' @rdname hat_vee
#' @export
so3_hat <- function(x) {
  w <- check_numeric_vector(x, 3, "x")
  matrix(c(0, w[3], -w[2],
           -w[3], 0, w[1],
           w[2], -w[1], 0), 3, 3)
}

#' @rdname hat_vee
#' @export
so3_vee <- function(x, tol = 1e-8) {
  W <- check_matrix(as_plain_matrix(x), 3, 3, "x")
  if (max(abs(W + t(W))) > tol * max(1, max(abs(W)))) {
    stop("`x` is not skew-symmetric.", call. = FALSE)
  }
  c(W[3, 2] - W[2, 3], W[1, 3] - W[3, 1], W[2, 1] - W[1, 2]) / 2
}

#' @rdname hat_vee
#' @export
se3_hat <- function(x) {
  xi <- check_numeric_vector(x, 6, "x")
  M <- matrix(0, 4, 4)
  M[1:3, 1:3] <- so3_hat(xi[4:6])
  M[1:3, 4] <- xi[1:3]
  M
}

#' @rdname hat_vee
#' @export
se3_vee <- function(x, tol = 1e-8) {
  M <- check_matrix(as_plain_matrix(x), 4, 4, "x")
  if (max(abs(M[4, ])) > tol) {
    stop("The last row of `x` must be zero.", call. = FALSE)
  }
  c(M[1:3, 4], so3_vee(M[1:3, 1:3], tol))
}

#' @rdname hat_vee
#' @export
so2_hat <- function(x) {
  th <- check_scalar(x, "x")
  matrix(c(0, th, -th, 0), 2, 2)
}

#' @rdname hat_vee
#' @export
so2_vee <- function(x, tol = 1e-8) {
  W <- check_matrix(as_plain_matrix(x), 2, 2, "x")
  if (max(abs(W + t(W))) > tol * max(1, max(abs(W)))) {
    stop("`x` is not skew-symmetric.", call. = FALSE)
  }
  (W[2, 1] - W[1, 2]) / 2
}

#' @rdname hat_vee
#' @export
se2_hat <- function(x) {
  xi <- check_numeric_vector(x, 3, "x")
  M <- matrix(0, 3, 3)
  M[1:2, 1:2] <- so2_hat(xi[3])
  M[1:2, 3] <- xi[1:2]
  M
}

#' @rdname hat_vee
#' @export
se2_vee <- function(x, tol = 1e-8) {
  M <- check_matrix(as_plain_matrix(x), 3, 3, "x")
  if (max(abs(M[3, ])) > tol) {
    stop("The last row of `x` must be zero.", call. = FALSE)
  }
  c(M[1:2, 3], so2_vee(M[1:2, 1:2], tol))
}

#' Exponential and logarithm maps of SO(3) and SE(3)
#'
#' `so3_exp()` maps a rotation vector \eqn{\omega = \theta u} to the
#' rotation matrix \eqn{\exp(\hat\omega)} using the Rodrigues formula, with
#' Taylor expansions of its coefficients when \eqn{\theta < 10^{-5}}.
#' `so3_log()` returns the rotation vector with \eqn{\theta \in [0, \pi]}.
#' The angle is computed with `atan2()` instead of `acos()`, and for
#' angles within \eqn{10^{-3}} of \eqn{\pi} the axis is recovered from the
#' symmetric part of the matrix, so the map stays accurate near the cut
#' locus where the antisymmetric part vanishes. At exactly \eqn{\pi} the
#' two vectors \eqn{\pm\pi u} are equally valid; the one returned is
#' deterministic but arbitrary.
#'
#' `se3_exp()` maps a linear-first twist \eqn{\xi = (v, \omega)} to a
#' homogeneous transform \eqn{[\exp(\hat\omega), J_l(\omega) v; 0, 1]},
#' where \eqn{J_l} is the left Jacobian of SO(3). `se3_log()` inverts it.
#'
#' @param x Rotation vector or twist (for `exp`), or a rotation matrix,
#'   homogeneous transform, `robot_rotation` or `robot_pose` (for `log`).
#' @param tol Tolerance for validating the input group element.
#' @return A matrix (exp) or numeric vector (log).
#' @examples
#' R <- so3_exp(c(0, 0, pi / 2))
#' so3_log(R)
#' T <- se3_exp(c(1, 0, 0, 0, 0, pi / 2))
#' se3_log(T)
#' @name lie_exp_log
NULL

#' @rdname lie_exp_log
#' @export
so3_exp <- function(x) {
  w <- check_numeric_vector(x, 3, "x")
  theta <- vnorm(w)
  k <- so3_coefficients(theta)
  W <- so3_hat(w)
  diag(3) + k$A * W + k$B * (W %*% W)
}

#' @rdname lie_exp_log
#' @export
so3_log <- function(x, tol = 1e-6) {
  R <- check_matrix(as_plain_matrix(x), 3, 3, "x")
  check_rotation_matrix(R, tol, "x")
  a <- c(R[3, 2] - R[2, 3], R[1, 3] - R[3, 1], R[2, 1] - R[1, 2]) / 2
  s <- vnorm(a)
  cth <- (sum(diag(R)) - 1) / 2
  theta <- atan2(s, cth)
  if (theta < small_angle) {
    t2 <- theta * theta
    return(a * (1 + t2 / 6 + 7 * t2 * t2 / 360))
  }
  if (theta < pi - 1e-3) {
    return(a * theta / s)
  }
  S <- (R + t(R)) / 2
  B <- (S - cth * diag(3)) / (1 - cth)
  k <- which.max(diag(B))
  u <- B[, k] / sqrt(max(B[k, k], .Machine$double.eps))
  u <- u / vnorm(u)
  if (sum(u * a) < 0) u <- -u
  theta * u
}

#' @rdname lie_exp_log
#' @export
se3_exp <- function(x) {
  xi <- check_numeric_vector(x, 6, "x")
  w <- xi[4:6]
  T <- diag(4)
  T[1:3, 1:3] <- so3_exp(w)
  T[1:3, 4] <- so3_jacobian_left(w) %*% xi[1:3]
  T
}

#' @rdname lie_exp_log
#' @export
se3_log <- function(x, tol = 1e-6) {
  T <- check_matrix(as_plain_matrix(x), 4, 4, "x")
  check_transform_matrix(T, tol, "x")
  w <- so3_log(T[1:3, 1:3], tol)
  v <- so3_jacobian_left_inverse(w) %*% T[1:3, 4]
  c(as.numeric(v), w)
}

#' Exponential and logarithm maps of SO(2) and SE(2)
#'
#' `so2_exp(theta)` returns the 2 by 2 rotation matrix; `so2_log()` returns
#' the angle in \eqn{(-\pi, \pi]}. `se2_exp()` maps a planar twist
#' `c(vx, vy, w)` to a 3 by 3 homogeneous transform, and `se2_log()`
#' inverts it.
#'
#' @param x Angle or twist (exp), or a matrix or planar object (log).
#' @param tol Tolerance for validating group elements.
#' @return A matrix (exp) or numeric value or vector (log).
#' @examples
#' se2_log(se2_exp(c(1, 0, pi / 2)))
#' @name lie_exp_log_2d
NULL

#' @rdname lie_exp_log_2d
#' @export
so2_exp <- function(x) {
  th <- check_scalar(x, "x")
  matrix(c(cos(th), sin(th), -sin(th), cos(th)), 2, 2)
}

#' @rdname lie_exp_log_2d
#' @export
so2_log <- function(x, tol = 1e-6) {
  R <- check_matrix(as_plain_matrix(x), 2, 2, "x")
  check_rotation_matrix(R, tol, "x")
  atan2(R[2, 1], R[1, 1])
}

se2_V <- function(theta) {
  if (abs(theta) < small_angle) {
    t2 <- theta * theta
    a <- 1 - t2 / 6
    b <- theta / 2 - theta * t2 / 24
  } else {
    a <- sin(theta) / theta
    b <- (1 - cos(theta)) / theta
  }
  matrix(c(a, b, -b, a), 2, 2)
}

#' @rdname lie_exp_log_2d
#' @export
se2_exp <- function(x) {
  xi <- check_numeric_vector(x, 3, "x")
  T <- diag(3)
  T[1:2, 1:2] <- so2_exp(xi[3])
  T[1:2, 3] <- se2_V(xi[3]) %*% xi[1:2]
  T
}

#' @rdname lie_exp_log_2d
#' @export
se2_log <- function(x, tol = 1e-6) {
  T <- check_matrix(as_plain_matrix(x), 3, 3, "x")
  check_transform_matrix(T, tol, "x")
  th <- atan2(T[2, 1], T[1, 1])
  v <- solve(se2_V(th), T[1:2, 3])
  c(as.numeric(v), th)
}

#' Jacobians of SO(3)
#'
#' The left Jacobian \eqn{J_l(\omega) = I + \frac{1 - \cos\theta}{\theta^2}
#' \hat\omega + \frac{\theta - \sin\theta}{\theta^3}\hat\omega^2} satisfies
#' \eqn{\exp(\widehat{\omega + \delta}) \approx \exp(\widehat{J_l \delta})
#' \exp(\hat\omega)} for small \eqn{\delta}. The right Jacobian is
#' \eqn{J_r(\omega) = J_l(-\omega)}. Inverses use closed forms with Taylor
#' expansions near zero. The inverses are singular at
#' \eqn{\theta = 2\pi k}, \eqn{k \ne 0}; rotation vectors returned by
#' [so3_log()] have \eqn{\theta \le \pi} and never reach that singularity.
#'
#' @param x Rotation vector.
#' @return A 3 by 3 matrix.
#' @examples
#' w <- c(0.1, -0.2, 0.3)
#' so3_jacobian_left(w) %*% so3_jacobian_left_inverse(w)
#' @name so3_jacobians
NULL

#' @rdname so3_jacobians
#' @export
so3_jacobian_left <- function(x) {
  w <- check_numeric_vector(x, 3, "x")
  k <- so3_coefficients(vnorm(w))
  W <- so3_hat(w)
  diag(3) + k$B * W + k$C * (W %*% W)
}

#' @rdname so3_jacobians
#' @export
so3_jacobian_right <- function(x) {
  so3_jacobian_left(-check_numeric_vector(x, 3, "x"))
}

#' @rdname so3_jacobians
#' @export
so3_jacobian_left_inverse <- function(x) {
  w <- check_numeric_vector(x, 3, "x")
  theta <- vnorm(w)
  t2 <- theta * theta
  if (theta < small_angle) {
    D <- 1 / 12 + t2 / 720 + t2 * t2 / 30240
  } else {
    D <- 1 / t2 - 1 / (2 * theta * tan(theta / 2))
  }
  W <- so3_hat(w)
  diag(3) - 0.5 * W + D * (W %*% W)
}

#' @rdname so3_jacobians
#' @export
so3_jacobian_right_inverse <- function(x) {
  so3_jacobian_left_inverse(-check_numeric_vector(x, 3, "x"))
}

#' Adjoint representation of SE(3) and SE(2)
#'
#' For \eqn{T = (R, p)}, the adjoint maps a linear-first twist expressed
#' in the child frame to the same twist expressed in the parent frame:
#' \eqn{Ad_T = [R, \hat p R; 0, R]}. For SE(2) with twists
#' `c(vx, vy, w)`, \eqn{Ad_T = [R, (p_y, -p_x)^T; 0, 1]}.
#'
#' @param x A homogeneous transform or `robot_pose`.
#' @return A 6 by 6 (SE(3)) or 3 by 3 (SE(2)) matrix.
#' @examples
#' se3_adjoint(se3_exp(c(1, 2, 3, 0.1, 0.2, 0.3)))
#' @export
se3_adjoint <- function(x) {
  T <- check_matrix(as_plain_matrix(x), 4, 4, "x")
  R <- T[1:3, 1:3]
  p <- T[1:3, 4]
  Ad <- matrix(0, 6, 6)
  Ad[1:3, 1:3] <- R
  Ad[4:6, 4:6] <- R
  Ad[1:3, 4:6] <- so3_hat(p) %*% R
  Ad
}

#' @rdname se3_adjoint
#' @export
se2_adjoint <- function(x) {
  T <- check_matrix(as_plain_matrix(x), 3, 3, "x")
  Ad <- matrix(0, 3, 3)
  Ad[1:2, 1:2] <- T[1:2, 1:2]
  Ad[1:2, 3] <- c(T[2, 3], -T[1, 3])
  Ad[3, 3] <- 1
  Ad
}
