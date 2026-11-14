# Rotations on SO(2) and SO(3).

check_rotation_matrix <- function(R, tol = 1e-6, name = "R") {
  n <- nrow(R)
  err <- max(abs(crossprod(R) - diag(n)))
  if (err > tol) {
    stop(sprintf("`%s` is not orthonormal (max |R'R - I| = %.3g).",
                 name, err), call. = FALSE)
  }
  d <- det(R)
  if (abs(d - 1) > tol) {
    stop(sprintf("`%s` must have determinant 1 (found %.6g).", name, d),
         call. = FALSE)
  }
  invisible(TRUE)
}

#' Test whether a matrix is a rotation matrix
#'
#' Checks \eqn{R^\top R = I} and \eqn{\det R = 1} within `tol`.
#'
#' @param x A numeric matrix or `robot_rotation`.
#' @param tol Absolute tolerance.
#' @return `TRUE` or `FALSE`.
#' @examples
#' is_rotation_matrix(diag(3))
#' is_rotation_matrix(diag(c(1, 1, -1)))
#' @export
is_rotation_matrix <- function(x, tol = 1e-6) {
  if (inherits(x, "robot_rotation")) return(TRUE)
  if (!is.matrix(x) || !is.numeric(x) || nrow(x) != ncol(x) ||
      !(nrow(x) %in% 2:3) || any(!is.finite(x))) {
    return(FALSE)
  }
  isTRUE(max(abs(crossprod(x) - diag(nrow(x)))) <= tol &&
           abs(det(x) - 1) <= tol)
}

#' Project a matrix onto the nearest rotation
#'
#' Returns the rotation matrix closest to `x` in the Frobenius norm,
#' computed from the singular value decomposition
#' \eqn{x = U S V^\top} as \eqn{U \mathrm{diag}(1, \ldots, \det(UV^\top))
#' V^\top}. Use it to remove drift that accumulates after many
#' compositions.
#'
#' @param x A square 2 by 2 or 3 by 3 numeric matrix.
#' @return A rotation matrix.
#' @examples
#' R <- so3_exp(c(0.1, 0.2, 0.3)) + 1e-4
#' is_rotation_matrix(R, tol = 1e-8)
#' is_rotation_matrix(orthonormalize(R), tol = 1e-12)
#' @export
orthonormalize <- function(x) {
  M <- as_plain_matrix(x)
  if (nrow(M) != ncol(M) || !(nrow(M) %in% 2:3)) {
    stop("`x` must be a 2 by 2 or 3 by 3 matrix.", call. = FALSE)
  }
  s <- svd(M)
  D <- diag(c(rep(1, nrow(M) - 1), sign(det(s$u %*% t(s$v)))))
  s$u %*% D %*% t(s$v)
}

#' Elementary rotation matrices
#'
#' `rotx()`, `roty()` and `rotz()` return 3 by 3 rotation matrices for an
#' active, right-handed rotation by `theta` radians about the x, y and z
#' axes. `rot2()` returns the 2 by 2 planar rotation.
#'
#' @param theta Angle in radians.
#' @return A numeric matrix.
#' @examples
#' rotz(pi / 2) %*% c(1, 0, 0)
#' @name elementary_rotations
NULL

#' @rdname elementary_rotations
#' @export
rotx <- function(theta) {
  theta <- check_scalar(theta, "theta")
  c_ <- cos(theta)
  s_ <- sin(theta)
  matrix(c(1, 0, 0, 0, c_, s_, 0, -s_, c_), 3, 3)
}

#' @rdname elementary_rotations
#' @export
roty <- function(theta) {
  theta <- check_scalar(theta, "theta")
  c_ <- cos(theta)
  s_ <- sin(theta)
  matrix(c(c_, 0, -s_, 0, 1, 0, s_, 0, c_), 3, 3)
}

#' @rdname elementary_rotations
#' @export
rotz <- function(theta) {
  theta <- check_scalar(theta, "theta")
  c_ <- cos(theta)
  s_ <- sin(theta)
  matrix(c(c_, s_, 0, -s_, c_, 0, 0, 0, 1), 3, 3)
}

#' @rdname elementary_rotations
#' @export
rot2 <- function(theta) so2_exp(theta)

axis_rotation <- function(axis_name, theta) {
  switch(axis_name, X = rotx(theta), Y = roty(theta), Z = rotz(theta))
}

euler_sequences <- c("ZYX", "XYZ", "ZYZ", "ZXZ", "XYX", "YZX", "XZY", "YXZ",
                     "ZXY", "XZX", "YXY", "YZY")

check_sequence <- function(sequence) {
  sequence <- toupper(check_string(sequence, "sequence"))
  if (!sequence %in% euler_sequences) {
    stop(sprintf("`sequence` must be one of %s.",
                 paste(euler_sequences, collapse = ", ")), call. = FALSE)
  }
  sequence
}

euler_to_matrix <- function(angles, sequence) {
  ax <- strsplit(sequence, "")[[1]]
  axis_rotation(ax[1], angles[1]) %*% axis_rotation(ax[2], angles[2]) %*%
    axis_rotation(ax[3], angles[3])
}

# Matrix to intrinsic Euler angles for any of the 12 sequences, using the
# quaternion method of Bernardes and Viollet (2022), "Quaternion to Euler
# angles conversion: A direct, general and computationally efficient
# method", PLoS ONE 17(11): e0276302. An intrinsic sequence (a, b, c) with
# angles (t1, t2, t3) equals the extrinsic sequence (c, b, a) with angles
# (t3, t2, t1), which is the form the method uses.
matrix_to_euler <- function(R, sequence) {
  ax <- match(rev(strsplit(sequence, "")[[1]]), c("X", "Y", "Z"))
  q <- matrix_to_quaternion(R)
  i <- ax[1]
  j <- ax[2]
  k <- ax[3]
  proper <- i == k
  if (proper) k <- 6L - i - j
  eps <- (i - j) * (j - k) * (k - i) / 2
  qv <- q[2:4]
  if (proper) {
    a <- q[1]
    b <- qv[i]
    c_ <- qv[j]
    d <- qv[k] * eps
  } else {
    a <- q[1] - qv[j]
    b <- qv[i] + qv[k] * eps
    c_ <- qv[j] + q[1]
    d <- qv[k] * eps - qv[i]
  }
  t2 <- 2 * atan2(sqrt(c_^2 + d^2), sqrt(a^2 + b^2))
  tp <- atan2(b, a)
  tm <- atan2(d, c_)
  if (abs(t2) < 1e-9) {
    t1 <- 0
    t3 <- 2 * tp
  } else if (abs(t2 - pi) < 1e-9) {
    t1 <- 0
    t3 <- 2 * tm
  } else {
    t1 <- tp - tm
    t3 <- tp + tm
  }
  if (!proper) {
    t3 <- eps * t3
    t2 <- t2 - pi / 2
  }
  # Extrinsic angles (t1, t2, t3) about (i, j, k); return intrinsic order.
  out <- c(t3, t2, t1)
  if (!proper) out <- c(t3, t2, t1)
  wrap_angle_internal(out)
}
