# Unit quaternions.
#
# Convention: scalar-first q = c(w, x, y, z), Hamilton product, right-handed,
# active rotation: R(q) v = q * (0, v) * conj(q). q and -q represent the same
# rotation (double cover); constructors return the representative with w >= 0.

quaternion_product <- function(a, b) {
  c(a[1] * b[1] - a[2] * b[2] - a[3] * b[3] - a[4] * b[4],
    a[1] * b[2] + a[2] * b[1] + a[3] * b[4] - a[4] * b[3],
    a[1] * b[3] - a[2] * b[4] + a[3] * b[1] + a[4] * b[2],
    a[1] * b[4] + a[2] * b[3] - a[3] * b[2] + a[4] * b[1])
}

quaternion_to_matrix <- function(q) {
  w <- q[1]; x <- q[2]; y <- q[3]; z <- q[4]
  matrix(c(1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y),
           2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x),
           2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
         3, 3)
}

# Shepperd's method: pick the largest of the four candidate pivots so the
# division is always well conditioned.
matrix_to_quaternion <- function(R) {
  tr <- R[1, 1] + R[2, 2] + R[3, 3]
  cand <- c(tr, R[1, 1], R[2, 2], R[3, 3])
  k <- which.max(cand)
  if (k == 1) {
    s <- 2 * sqrt(1 + tr)
    q <- c(s / 4, (R[3, 2] - R[2, 3]) / s, (R[1, 3] - R[3, 1]) / s,
           (R[2, 1] - R[1, 2]) / s)
  } else if (k == 2) {
    s <- 2 * sqrt(1 + R[1, 1] - R[2, 2] - R[3, 3])
    q <- c((R[3, 2] - R[2, 3]) / s, s / 4, (R[1, 2] + R[2, 1]) / s,
           (R[1, 3] + R[3, 1]) / s)
  } else if (k == 3) {
    s <- 2 * sqrt(1 + R[2, 2] - R[1, 1] - R[3, 3])
    q <- c((R[1, 3] - R[3, 1]) / s, (R[1, 2] + R[2, 1]) / s, s / 4,
           (R[2, 3] + R[3, 2]) / s)
  } else {
    s <- 2 * sqrt(1 + R[3, 3] - R[1, 1] - R[2, 2])
    q <- c((R[2, 1] - R[1, 2]) / s, (R[1, 3] + R[3, 1]) / s,
           (R[2, 3] + R[3, 2]) / s, s / 4)
  }
  q <- q / vnorm(q)
  canonical_quaternion(q)
}

canonical_quaternion <- function(q) {
  if (q[1] < 0 || (q[1] == 0 && q[which(q != 0)[1]] < 0)) -q else q
}

new_quaternion <- function(q) {
  structure(list(q = as.numeric(q)), class = "robot_quaternion")
}

#' Unit quaternions
#'
#' Creates a unit quaternion with scalar-first component order
#' \eqn{q = (w, x, y, z)}. Quaternions use the Hamilton product and
#' represent active, right-handed rotations:
#' \eqn{R(q)v = q \otimes (0, v) \otimes q^*}. Because \eqn{q} and
#' \eqn{-q} represent the same rotation, the constructor normalizes the
#' input and returns the representative with \eqn{w \ge 0}. Composition
#' order matches rotation matrices: `R(q1 * q2) = R(q1) %*% R(q2)`.
#'
#' @param x A length-4 numeric vector `c(w, x, y, z)`, a rotation matrix,
#'   or a `robot_rotation`. Vectors are normalized; a zero vector is an
#'   error.
#' @param canonical If `TRUE` (default) return the representative with
#'   nonnegative scalar part.
#' @return A `robot_quaternion`.
#' @examples
#' q <- robot_quaternion(c(cos(pi / 4), 0, 0, sin(pi / 4)))
#' as.matrix(robot_rotation(q))
#' q * quaternion_conjugate(q)
#' @export
robot_quaternion <- function(x, canonical = TRUE) {
  if (inherits(x, "robot_quaternion")) return(x)
  if (inherits(x, "robot_rotation")) {
    if (x$dim != 3) stop("Quaternions represent 3D rotations only.",
                         call. = FALSE)
    return(new_quaternion(matrix_to_quaternion(x$matrix)))
  }
  if (is.matrix(x)) {
    R <- check_matrix(x, 3, 3, "x")
    check_rotation_matrix(R, 1e-6, "x")
    return(new_quaternion(matrix_to_quaternion(R)))
  }
  q <- check_numeric_vector(x, 4, "x")
  n <- vnorm(q)
  if (n < 1e-12) stop("`x` must be a nonzero quaternion.", call. = FALSE)
  q <- q / n
  if (canonical) q <- canonical_quaternion(q)
  new_quaternion(q)
}

#' @export
as.numeric.robot_quaternion <- function(x, ...) x$q

#' @export
as.double.robot_quaternion <- function(x, ...) x$q

#' @export
print.robot_quaternion <- function(x, digits = 6, ...) {
  q <- signif(x$q, digits)
  cat(sprintf("<robot_quaternion> w = %s, (x, y, z) = (%s, %s, %s)\n",
              q[1], q[2], q[3], q[4]))
  invisible(x)
}

#' @export
Ops.robot_quaternion <- function(e1, e2) {
  if (.Generic != "*") {
    stop("Only `*` (Hamilton product) is defined for quaternions.",
         call. = FALSE)
  }
  a <- robot_quaternion(e1)
  b <- robot_quaternion(e2)
  q <- quaternion_product(a$q, b$q)
  new_quaternion(q / vnorm(q))
}

#' Quaternion operations
#'
#' `quaternion_conjugate()` returns \eqn{(w, -x, -y, -z)}, the inverse of a
#' unit quaternion. `quaternion_rotate()` rotates the columns of a 3 by n
#' matrix (or one 3-vector). `quaternion_distance()` returns the rotation
#' angle between two orientations in \eqn{[0, \pi]}, which is invariant to
#' the sign ambiguity.
#'
#' @param q,q1,q2 `robot_quaternion` objects or length-4 vectors.
#' @param v A 3-vector or 3 by n matrix.
#' @return A quaternion, matrix, vector or number.
#' @examples
#' q <- robot_quaternion(robot_rotation(axis = c(0, 0, 1), angle = pi / 2))
#' quaternion_rotate(q, c(1, 0, 0))
#' quaternion_distance(q, robot_quaternion(c(1, 0, 0, 0)))
#' @name quaternion_ops
NULL

#' @rdname quaternion_ops
#' @export
quaternion_conjugate <- function(q) {
  q <- robot_quaternion(q, canonical = FALSE)$q
  new_quaternion(c(q[1], -q[2:4]))
}

#' @rdname quaternion_ops
#' @export
quaternion_rotate <- function(q, v) {
  R <- quaternion_to_matrix(robot_quaternion(q, canonical = FALSE)$q)
  out <- R %*% (if (is.matrix(v)) check_matrix(v, 3, NULL, "v")
                else check_numeric_vector(v, 3, "v"))
  if (is.matrix(v)) out else as.numeric(out)
}

#' @rdname quaternion_ops
#' @export
quaternion_distance <- function(q1, q2) {
  a <- robot_quaternion(q1, canonical = FALSE)$q
  b <- robot_quaternion(q2, canonical = FALSE)$q
  # Angle of the relative rotation a* b, via atan2 so that small angles
  # keep full relative precision (acos of a dot product near 1 does not).
  w <- sum(a * b)
  v <- c(a[1] * b[2] - a[2] * b[1] - a[3] * b[4] + a[4] * b[3],
         a[1] * b[3] + a[2] * b[4] - a[3] * b[1] - a[4] * b[2],
         a[1] * b[4] - a[2] * b[3] + a[3] * b[2] - a[4] * b[1])
  2 * atan2(sqrt(sum(v * v)), abs(w))
}

#' Spherical linear interpolation of rotations
#'
#' Interpolates between two orientations along the shortest great circle:
#' \eqn{q(s) = q_1 (q_1^{*} q_2)^s} for \eqn{s \in [0, 1]}. The sign of
#' \eqn{q_2} is flipped when needed so the path is the shorter of the two
#' arcs that represent the same pair of rotations. For nearly identical
#' inputs the method falls back to normalized linear interpolation.
#'
#' @param q1,q2 Start and end orientations: quaternions, rotation matrices
#'   or `robot_rotation` objects.
#' @param s Interpolation parameter(s) in \eqn{[0, 1]}.
#' @return A `robot_quaternion` when `s` is a scalar, otherwise a list.
#' @examples
#' qa <- robot_quaternion(c(1, 0, 0, 0))
#' qb <- robot_quaternion(robot_rotation(axis = c(0, 0, 1), angle = pi / 2))
#' quaternion_slerp(qa, qb, 0.5)
#' @export
quaternion_slerp <- function(q1, q2, s) {
  a <- robot_quaternion(q1)$q
  b <- robot_quaternion(q2)$q
  s <- check_numeric_vector(s, name = "s")
  if (any(s < 0 | s > 1)) stop("`s` must lie in [0, 1].", call. = FALSE)
  d <- sum(a * b)
  if (d < 0) {
    b <- -b
    d <- -d
  }
  one <- function(si) {
    if (d > 1 - 1e-12) {
      q <- (1 - si) * a + si * b
    } else {
      om <- acos(min(1, d))
      q <- (sin((1 - si) * om) * a + sin(si * om) * b) / sin(om)
    }
    robot_quaternion(q)
  }
  if (length(s) == 1) one(s) else lapply(s, one)
}
