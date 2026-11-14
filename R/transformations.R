# robot_rotation and robot_pose objects, composition, inversion and
# point transformation.

new_rotation <- function(R) {
  R <- unname(R)
  structure(list(matrix = R, dim = nrow(R)), class = "robot_rotation")
}

#' Rotations in two and three dimensions
#'
#' Creates a validated rotation, an element of SO(2) or SO(3). Exactly one
#' representation must be supplied:
#'
#' * `x`: a rotation matrix, `robot_rotation`, or `robot_quaternion`;
#' * `axis` and `angle`: active right-handed rotation by `angle` radians
#'   about the unit vector `axis` (any nonzero vector is normalized);
#' * `angle` alone: a planar rotation (SO(2));
#' * `euler` with `sequence`: intrinsic Euler angles, so
#'   `sequence = "ZYX"` and `euler = c(a, b, c)` gives
#'   \eqn{R_z(a) R_y(b) R_x(c)} (yaw, pitch, roll). The URDF
#'   `rpy = c(r, p, y)` attribute corresponds to `sequence = "ZYX"` with
#'   `euler = c(y, p, r)`;
#' * `rotvec`: a rotation vector \eqn{\theta u};
#' * `quaternion`: scalar-first `c(w, x, y, z)`.
#'
#' Matrices are checked for \eqn{R^\top R = I} and \eqn{\det R = 1} within
#' `tol`; use [orthonormalize()] first to repair accumulated drift.
#'
#' @param x Optional matrix, `robot_rotation`, or `robot_quaternion`.
#' @param axis Rotation axis (length 3).
#' @param angle Rotation angle in radians.
#' @param euler Three Euler angles in radians.
#' @param sequence Intrinsic axis sequence, one of the 12 valid sequences
#'   such as `"ZYX"`, `"XYZ"` or `"ZYZ"`.
#' @param rotvec Rotation vector (length 3).
#' @param quaternion Scalar-first quaternion (length 4).
#' @param tol Tolerance for matrix validation.
#' @return A `robot_rotation` object.
#' @examples
#' R <- robot_rotation(axis = c(0, 0, 1), angle = pi / 2)
#' R
#' robot_rotation(euler = c(0.3, -0.2, 0.1), sequence = "ZYX")
#' robot_rotation(angle = pi / 6)
#' @export
robot_rotation <- function(x = NULL, axis = NULL, angle = NULL, euler = NULL,
                           sequence = "ZYX", rotvec = NULL,
                           quaternion = NULL, tol = 1e-6) {
  given <- c(x = !is.null(x), axis = !is.null(axis), euler = !is.null(euler),
             rotvec = !is.null(rotvec), quaternion = !is.null(quaternion))
  if (sum(given) == 0 && !is.null(angle)) {
    return(new_rotation(so2_exp(angle)))
  }
  if (sum(given) != 1) {
    stop("Supply exactly one rotation representation.", call. = FALSE)
  }
  if (given["x"]) {
    if (inherits(x, "robot_rotation")) return(x)
    if (inherits(x, "robot_quaternion")) {
      return(new_rotation(quaternion_to_matrix(x$q)))
    }
    R <- check_matrix(as_plain_matrix(x), name = "x")
    if (!(nrow(R) %in% 2:3) || nrow(R) != ncol(R)) {
      stop("`x` must be a 2 by 2 or 3 by 3 matrix.", call. = FALSE)
    }
    check_rotation_matrix(R, tol, "x")
    return(new_rotation(R))
  }
  if (given["axis"]) {
    if (is.null(angle)) stop("`angle` is required with `axis`.", call. = FALSE)
    u <- unit_vector(check_numeric_vector(axis, 3, "axis"))
    return(new_rotation(so3_exp(u * check_scalar(angle, "angle"))))
  }
  if (given["euler"]) {
    e <- check_numeric_vector(euler, 3, "euler")
    return(new_rotation(euler_to_matrix(e, check_sequence(sequence))))
  }
  if (given["rotvec"]) {
    return(new_rotation(so3_exp(check_numeric_vector(rotvec, 3, "rotvec"))))
  }
  q <- robot_quaternion(quaternion)
  new_rotation(quaternion_to_matrix(q$q))
}

#' @export
as.matrix.robot_rotation <- function(x, ...) x$matrix

#' @export
print.robot_rotation <- function(x, digits = 4, ...) {
  cat(sprintf("<robot_rotation> SO(%d)\n", x$dim))
  if (x$dim == 3) {
    w <- so3_log(x$matrix)
    cat(sprintf("angle %.6g rad about axis (%s)\n", vnorm(w),
                if (vnorm(w) > 0) paste(signif(w / vnorm(w), digits),
                                        collapse = ", ") else "any"))
  } else {
    cat(sprintf("angle %.6g rad\n", so2_log(x$matrix)))
  }
  print(round(x$matrix, digits))
  invisible(x)
}

#' Convert a rotation to other representations
#'
#' `as_euler()` returns intrinsic Euler angles for `sequence` (see
#' [robot_rotation()]); the middle angle lies in \eqn{[-\pi/2, \pi/2]} for
#' Tait-Bryan sequences and \eqn{[0, \pi]} for proper Euler sequences, and
#' the outer angles lie in \eqn{[-\pi, \pi)}. At gimbal lock only the sum
#' or difference of the outer angles is determined; the third (last)
#' angle is then set to zero. `as_axis_angle()` returns
#' a list with a unit `axis` and `angle` in \eqn{[0, \pi]}; the axis is
#' `c(0, 0, 1)` for the identity. `as_rotvec()` returns
#' \eqn{\theta u}. For planar rotations `as_axis_angle()` returns the
#' signed angle.
#'
#' @param x A `robot_rotation`, `robot_pose`, `robot_quaternion` or matrix.
#' @param sequence Intrinsic Euler sequence.
#' @return Numeric vector or list.
#' @examples
#' R <- robot_rotation(euler = c(0.3, -0.2, 0.1), sequence = "ZYX")
#' as_euler(R, "ZYX")
#' as_axis_angle(R)
#' @name rotation_conversions
NULL

rotation_matrix_of <- function(x) {
  if (inherits(x, "robot_pose")) return(pose_rotation_matrix(x))
  robot_rotation(x)$matrix
}

#' @rdname rotation_conversions
#' @export
as_euler <- function(x, sequence = "ZYX") {
  R <- rotation_matrix_of(x)
  if (nrow(R) != 3) stop("Euler angles need a 3D rotation.", call. = FALSE)
  matrix_to_euler(R, check_sequence(sequence))
}

#' @rdname rotation_conversions
#' @export
as_axis_angle <- function(x) {
  R <- rotation_matrix_of(x)
  if (nrow(R) == 2) return(list(angle = so2_log(R)))
  w <- so3_log(R)
  th <- vnorm(w)
  list(axis = if (th > 0) w / th else c(0, 0, 1), angle = th)
}

#' @rdname rotation_conversions
#' @export
as_rotvec <- function(x) {
  R <- rotation_matrix_of(x)
  if (nrow(R) != 3) stop("Rotation vectors need a 3D rotation.",
                         call. = FALSE)
  so3_log(R)
}

#' @export
Ops.robot_rotation <- function(e1, e2) {
  if (.Generic != "*") {
    stop("Only `*` (composition) is defined for rotations.", call. = FALSE)
  }
  if (inherits(e1, "robot_rotation") && inherits(e2, "robot_rotation")) {
    if (e1$dim != e2$dim) stop("Rotation dimensions differ.", call. = FALSE)
    return(new_rotation(e1$matrix %*% e2$matrix))
  }
  stop("Both operands of `*` must be robot_rotation objects; use ",
       "robot_transform_point() to rotate points.", call. = FALSE)
}

new_pose <- function(M, parent = NULL, child = NULL) {
  M <- unname(M)
  d <- nrow(M) - 1L
  M[d + 1, ] <- c(rep(0, d), 1)
  structure(list(matrix = M, dim = d, parent = parent, child = child),
            class = "robot_pose")
}

check_transform_matrix <- function(T, tol = 1e-6, name = "T") {
  d <- nrow(T) - 1
  if (max(abs(T[d + 1, ] - c(rep(0, d), 1))) > tol) {
    stop(sprintf("The last row of `%s` must be (0, ..., 0, 1).", name),
         call. = FALSE)
  }
  check_rotation_matrix(T[1:d, 1:d, drop = FALSE], tol, name)
}

#' Rigid-body poses in SE(2) and SE(3)
#'
#' A pose is a homogeneous transform \eqn{T = [R, p; 0, 1]}. The pose
#' \eqn{{}^{a}T_{b}} gives the position and orientation of frame `b`
#' (`child`) relative to frame `a` (`parent`), and maps coordinates of a
#' point expressed in `b` into `a`: \eqn{{}^{a}p = {}^{a}T_{b}\,{}^{b}p}.
#' Composition follows matrix multiplication:
#' \eqn{{}^{a}T_{c} = {}^{a}T_{b}\,{}^{b}T_{c}}, written `T_ab * T_bc`.
#' When both operands carry frame names, composition checks that the
#' child of the left operand equals the parent of the right operand.
#'
#' @param rotation A `robot_rotation`, rotation matrix, or quaternion.
#'   Defaults to the identity of the dimension implied by `translation`.
#' @param translation Translation vector of length 2 or 3 (meters).
#' @param matrix Alternatively, a full homogeneous matrix.
#' @param parent,child Optional frame names.
#' @param tol Validation tolerance.
#' @return A `robot_pose` object.
#' @examples
#' T <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2),
#'                 c(1, 2, 3))
#' robot_transform_point(T, c(1, 0, 0))
#' robot_inverse(T) * T
#' @export
robot_pose <- function(rotation = NULL, translation = NULL, matrix = NULL,
                       parent = NULL, child = NULL, tol = 1e-6) {
  if (!is.null(parent)) parent <- check_string(parent, "parent")
  if (!is.null(child)) child <- check_string(child, "child")
  if (!is.null(matrix)) {
    if (!is.null(rotation) || !is.null(translation)) {
      stop("Supply either `matrix` or `rotation`/`translation`.",
           call. = FALSE)
    }
    if (inherits(matrix, "robot_pose")) {
      return(new_pose(matrix$matrix, parent %||% matrix$parent,
                      child %||% matrix$child))
    }
    M <- check_matrix(matrix, name = "matrix")
    if (!(nrow(M) %in% 3:4) || nrow(M) != ncol(M)) {
      stop("`matrix` must be 3 by 3 (SE(2)) or 4 by 4 (SE(3)).",
           call. = FALSE)
    }
    check_transform_matrix(M, tol, "matrix")
    return(new_pose(M, parent, child))
  }
  if (is.null(rotation)) {
    d <- if (is.null(translation)) 3L else length(translation)
    R <- diag(d)
  } else {
    R <- robot_rotation(rotation, tol = tol)$matrix
    d <- nrow(R)
  }
  p <- if (is.null(translation)) rep(0, d)
       else check_numeric_vector(translation, name = "translation")
  if (length(p) != d) {
    stop(sprintf("`translation` must have length %d.", d), call. = FALSE)
  }
  if (!(d %in% 2:3)) stop("Poses must be 2D or 3D.", call. = FALSE)
  M <- diag(d + 1)
  M[1:d, 1:d] <- R
  M[1:d, d + 1] <- p
  new_pose(M, parent, child)
}

pose_rotation_matrix <- function(T) {
  T$matrix[1:T$dim, 1:T$dim, drop = FALSE]
}

pose_translation_vector <- function(T) T$matrix[1:T$dim, T$dim + 1]

#' Pose components
#'
#' `pose_rotation()` returns the rotation part as a `robot_rotation`, and
#' `pose_translation()` the translation vector.
#'
#' @param x A `robot_pose`.
#' @return A `robot_rotation` or numeric vector.
#' @examples
#' T <- robot_pose(translation = c(1, 2, 3))
#' pose_translation(T)
#' @name pose_components
NULL

#' @rdname pose_components
#' @export
pose_rotation <- function(x) {
  stopifnot(inherits(x, "robot_pose"))
  new_rotation(pose_rotation_matrix(x))
}

#' @rdname pose_components
#' @export
pose_translation <- function(x) {
  stopifnot(inherits(x, "robot_pose"))
  pose_translation_vector(x)
}

#' @export
as.matrix.robot_pose <- function(x, ...) x$matrix

#' @export
print.robot_pose <- function(x, digits = 4, ...) {
  frames <- if (!is.null(x$parent) || !is.null(x$child)) {
    sprintf(" %s <- %s", x$parent %||% "?", x$child %||% "?")
  } else ""
  cat(sprintf("<robot_pose> SE(%d)%s\n", x$dim, frames))
  print(round(x$matrix, digits))
  invisible(x)
}

compose_two <- function(a, b) {
  if (inherits(a, "robot_rotation")) a <- robot_pose(a)
  if (inherits(b, "robot_rotation")) b <- robot_pose(b)
  if (!inherits(a, "robot_pose") || !inherits(b, "robot_pose")) {
    stop("Composition needs robot_pose or robot_rotation objects.",
         call. = FALSE)
  }
  if (a$dim != b$dim) stop("Pose dimensions differ.", call. = FALSE)
  if (!is.null(a$child) && !is.null(b$parent) && a$child != b$parent) {
    stop(sprintf("Frame mismatch: left pose ends in '%s' but right pose starts in '%s'.",
                 a$child, b$parent), call. = FALSE)
  }
  new_pose(a$matrix %*% b$matrix, a$parent, b$child)
}

#' @export
Ops.robot_pose <- function(e1, e2) {
  if (.Generic != "*") {
    stop("Only `*` (composition) is defined for poses.", call. = FALSE)
  }
  compose_two(e1, e2)
}

#' Compose poses
#'
#' `robot_compose(a, b, c)` returns `a * b * c`, the product of the
#' homogeneous matrices from left to right.
#'
#' @param ... `robot_pose` or `robot_rotation` objects.
#' @return A `robot_pose`.
#' @examples
#' A <- robot_pose(translation = c(1, 0, 0), parent = "world", child = "a")
#' B <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2),
#'                 parent = "a", child = "b")
#' robot_compose(A, B)
#' @export
robot_compose <- function(...) {
  xs <- list(...)
  if (length(xs) == 0) stop("Nothing to compose.", call. = FALSE)
  Reduce(compose_two, xs[-1], if (inherits(xs[[1]], "robot_rotation"))
    robot_pose(xs[[1]]) else xs[[1]])
}

#' Inverse of a rotation or pose
#'
#' For \eqn{T = [R, p; 0, 1]} returns \eqn{T^{-1} = [R^\top, -R^\top p;
#' 0, 1]}, computed in closed form. Frame names are swapped.
#'
#' @param x A `robot_pose`, `robot_rotation`, or `robot_quaternion`.
#' @return An object of the same class.
#' @examples
#' T <- robot_pose(robot_rotation(axis = c(1, 0, 0), angle = 0.5), c(1, 2, 3))
#' as.matrix(robot_inverse(T) * T)
#' @export
robot_inverse <- function(x) {
  if (inherits(x, "robot_rotation")) return(new_rotation(t(x$matrix)))
  if (inherits(x, "robot_quaternion")) return(quaternion_conjugate(x))
  if (!inherits(x, "robot_pose")) {
    stop("`x` must be a robot_pose, robot_rotation or robot_quaternion.",
         call. = FALSE)
  }
  d <- x$dim
  R <- pose_rotation_matrix(x)
  p <- pose_translation_vector(x)
  M <- diag(d + 1)
  M[1:d, 1:d] <- t(R)
  M[1:d, d + 1] <- -t(R) %*% p
  new_pose(M, x$child, x$parent)
}

#' Relative pose between two frames
#'
#' Given the poses of frames `a` and `b` in a common frame `w`, returns
#' \eqn{{}^{a}T_{b} = ({}^{w}T_{a})^{-1}\,{}^{w}T_{b}}.
#'
#' @param from Pose of frame `a`.
#' @param to Pose of frame `b`.
#' @return A `robot_pose`.
#' @examples
#' A <- robot_pose(translation = c(1, 0, 0))
#' B <- robot_pose(translation = c(1, 1, 0))
#' pose_translation(robot_relative_pose(A, B))
#' @export
robot_relative_pose <- function(from, to) {
  compose_two(robot_inverse(from), to)
}

as_point_matrix <- function(p, d) {
  if (is.matrix(p)) {
    P <- check_matrix(p, NULL, d, "points")
    list(P = P, vector = FALSE)
  } else {
    list(P = matrix(check_numeric_vector(p, d, "points"), 1), vector = TRUE)
  }
}

#' Transform points and directions
#'
#' `robot_transform_point(T, p)` returns \eqn{R p + t}. Directions (free
#' vectors such as velocities or axes) only rotate:
#' `robot_transform_direction(T, v)` returns \eqn{R v}. Points may be one
#' vector or a matrix with one point per row.
#'
#' @param x A `robot_pose` or `robot_rotation`.
#' @param points A vector or matrix with one point per row.
#' @return Transformed points with the same shape as the input.
#' @examples
#' T <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2), c(1, 2, 3))
#' robot_transform_point(T, rbind(c(1, 0, 0), c(0, 1, 0)))
#' robot_transform_direction(T, c(1, 0, 0))
#' @export
robot_transform_point <- function(x, points) {
  if (inherits(x, "robot_rotation")) x <- robot_pose(x)
  stopifnot(inherits(x, "robot_pose"))
  d <- x$dim
  pp <- as_point_matrix(points, d)
  out <- pp$P %*% t(pose_rotation_matrix(x))
  out <- sweep(out, 2, pose_translation_vector(x), "+")
  if (pp$vector) as.numeric(out) else out
}

#' @rdname robot_transform_point
#' @export
robot_transform_direction <- function(x, points) {
  if (inherits(x, "robot_rotation")) x <- robot_pose(x)
  stopifnot(inherits(x, "robot_pose"))
  pp <- as_point_matrix(points, x$dim)
  out <- pp$P %*% t(pose_rotation_matrix(x))
  if (pp$vector) as.numeric(out) else out
}

#' Distance between two poses
#'
#' Returns the Euclidean translation distance and the rotation angle
#' \eqn{\|\log(R_1^\top R_2)\|} in radians. The two parts have different
#' units and are reported separately.
#'
#' @param a,b `robot_pose` objects of the same dimension.
#' @return A named numeric vector `c(translation, rotation)`.
#' @examples
#' robot_pose_distance(robot_pose(), robot_pose(translation = c(1, 0, 0)))
#' @export
robot_pose_distance <- function(a, b) {
  stopifnot(inherits(a, "robot_pose"), inherits(b, "robot_pose"),
            a$dim == b$dim)
  Rrel <- crossprod(pose_rotation_matrix(a), pose_rotation_matrix(b))
  ang <- if (a$dim == 3) vnorm(so3_log(Rrel)) else abs(so2_log(Rrel))
  c(translation = vnorm(pose_translation_vector(a) -
                          pose_translation_vector(b)),
    rotation = ang)
}

#' Interpolate between two poses
#'
#' With `method = "decoupled"` (default) the translation is interpolated
#' linearly and the rotation by [quaternion_slerp()] (or linear angle
#' interpolation in 2D). With `method = "screw"` the interpolation follows
#' the constant twist \eqn{T_1 \exp(s \log(T_1^{-1} T_2))}, a screw motion.
#'
#' @param a,b Start and end poses.
#' @param s Interpolation parameter(s) in \eqn{[0, 1]}.
#' @param method `"decoupled"` or `"screw"`.
#' @return A `robot_pose` (scalar `s`) or a list of poses.
#' @examples
#' a <- robot_pose()
#' b <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2), c(1, 1, 0))
#' robot_pose_interp(a, b, 0.5)
#' @export
robot_pose_interp <- function(a, b, s, method = c("decoupled", "screw")) {
  method <- match.arg(method)
  stopifnot(inherits(a, "robot_pose"), inherits(b, "robot_pose"),
            a$dim == b$dim)
  s <- check_numeric_vector(s, name = "s")
  if (any(s < 0 | s > 1)) stop("`s` must lie in [0, 1].", call. = FALSE)
  d <- a$dim
  rel <- robot_relative_pose(robot_pose(matrix = a$matrix),
                             robot_pose(matrix = b$matrix))
  one <- function(si) {
    if (method == "screw") {
      xi <- if (d == 3) se3_log(rel$matrix) else se2_log(rel$matrix)
      E <- if (d == 3) se3_exp(si * xi) else se2_exp(si * xi)
      return(new_pose(a$matrix %*% E, a$parent, b$child))
    }
    p <- (1 - si) * pose_translation_vector(a) + si * pose_translation_vector(b)
    if (d == 3) {
      R <- quaternion_to_matrix(quaternion_slerp(
        robot_quaternion(pose_rotation_matrix(a)),
        robot_quaternion(pose_rotation_matrix(b)), si)$q)
    } else {
      th <- so2_log(pose_rotation_matrix(a)) +
        si * so2_log(pose_rotation_matrix(rel))
      R <- so2_exp(th)
    }
    M <- diag(d + 1)
    M[1:d, 1:d] <- R
    M[1:d, d + 1] <- p
    new_pose(M, a$parent, b$child)
  }
  if (length(s) == 1) one(s) else lapply(s, one)
}

#' Homogeneous transforms from elementary motions
#'
#' `transl()` returns a pure translation and `trotx()`, `troty()` and
#' `trotz()` pure rotations, all as `robot_pose` objects, so that sequences
#' of elementary transformations can be composed with `*`.
#'
#' @param x,y,z Translation components in meters. `x` may be a length-3
#'   vector.
#' @param theta Angle in radians.
#' @return A `robot_pose`.
#' @examples
#' T <- transl(1, 0, 0) * trotz(pi / 2) * transl(1, 0, 0)
#' pose_translation(T)
#' @name elementary_transforms
NULL

#' @rdname elementary_transforms
#' @export
transl <- function(x = 0, y = 0, z = 0) {
  p <- if (length(x) == 3) x else c(x, y, z)
  robot_pose(translation = check_numeric_vector(p, 3, "translation"))
}

#' @rdname elementary_transforms
#' @export
trotx <- function(theta) robot_pose(rotx(theta))

#' @rdname elementary_transforms
#' @export
troty <- function(theta) robot_pose(roty(theta))

#' @rdname elementary_transforms
#' @export
trotz <- function(theta) robot_pose(rotz(theta))
