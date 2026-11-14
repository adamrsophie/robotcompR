# Joints and links.

joint_types <- c("revolute", "continuous", "prismatic", "fixed")

#' Robot joints
#'
#' A joint connects a `parent` link to a `child` link. The pose of the
#' child link frame relative to the parent link frame is
#' \deqn{{}^{parent}T_{child}(q) = T_{origin}\, M(q)\, T_{post},}
#' where \eqn{T_{origin}} is the fixed joint frame (the URDF `<origin>`),
#' \eqn{M(q)} is a rotation by \eqn{q + offset} about `axis`
#' (revolute and continuous joints) or a translation by
#' \eqn{(q + offset)} along `axis` (prismatic joints), and
#' \eqn{T_{post}} is an optional fixed transform applied after the motion.
#' URDF joints use \eqn{T_{post} = I}; Denavit-Hartenberg models use it to
#' place the link frame at the DH frame. The axis is expressed in the joint
#' frame and normalized.
#'
#' Units are radians and meters. Revolute joints default to unbounded
#' limits; continuous joints are always unbounded and reject finite
#' limits. Velocity, acceleration and effort limits are optional
#' (`NULL` means unspecified, not zero).
#'
#' @param name Joint name.
#' @param type One of `"revolute"`, `"continuous"`, `"prismatic"`,
#'   `"fixed"`.
#' @param parent,child Names of the parent and child links.
#' @param origin Fixed `robot_pose` (SE(3)) of the joint frame in the parent
#'   link frame.
#' @param axis Joint axis in the joint frame.
#' @param limits Length-2 vector `c(lower, upper)` of position limits.
#' @param velocity,acceleration,effort Optional nonnegative limits.
#' @param offset Constant added to the joint variable before the motion.
#' @param post Optional fixed `robot_pose` applied after the motion.
#' @return A `robot_joint` object.
#' @examples
#' robot_joint("shoulder", "revolute", "base", "upper_arm",
#'             origin = transl(0, 0, 0.1), axis = c(0, 0, 1),
#'             limits = c(-pi, pi))
#' @export
robot_joint <- function(name, type = c("revolute", "continuous", "prismatic",
                                       "fixed"),
                        parent, child, origin = robot_pose(),
                        axis = c(0, 0, 1), limits = NULL, velocity = NULL,
                        acceleration = NULL, effort = NULL, offset = 0,
                        post = NULL) {
  name <- check_string(name, "name")
  type <- match.arg(type)
  parent <- check_string(parent, "parent")
  child <- check_string(child, "child")
  if (parent == child) {
    stop(sprintf("Joint '%s' connects link '%s' to itself.", name, parent),
         call. = FALSE)
  }
  origin <- as_pose3(origin, "origin")
  post <- if (is.null(post)) NULL else as_pose3(post, "post")
  axis <- if (type == "fixed") c(0, 0, 1)
          else unit_vector(check_numeric_vector(axis, 3, "axis"))
  if (type == "fixed") {
    if (!is.null(limits)) {
      stop(sprintf("Fixed joint '%s' cannot have limits.", name),
           call. = FALSE)
    }
    limits <- c(0, 0)
  } else if (type == "continuous") {
    if (!is.null(limits) && any(is.finite(limits))) {
      stop(sprintf("Continuous joint '%s' cannot have finite position limits.",
                   name), call. = FALSE)
    }
    limits <- c(-Inf, Inf)
  } else {
    limits <- limits %||% c(-Inf, Inf)
    if (!is.numeric(limits) || length(limits) != 2 || anyNA(limits)) {
      stop(sprintf("`limits` of joint '%s' must be c(lower, upper).", name),
           call. = FALSE)
    }
    if (limits[1] > limits[2]) {
      stop(sprintf("Joint '%s' has lower limit %g above upper limit %g.",
                   name, limits[1], limits[2]), call. = FALSE)
    }
  }
  chk <- function(v, nm) {
    if (is.null(v)) return(NULL)
    check_scalar(v, nm, nonnegative = TRUE)
  }
  structure(list(
    name = name, type = type, parent = parent, child = child,
    origin = origin$matrix, axis = axis, limits = as.numeric(limits),
    velocity = chk(velocity, "velocity"),
    acceleration = chk(acceleration, "acceleration"),
    effort = chk(effort, "effort"),
    offset = check_scalar(offset, "offset"),
    post = if (is.null(post)) NULL else post$matrix
  ), class = "robot_joint")
}

as_pose3 <- function(x, name) {
  if (inherits(x, "robot_pose")) {
    if (x$dim != 3) stop(sprintf("`%s` must be an SE(3) pose.", name),
                         call. = FALSE)
    return(x)
  }
  if (is.matrix(x)) return(robot_pose(matrix = x))
  stop(sprintf("`%s` must be a robot_pose.", name), call. = FALSE)
}

#' @export
print.robot_joint <- function(x, ...) {
  lim <- if (x$type %in% c("revolute", "prismatic"))
    sprintf(" limits [%g, %g]", x$limits[1], x$limits[2]) else ""
  cat(sprintf("<robot_joint> %s (%s): %s -> %s, axis (%s)%s\n", x$name,
              x$type, x$parent, x$child,
              paste(signif(x$axis, 4), collapse = ", "), lim))
  invisible(x)
}

is_movable <- function(j) j$type != "fixed"

joint_motion <- function(j, q) {
  M <- diag(4)
  if (j$type == "fixed") return(M)
  v <- q + j$offset
  if (j$type == "prismatic") {
    M[1:3, 4] <- j$axis * v
  } else {
    M[1:3, 1:3] <- so3_exp(j$axis * v)
  }
  M
}

check_inertia <- function(I, name, tol = 1e-9) {
  I <- check_matrix(I, 3, 3, name)
  if (max(abs(I - t(I))) > tol * max(1, max(abs(I)))) {
    stop(sprintf("Inertia of link '%s' is not symmetric.", name),
         call. = FALSE)
  }
  I <- (I + t(I)) / 2
  ev <- eigen(I, symmetric = TRUE, only.values = TRUE)$values
  scale <- max(1, max(abs(ev)))
  if (min(ev) < -tol * scale) {
    stop(sprintf("Inertia of link '%s' is not positive semidefinite.", name),
         call. = FALSE)
  }
  # Principal moments of a physical body satisfy the triangle inequality.
  if (ev[1] > ev[2] + ev[3] + tol * scale) {
    stop(sprintf(paste0("Inertia of link '%s' violates the triangle ",
                        "inequality of principal moments."), name),
         call. = FALSE)
  }
  I
}

#' Robot links
#'
#' A link is a rigid body with an attached frame. Inertial parameters are
#' optional: `mass` in kilograms, `com` the center of mass in the link
#' frame (meters), and `inertia` the 3 by 3 rotational inertia about the
#' center of mass, expressed in axes parallel to the link frame (kg m^2).
#' Inertia matrices must be symmetric positive semidefinite and satisfy
#' the triangle inequality of principal moments.
#'
#' @param name Link name.
#' @param mass Mass (nonnegative).
#' @param com Center of mass in the link frame.
#' @param inertia Inertia about the center of mass.
#' @return A `robot_link` object.
#' @examples
#' robot_link("upper_arm", mass = 2, com = c(0.25, 0, 0),
#'            inertia = diag(c(0.001, 0.04, 0.04)))
#' @export
robot_link <- function(name, mass = 0, com = c(0, 0, 0),
                       inertia = matrix(0, 3, 3)) {
  name <- check_string(name, "name")
  mass <- check_scalar(mass, "mass", nonnegative = TRUE)
  com <- check_numeric_vector(com, 3, "com")
  inertia <- check_inertia(inertia, name)
  structure(list(name = name, mass = mass, com = com, inertia = inertia,
                 collision = list(), visual = list()),
            class = "robot_link")
}

#' @export
print.robot_link <- function(x, ...) {
  cat(sprintf("<robot_link> %s: mass %g kg, com (%s)\n", x$name, x$mass,
              paste(signif(x$com, 4), collapse = ", ")))
  invisible(x)
}

#' Inertia tensors of simple solids
#'
#' Rotational inertia about the center of mass for uniform solids, in the
#' body axes stated: a rod of length `length` along `axis`, a solid
#' cuboid with edge lengths `size`, a solid cylinder of radius `radius` and
#' height `length` along z, and a solid sphere.
#'
#' @param mass Mass in kilograms.
#' @param length Length in meters.
#' @param axis For rods, `"x"`, `"y"` or `"z"`.
#' @param size Edge lengths c(x, y, z).
#' @param radius Radius in meters.
#' @return A 3 by 3 inertia matrix.
#' @examples
#' inertia_rod(1, 0.5)
#' inertia_box(2, c(0.1, 0.2, 0.3))
#' @name inertia_solids
NULL

#' @rdname inertia_solids
#' @export
inertia_rod <- function(mass, length, axis = c("x", "y", "z")) {
  axis <- match.arg(axis)
  v <- mass * length^2 / 12
  d <- c(v, v, v)
  d[match(axis, c("x", "y", "z"))] <- 0
  diag(d)
}

#' @rdname inertia_solids
#' @export
inertia_box <- function(mass, size) {
  s <- check_numeric_vector(size, 3, "size")
  mass / 12 * diag(c(s[2]^2 + s[3]^2, s[1]^2 + s[3]^2, s[1]^2 + s[2]^2))
}

#' @rdname inertia_solids
#' @export
inertia_cylinder <- function(mass, radius, length) {
  a <- mass * (3 * radius^2 + length^2) / 12
  diag(c(a, a, mass * radius^2 / 2))
}

#' @rdname inertia_solids
#' @export
inertia_sphere <- function(mass, radius) {
  diag(rep(2 * mass * radius^2 / 5, 3))
}
