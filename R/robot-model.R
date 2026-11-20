# Rigid-body tree robot models and forward kinematics.

#' Robot models as rigid-body trees
#'
#' Builds a validated kinematic tree from links and joints. Every link
#' except one root link has exactly one parent joint; the structure must
#' be connected and acyclic. Closed kinematic chains are not representable
#' and are rejected. The configuration vector `q` has one entry per
#' movable (non-fixed) joint, ordered by a breadth-first traversal from
#' the root that preserves the order joints were supplied in; see
#' [robot_joint_names()]. The number of joints therefore can exceed the
#' number of degrees of freedom when fixed joints are present.
#'
#' @param links List of [robot_link()] objects. Links named by joints but
#'   missing here are created as massless links.
#' @param joints List of [robot_joint()] objects.
#' @param base Pose of the root link in the world frame.
#' @param end_effector Name of the default end-effector frame. Defaults to
#'   the last leaf link in traversal order.
#' @param name Model name.
#' @return A `robot_model` object.
#' @examples
#' links <- list(robot_link("base"), robot_link("link1"), robot_link("link2"))
#' joints <- list(
#'   robot_joint("j1", "revolute", "base", "link1"),
#'   robot_joint("j2", "revolute", "link1", "link2", origin = transl(1, 0, 0))
#' )
#' robot <- robot_model(links, joints, name = "planar")
#' robot
#' @export
robot_model <- function(links = list(), joints, base = robot_pose(),
                        end_effector = NULL, name = "robot") {
  if (inherits(links, "robot_link")) links <- list(links)
  if (inherits(joints, "robot_joint")) joints <- list(joints)
  if (!all(vapply(links, inherits, logical(1), "robot_link"))) {
    stop("`links` must be a list of robot_link objects.", call. = FALSE)
  }
  if (!length(joints) ||
      !all(vapply(joints, inherits, logical(1), "robot_joint"))) {
    stop("`joints` must be a non-empty list of robot_joint objects.",
         call. = FALSE)
  }
  link_names <- vapply(links, `[[`, "", "name")
  if (anyDuplicated(link_names)) {
    stop(sprintf("Duplicate link names: %s.",
                 paste(unique(link_names[duplicated(link_names)]),
                       collapse = ", ")), call. = FALSE)
  }
  joint_names <- vapply(joints, `[[`, "", "name")
  if (anyDuplicated(joint_names)) {
    stop(sprintf("Duplicate joint names: %s.",
                 paste(unique(joint_names[duplicated(joint_names)]),
                       collapse = ", ")), call. = FALSE)
  }
  names(links) <- link_names
  referenced <- unique(c(vapply(joints, `[[`, "", "parent"),
                         vapply(joints, `[[`, "", "child")))
  for (nm in setdiff(referenced, link_names)) links[[nm]] <- robot_link(nm)
  names(joints) <- joint_names
  compile_model(list(name = check_string(name, "name"), links = links,
                     joints = joints, base = as_pose3(base, "base")$matrix,
                     end_effector = end_effector))
}

compile_model <- function(m) {
  links <- m$links
  joints <- m$joints
  children <- vapply(joints, `[[`, "", "child")
  parents <- vapply(joints, `[[`, "", "parent")
  if (anyDuplicated(children)) {
    bad <- unique(children[duplicated(children)])
    stop(sprintf(paste0("Link(s) %s have more than one parent joint; closed ",
                        "chains are not supported."),
                 paste(bad, collapse = ", ")), call. = FALSE)
  }
  roots <- setdiff(names(links), children)
  if (length(roots) != 1) {
    if (length(roots) == 0) {
      stop("The model has no root link (the joints form a cycle).",
           call. = FALSE)
    }
    stop(sprintf("The model is disconnected: candidate roots %s.",
                 paste(roots, collapse = ", ")), call. = FALSE)
  }
  # Breadth-first traversal from the root, preserving joint order.
  order_j <- integer(0)
  frontier <- roots
  while (length(frontier)) {
    nxt <- character(0)
    for (lk in frontier) {
      idx <- which(parents == lk)
      order_j <- c(order_j, idx)
      nxt <- c(nxt, children[idx])
    }
    frontier <- nxt
  }
  if (length(order_j) != length(joints)) {
    stop("The joints contain a cycle that is not reachable from the root.",
         call. = FALSE)
  }
  joints <- joints[order_j]
  link_order <- unname(c(roots, vapply(joints, `[[`, "", "child")))
  links <- links[link_order]
  nl <- length(links)
  parent_index <- c(0L, match(vapply(joints, `[[`, "", "parent"), link_order))
  movable <- vapply(joints, is_movable, logical(1))
  q_index <- rep(NA_integer_, length(joints))
  q_index[movable] <- seq_len(sum(movable))
  # Movable-joint ancestors of each link, used by Jacobians and dynamics.
  ancestors <- vector("list", nl)
  ancestors[[1]] <- integer(0)
  for (k in seq_len(nl)[-1]) {
    ji <- k - 1L
    ancestors[[k]] <- c(ancestors[[parent_index[k]]],
                        if (movable[ji]) ji else integer(0))
  }
  leaves <- setdiff(link_order, vapply(joints, `[[`, "", "parent"))
  ee <- m$end_effector %||% utils::tail(leaves, 1)
  if (!ee %in% link_order) {
    stop(sprintf("End effector '%s' is not a link of the model.", ee),
         call. = FALSE)
  }
  structure(list(
    name = m$name, links = links, joints = joints, base = m$base,
    end_effector = ee, link_names = link_order,
    joint_names = names(joints), parent_index = parent_index,
    movable = movable, q_index = q_index, dof = sum(movable),
    ancestors = ancestors,
    # Configuration-independent dynamics quantities, rebuilt on every
    # compile (all model modifiers recompile), never carried over.
    spatial_inertia = lapply(links, spatial_inertia),
    motion_subspace = c(list(NULL), lapply(joints, motion_subspace))
  ), class = "robot_model")
}

recompile <- function(robot, ...) {
  m <- unclass(robot)
  args <- list(...)
  for (nm in names(args)) m[[nm]] <- args[[nm]]
  out <- compile_model(m[c("name", "links", "joints", "base", "end_effector")])
  # Keep auxiliary fields such as DH parameters and attached geometry.
  for (nm in setdiff(names(m), names(out))) out[[nm]] <- m[[nm]]
  out
}

#' @export
print.robot_model <- function(x, ...) {
  cat(sprintf("<robot_model> %s: %d links, %d joints, %d degrees of freedom\n",
              x$name, length(x$links), length(x$joints), x$dof))
  cat(sprintf("end effector: %s\n", x$end_effector))
  tab <- robot_joints(x)
  print(tab, row.names = FALSE)
  invisible(x)
}

#' @export
summary.robot_model <- function(object, ...) {
  print(object)
  m <- vapply(object$links, `[[`, 0, "mass")
  cat(sprintf("total mass: %g kg\n", sum(m)))
  invisible(object)
}

#' Model introspection
#'
#' `robot_dof()` returns the number of independent degrees of freedom (the
#' number of movable joints). `robot_joints()` returns a data frame of all
#' joints in traversal order; `q_index` gives the position of each movable
#' joint in configuration vectors. `robot_joint_names()` returns the names
#' of movable joints in configuration order. `robot_links()` returns a
#' data frame of links with inertial summaries. `robot_frames()` returns
#' all frame names (one per link). `robot_limits()` returns a matrix of
#' position limits with one row per degree of freedom.
#'
#' @param robot A `robot_model`.
#' @return An integer, data frame, character vector or matrix.
#' @examples
#' robot <- robot_example("planar2")
#' robot_dof(robot)
#' robot_joints(robot)
#' robot_limits(robot)
#' @name robot_introspection
NULL

#' @rdname robot_introspection
#' @export
robot_dof <- function(robot) {
  check_robot(robot)
  robot$dof
}

#' @rdname robot_introspection
#' @export
robot_joints <- function(robot) {
  check_robot(robot)
  j <- robot$joints
  data.frame(
    name = robot$joint_names,
    type = vapply(j, `[[`, "", "type"),
    parent = vapply(j, `[[`, "", "parent"),
    child = vapply(j, `[[`, "", "child"),
    q_index = robot$q_index,
    lower = vapply(j, function(z) z$limits[1], 0),
    upper = vapply(j, function(z) z$limits[2], 0),
    stringsAsFactors = FALSE
  )
}

#' @rdname robot_introspection
#' @export
robot_joint_names <- function(robot) {
  check_robot(robot)
  robot$joint_names[robot$movable]
}

#' @rdname robot_introspection
#' @export
robot_links <- function(robot) {
  check_robot(robot)
  l <- robot$links
  data.frame(
    name = robot$link_names,
    parent = c(NA_character_, robot$link_names[robot$parent_index[-1]]),
    mass = vapply(l, `[[`, 0, "mass"),
    n_collision = vapply(l, function(z) length(z$collision), 0L),
    row.names = NULL,
    stringsAsFactors = FALSE
  )
}

#' @rdname robot_introspection
#' @export
robot_frames <- function(robot) {
  check_robot(robot)
  robot$link_names
}

#' @rdname robot_introspection
#' @export
robot_limits <- function(robot) {
  check_robot(robot)
  L <- t(vapply(robot$joints[robot$movable], `[[`, c(0, 0), "limits"))
  if (robot$dof == 1) L <- matrix(L, 1)
  dimnames(L) <- list(robot_joint_names(robot), c("lower", "upper"))
  L
}

check_robot <- function(robot) {
  if (!inherits(robot, "robot_model")) {
    stop("`robot` must be a robot_model.", call. = FALSE)
  }
  invisible(robot)
}

# Joint arrays (configurations, velocities, accelerations, efforts) enter
# every public function through these two checks. Unnamed values are
# positional, in robot_joint_names() order. Named values must carry exactly
# the joint names, each once, in any order, and are permuted into joint
# order; partial, duplicated or unknown names are an error. Names are
# dropped only after the permutation.
joint_order <- function(robot, nms, name) {
  if (is.null(nms) || all(!nzchar(nms))) return(NULL)
  jn <- robot_joint_names(robot)
  if (length(nms) != length(jn) || anyDuplicated(nms) || !setequal(nms, jn)) {
    stop(sprintf(paste0("Names of `%s` must be the joint names (%s), each ",
                        "once, in any order; or leave `%s` unnamed."),
                 name, paste(jn, collapse = ", "), name), call. = FALSE)
  }
  match(jn, nms)
}

check_q <- function(robot, q, name = "q") {
  nms <- if (is.matrix(q) && nrow(q) == 1L) colnames(q) else names(q)
  q <- check_numeric_vector(q, name = name)
  if (length(q) != robot$dof) {
    stop(sprintf("`%s` must have length %d (the number of degrees of freedom).",
                 name, robot$dof), call. = FALSE)
  }
  idx <- joint_order(robot, nms, name)
  if (!is.null(idx)) q <- q[idx]
  unname(q)
}

check_q_matrix <- function(robot, Q, name = "Q") {
  if (!is.matrix(Q)) {
    nms <- names(Q)
    Q <- matrix(Q, 1, dimnames = list(NULL, nms))
  }
  Q <- check_matrix(Q, NULL, robot$dof, name)
  idx <- joint_order(robot, colnames(Q), name)
  if (!is.null(idx)) Q <- Q[, idx, drop = FALSE]
  unname(Q)
}

frame_index <- function(robot, frame) {
  frame <- frame %||% robot$end_effector
  k <- match(check_string(frame, "frame"), robot$link_names)
  if (is.na(k)) {
    stop(sprintf("Unknown frame '%s'. Available frames: %s.", frame,
                 paste(robot$link_names, collapse = ", ")), call. = FALSE)
  }
  k
}

#' Modify a robot model
#'
#' Models are immutable; these functions return modified copies.
#' `robot_add_frame()` attaches a new massless frame (for example a tool
#' center point) to an existing link through a fixed joint.
#' `robot_set_base()` changes the pose of the root link in the world.
#' `robot_set_end_effector()` changes the default end-effector frame.
#' `robot_set_inertia()` replaces the inertial parameters of a link.
#' `robot_set_limits()` replaces position limits of movable joints.
#'
#' @param robot A `robot_model`.
#' @param name New frame name.
#' @param parent Existing link to attach to.
#' @param origin Pose of the new frame in the parent link frame.
#' @param set_end_effector Make the new frame the default end effector.
#' @param base New base pose.
#' @param frame Frame name.
#' @param link Link name.
#' @param mass,com,inertia Inertial parameters as in [robot_link()].
#' @param limits Matrix or list of `c(lower, upper)` with one entry per
#'   degree of freedom, or a named list keyed by joint name.
#' @return A modified `robot_model`.
#' @examples
#' robot <- robot_example("planar2")
#' robot <- robot_add_frame(robot, "tcp", "link2", transl(0.1, 0, 0))
#' pose_translation(robot_fk(robot, c(0, 0)))
#' @name robot_modify
NULL

#' @rdname robot_modify
#' @export
robot_add_frame <- function(robot, name, parent, origin = robot_pose(),
                            set_end_effector = TRUE) {
  check_robot(robot)
  name <- check_string(name, "name")
  if (name %in% robot$link_names) {
    stop(sprintf("Frame '%s' already exists.", name), call. = FALSE)
  }
  frame_index(robot, parent)
  jn <- paste0(name, "_joint")
  while (jn %in% robot$joint_names) jn <- paste0(jn, "_")
  joints <- c(robot$joints, list(robot_joint(jn, "fixed", parent, name,
                                             origin = origin)))
  names(joints)[length(joints)] <- jn
  links <- c(robot$links, stats::setNames(list(robot_link(name)), name))
  recompile(robot, links = links, joints = joints,
            end_effector = if (set_end_effector) name else robot$end_effector)
}

#' @rdname robot_modify
#' @export
robot_set_base <- function(robot, base) {
  check_robot(robot)
  recompile(robot, base = as_pose3(base, "base")$matrix)
}

#' @rdname robot_modify
#' @export
robot_set_end_effector <- function(robot, frame) {
  check_robot(robot)
  k <- frame_index(robot, frame)
  recompile(robot, end_effector = robot$link_names[k])
}

#' @rdname robot_modify
#' @export
robot_set_inertia <- function(robot, link, mass, com = c(0, 0, 0),
                              inertia = matrix(0, 3, 3)) {
  check_robot(robot)
  k <- frame_index(robot, link)
  old <- robot$links[[k]]
  new <- robot_link(old$name, mass, com, inertia)
  new$collision <- old$collision
  new$visual <- old$visual
  links <- robot$links
  links[[k]] <- new
  recompile(robot, links = links)
}

#' @rdname robot_modify
#' @export
robot_set_limits <- function(robot, limits) {
  check_robot(robot)
  mv <- which(robot$movable)
  if (is.matrix(limits)) limits <- lapply(seq_len(nrow(limits)),
                                          function(i) limits[i, ])
  if (!is.null(names(limits)) && all(nzchar(names(limits)))) {
    idx <- match(names(limits), robot$joint_names)
    if (anyNA(idx)) stop("Unknown joint names in `limits`.", call. = FALSE)
  } else {
    if (length(limits) != robot$dof) {
      stop("`limits` needs one entry per degree of freedom.", call. = FALSE)
    }
    idx <- mv
  }
  joints <- robot$joints
  for (i in seq_along(idx)) {
    j <- joints[[idx[i]]]
    # The constructor validates the new limits (continuous joints reject
    # finite ones); every other field, such as damping, is kept.
    nj <- robot_joint(
      j$name, j$type, j$parent, j$child, robot_pose(matrix = j$origin),
      j$axis, limits[[i]], j$velocity, j$acceleration, j$effort, j$offset,
      if (is.null(j$post)) NULL else robot_pose(matrix = j$post))
    j$limits <- nj$limits
    joints[[idx[i]]] <- j
  }
  names(joints) <- robot$joint_names
  recompile(robot, joints = joints)
}

# Core forward kinematics: world poses of every link frame and every joint
# frame (the frame after the motion and before `post`, in which the joint
# axis is constant).
fk_internal <- function(robot, q) {
  nl <- length(robot$link_names)
  Tl <- vector("list", nl)
  Tj <- vector("list", nl)
  Tl[[1]] <- robot$base
  for (k in seq_len(nl)[-1]) {
    ji <- k - 1L
    j <- robot$joints[[ji]]
    qi <- if (robot$movable[ji]) q[robot$q_index[ji]] else 0
    A <- Tl[[robot$parent_index[k]]] %*% j$origin %*% joint_motion(j, qi)
    Tj[[k]] <- A
    Tl[[k]] <- if (is.null(j$post)) A else A %*% j$post
  }
  list(link = Tl, joint = Tj)
}

#' Forward kinematics
#'
#' Computes the pose of a frame in the world frame,
#' \eqn{{}^{world}T_{frame}(q) = T_{base} \prod_k {}^{k-1}T_{k}(q_k)},
#' along the path from the root to `frame`. With `frame = "all"` the poses
#' of every link frame are returned as a named list. A matrix `q` with one
#' configuration per row returns a list with one result per row.
#' Configurations outside the joint limits are evaluated without error;
#' use [robot_within_limits()] to check them.
#'
#' @param robot A `robot_model`.
#' @param q Joint configuration (length [robot_dof()]) or a matrix with one
#'   configuration per row.
#' @param frame Frame name, `"all"`, or `NULL` for the end effector.
#' @return A `robot_pose`, or a list of poses.
#' @examples
#' robot <- robot_example("planar2")
#' robot_fk(robot, c(pi / 2, pi / 2))
#' @export
robot_fk <- function(robot, q, frame = NULL) {
  check_robot(robot)
  if (is.matrix(q)) {
    return(lapply(seq_len(nrow(q)), function(i) robot_fk(robot, q[i, ], frame)))
  }
  q <- check_q(robot, q)
  fk <- fk_internal(robot, q)
  if (identical(frame, "all")) {
    out <- lapply(seq_along(fk$link), function(k)
      new_pose(fk$link[[k]], "world", robot$link_names[k]))
    names(out) <- robot$link_names
    return(out)
  }
  k <- frame_index(robot, frame)
  new_pose(fk$link[[k]], "world", robot$link_names[k])
}

#' Positions of a frame for many configurations
#'
#' Evaluates forward kinematics for each row of `Q` and returns the frame
#' origin positions as a matrix with one row per configuration.
#'
#' @param robot A `robot_model`.
#' @param Q Matrix with one configuration per row.
#' @param frame Frame name (`NULL` for the end effector).
#' @return A matrix with columns x, y, z.
#' @examples
#' robot <- robot_example("planar2")
#' robot_fk_positions(robot, rbind(c(0, 0), c(pi / 2, 0)))
#' @export
robot_fk_positions <- function(robot, Q, frame = NULL) {
  check_robot(robot)
  Q <- check_q_matrix(robot, Q, "Q")
  k <- frame_index(robot, frame)
  out <- t(vapply(seq_len(nrow(Q)), function(i)
    fk_internal(robot, Q[i, ])$link[[k]][1:3, 4], numeric(3)))
  colnames(out) <- c("x", "y", "z")
  out
}

#' Joint limits checks
#'
#' `robot_within_limits()` reports whether each configuration satisfies the
#' position limits within `tol`. `robot_clamp()` projects a configuration
#' onto the limit box.
#'
#' @param robot A `robot_model`.
#' @param q Configuration vector.
#' @param tol Tolerance.
#' @return Logical scalar (`robot_within_limits`) or numeric vector.
#' @examples
#' robot <- robot_example("planar2")
#' robot_within_limits(robot, c(0, 4))
#' robot_clamp(robot, c(0, 4))
#' @name joint_limits
NULL

#' @rdname joint_limits
#' @export
robot_within_limits <- function(robot, q, tol = 1e-10) {
  check_robot(robot)
  q <- check_q(robot, q)
  L <- robot_limits(robot)
  all(q >= L[, 1] - tol & q <= L[, 2] + tol)
}

#' @rdname joint_limits
#' @export
robot_clamp <- function(robot, q) {
  check_robot(robot)
  q <- check_q(robot, q)
  L <- robot_limits(robot)
  pmin(pmax(q, L[, 1]), L[, 2])
}

#' Random and home configurations
#'
#' `robot_random_configuration()` samples uniformly within the position
#' limits; joints unbounded on both sides are sampled in \eqn{[-\pi, \pi)}
#' (revolute and continuous) or \eqn{[-1, 1]} meters (prismatic), and a
#' joint bounded on one side only is sampled over a span of \eqn{2\pi} rad or
#' 2 m from its finite limit. The caller's random
#' number state is left unchanged when `seed` is supplied.
#' `robot_home()` returns zeros projected onto the limits.
#'
#' @param robot A `robot_model`.
#' @param n Number of configurations.
#' @param seed Optional integer seed.
#' @return A vector (`n = 1`) or a matrix with one row per configuration.
#' @examples
#' robot <- robot_example("planar2")
#' robot_random_configuration(robot, 3, seed = 1)
#' @name robot_configurations
NULL

#' @rdname robot_configurations
#' @export
robot_random_configuration <- function(robot, n = 1, seed = NULL) {
  check_robot(robot)
  n <- check_count(n, "n")
  B <- sampling_bounds(robot)
  with_seed_internal(seed, {
    Q <- matrix(stats::runif(n * robot$dof), n, robot$dof, byrow = TRUE)
    Q <- sweep(sweep(Q, 2, B[, 2] - B[, 1], "*"), 2, B[, 1], "+")
    colnames(Q) <- robot_joint_names(robot)
    if (n == 1) Q[1, ] else Q
  })
}

sampling_bounds <- function(robot) {
  L <- robot_limits(robot)
  types <- vapply(robot$joints[robot$movable], `[[`, "", "type")
  for (i in seq_len(nrow(L))) {
    # Unbounded sides get a default span (2 m or 2 pi rad), anchored at
    # the finite limit when only one side is bounded.
    w <- if (types[i] == "prismatic") 2 else 2 * pi
    lo <- is.finite(L[i, 1])
    hi <- is.finite(L[i, 2])
    if (!lo && !hi) {
      L[i, ] <- c(-w / 2, w / 2)
    } else if (!lo) {
      L[i, 1] <- L[i, 2] - w
    } else if (!hi) {
      L[i, 2] <- L[i, 1] + w
    }
  }
  L
}

#' @rdname robot_configurations
#' @export
robot_home <- function(robot) {
  check_robot(robot)
  q <- robot_clamp(robot, rep(0, robot$dof))
  names(q) <- robot_joint_names(robot)
  q
}
