# Distance and collision queries.

# Closest point to the origin on the convex hull of the rows of W, by
# enumerating faces of the simplex (at most 4 points). Returns barycentric
# weights over the rows.
simplex_closest <- function(W) {
  k <- nrow(W)
  best <- NULL
  best_d <- Inf
  for (mask in seq_len(2^k - 1)) {
    idx <- which(bitwAnd(mask, 2^(seq_len(k) - 1)) > 0)
    P <- W[idx, , drop = FALSE]
    if (length(idx) == 1) {
      lam <- 1
    } else {
      D <- t(P[-1, , drop = FALSE]) - P[1, ]
      G <- crossprod(D)
      if (rcond(G) < 1e-14) next
      mu <- -solve(G, crossprod(D, P[1, ]))
      lam <- c(1 - sum(mu), mu)
      if (any(lam < -1e-12)) next
    }
    x <- as.numeric(crossprod(P, lam))
    d <- sum(x * x)
    if (d < best_d * (1 - 1e-12)) {
      best_d <- d
      best <- list(idx = idx, lambda = pmax(lam, 0) / sum(pmax(lam, 0)), x = x)
    }
  }
  best
}

# GJK distance between the convex cores of two geometries. Tolerances are
# relative to the size of the geometries so that tiny and huge objects are
# treated alike.
gjk_distance <- function(gA, TA, gB, TB, max_iter = 64, rel_tol = 1e-10) {
  cA <- TA[1:3, 4]
  cB <- TB[1:3, 4]
  scale <- max(geometry_bound(gA) - geometry_radius(gA),
               geometry_bound(gB) - geometry_radius(gB),
               .Machine$double.xmin)
  tol <- rel_tol * scale
  d <- cA - cB
  if (vnorm(d) < tol) d <- c(1, 0, 0)
  sa <- geometry_support(gA, TA, -d)
  sb <- geometry_support(gB, TB, d)
  A <- matrix(sa, 1)
  B <- matrix(sb, 1)
  v <- sa - sb
  lam <- 1
  for (it in seq_len(max_iter)) {
    vn2 <- sum(v * v)
    if (vn2 < tol^2) {
      return(list(distance = 0, pa = as.numeric(crossprod(A, lam)),
                  pb = as.numeric(crossprod(B, lam)), intersect = TRUE))
    }
    sa <- geometry_support(gA, TA, -v)
    sb <- geometry_support(gB, TB, v)
    w <- sa - sb
    # Converged when the new support point does not improve the bound
    # (relative test of van den Bergen, 2003).
    if (vn2 - sum(v * w) <= 1e-12 * vn2) break
    W <- rbind(A - B, w)
    if (any(rowSums(abs(sweep(A - B, 2, w))) < 1e-14 * scale)) break
    A2 <- rbind(A, sa)
    B2 <- rbind(B, sb)
    cl <- simplex_closest(W)
    A <- A2[cl$idx, , drop = FALSE]
    B <- B2[cl$idx, , drop = FALSE]
    lam <- cl$lambda
    vnew <- cl$x
    if (sum(vnew * vnew) >= vn2 * (1 - 1e-12) && length(cl$idx) < 4) {
      v <- vnew
      break
    }
    v <- vnew
    if (length(cl$idx) == 4) {
      return(list(distance = 0, pa = as.numeric(crossprod(A, lam)),
                  pb = as.numeric(crossprod(B, lam)), intersect = TRUE))
    }
  }
  pa <- as.numeric(crossprod(A, lam))
  pb <- as.numeric(crossprod(B, lam))
  dist <- vnorm(v)
  list(distance = dist, pa = pa, pb = pb, intersect = dist < tol)
}

geometry_bound <- function(g) {
  r <- geometry_radius(g)
  r + switch(g$type,
    sphere = 0,
    capsule = g$params$length / 2,
    box = vnorm(g$params$size) / 2,
    cylinder = sqrt(g$params$radius^2 + (g$params$length / 2)^2),
    convex = sqrt(max(rowSums(g$params$vertices^2))),
    0)
}

# Core segment of a sphere (degenerate) or capsule in world coordinates.
core_segment <- function(g, T) {
  h <- if (g$type == "capsule") g$params$length / 2 else 0
  z <- T[1:3, 3] * h
  list(a = T[1:3, 4] - z, b = T[1:3, 4] + z)
}

# Closest points between segments p1-q1 and p2-q2 (Ericson, "Real-Time
# Collision Detection", 2005, section 5.1.9).
segment_closest <- function(p1, q1, p2, q2) {
  d1 <- q1 - p1
  d2 <- q2 - p2
  r <- p1 - p2
  a <- sum(d1 * d1)
  e <- sum(d2 * d2)
  f <- sum(d2 * r)
  eps <- 1e-15
  if (a <= eps && e <= eps) {
    s <- 0; t <- 0
  } else if (a <= eps) {
    s <- 0
    t <- min(1, max(0, f / e))
  } else {
    c_ <- sum(d1 * r)
    if (e <= eps) {
      t <- 0
      s <- min(1, max(0, -c_ / a))
    } else {
      b <- sum(d1 * d2)
      den <- a * e - b * b
      s <- if (den > eps * a * e) min(1, max(0, (b * f - c_ * e) / den)) else 0
      t <- (b * s + f) / e
      if (t < 0) {
        t <- 0
        s <- min(1, max(0, -c_ / a))
      } else if (t > 1) {
        t <- 1
        s <- min(1, max(0, (b - c_) / a))
      }
    }
  }
  pa <- p1 + d1 * s
  pb <- p2 + d2 * t
  list(distance = vnorm(pa - pb), pa = pa, pb = pb)
}

core_distance <- function(gA, TA, gB, TB) {
  swept <- c("sphere", "capsule")
  if (gA$type %in% swept && gB$type %in% swept) {
    sa <- core_segment(gA, TA)
    sb <- core_segment(gB, TB)
    r <- segment_closest(sa$a, sa$b, sb$a, sb$b)
    r$intersect <- r$distance < 1e-12
    return(r)
  }
  gjk_distance(gA, TA, gB, TB)
}

geometry_pair <- function(gA, TA, gB, TB) {
  core <- core_distance(gA, TA, gB, TB)
  rA <- geometry_radius(gA)
  rB <- geometry_radius(gB)
  if (core$intersect) {
    return(list(distance = -(rA + rB), collision = TRUE, depth_known = FALSE,
                pa = core$pa, pb = core$pb))
  }
  n <- (core$pb - core$pa) / core$distance
  dist <- core$distance - rA - rB
  list(distance = dist, collision = dist <= 0, depth_known = TRUE,
       pa = core$pa + rA * n, pb = core$pb - rB * n)
}

#' Distance between two geometric primitives
#'
#' Computes the separation distance and closest points between two convex
#' primitives placed in a common frame. Each primitive is a convex core
#' plus a radius: GJK (Gilbert, Johnson and Keerthi, 1988) computes the
#' distance between the cores, and the radii are subtracted. The result
#' is exact (up to the GJK tolerance) for separated shapes. When shapes
#' with a radius overlap but their cores do not, the negative distance is
#' the exact penetration depth. When the cores themselves overlap the
#' penetration depth is not computed: `distance` is reported as
#' \eqn{-(r_A + r_B)} with `depth_known = FALSE`, and only `collision`
#' should be relied on. Meshes are not supported.
#'
#' @param a,b `robot_geometry` objects.
#' @param pose_a,pose_b Poses of the frames the geometries are attached to.
#' @return A list with `distance`, `collision`, `depth_known` and the
#'   closest points `point_a` and `point_b`.
#' @examples
#' geom_distance(geom_sphere(0.5), geom_box(c(1, 1, 1)),
#'               transl(2, 0, 0), robot_pose())$distance
#' @export
geom_distance <- function(a, b, pose_a = robot_pose(), pose_b = robot_pose()) {
  stopifnot(inherits(a, "robot_geometry"), inherits(b, "robot_geometry"))
  if (!collidable(a) || !collidable(b)) {
    stop("Mesh geometry is not supported by distance queries.", call. = FALSE)
  }
  TA <- as_pose3(pose_a, "pose_a")$matrix %*% a$origin
  TB <- as_pose3(pose_b, "pose_b")$matrix %*% b$origin
  r <- geometry_pair(a, TA, b, TB)
  list(distance = r$distance, collision = r$collision,
       depth_known = r$depth_known, point_a = r$pa, point_b = r$pb)
}

#' Attach collision geometry to a link
#'
#' Returns a copy of `robot` with `geometry` added to the collision
#' geometry of `link`. The geometry origin is relative to the link frame.
#'
#' @param robot A `robot_model`.
#' @param link Link name.
#' @param geometry A `robot_geometry`.
#' @return A `robot_model`.
#' @examples
#' robot <- robot_example("planar2")
#' robot <- robot_attach_geometry(robot, "link1",
#'   geom_capsule(0.05, 1, origin = transl(-0.5, 0, 0) * troty(pi / 2)))
#' robot_links(robot)
#' @export
robot_attach_geometry <- function(robot, link, geometry) {
  check_robot(robot)
  stopifnot(inherits(geometry, "robot_geometry"))
  k <- frame_index(robot, link)
  robot$links[[k]]$collision[[length(robot$links[[k]]$collision) + 1]] <- geometry
  robot
}

# A capsule whose core segment runs from p0 to p1 (in the frame it is
# attached to).
capsule_between <- function(radius, p0, p1) {
  u <- p1 - p0
  L <- vnorm(u)
  z <- c(0, 0, 1)
  axis <- c(-u[2], u[1], 0)  # z cross u
  ang <- atan2(vnorm(axis), u[3])
  R <- if (vnorm(axis) < 1e-12 * L) {
    if (u[3] >= 0) diag(3) else rotx(pi)
  } else {
    so3_exp(axis / vnorm(axis) * ang)
  }
  geom_capsule(radius, L, origin = robot_pose(R, (p0 + p1) / 2))
}

#' Capsule collision geometry for DH arms
#'
#' Adds a capsule along every DH link, with its core segment joining the
#' origins of consecutive DH frames, so offsets along both \eqn{a} and
#' \eqn{d} are covered (for example the PUMA 560 forearm). For the
#' standard convention the segment is fixed in the child link frame and is
#' attached there; for the modified convention it is fixed in the parent
#' frame and is attached to the parent link (the base for the first
#' joint). Segments of zero length are skipped. Prismatic joints change the
#' segment length with the joint value, so they get no capsule and a
#' warning names them: attach geometry to those links yourself.
#'
#' @param robot A `robot_model` built by [robot_dh()] or [robot_example()].
#' @param radius Capsule radius.
#' @return A `robot_model`.
#' @examples
#' robot <- robot_capsule_links(robot_example("planar2"), 0.05)
#' @export
robot_capsule_links <- function(robot, radius) {
  check_robot(robot)
  if (is.null(robot$dh)) stop("`robot` must be a DH model.", call. = FALSE)
  radius <- check_scalar(radius, "radius", positive = TRUE)
  modified <- identical(robot$dh_convention, "modified")
  skipped <- character(0)
  for (i in seq_len(nrow(robot$dh))) {
    a <- robot$dh$a[i]
    al <- robot$dh$alpha[i]
    d <- robot$dh$d[i]
    if (robot$dh$type[i] == "prismatic") {
      skipped <- c(skipped, paste0("link", i))
      next
    }
    if (modified) {
      # Child origin in the parent frame: Rx(alpha) (a, 0, 0) + Rx(alpha) (0, 0, d).
      p1 <- c(a, -sin(al) * d, cos(al) * d)
      link <- if (i == 1) "base" else paste0("link", i - 1)
    } else {
      # Parent origin in the child frame: Rx(-alpha) (-a, 0, -d).
      p1 <- c(-a, -sin(al) * d, -cos(al) * d)
      link <- paste0("link", i)
    }
    if (vnorm(p1) < 1e-12) next
    robot <- robot_attach_geometry(robot, link,
                                   capsule_between(radius, c(0, 0, 0), p1))
  }
  if (length(skipped)) {
    warning(sprintf("No capsule for prismatic link(s) %s; attach geometry manually.",
                    paste(skipped, collapse = ", ")), call. = FALSE)
  }
  robot
}

#' Static environments
#'
#' An environment is a named list of world-frame obstacles. Geometry
#' origins are interpreted in the world frame.
#'
#' @param ... `robot_geometry` objects, optionally named.
#' @return A `robot_environment`.
#' @examples
#' env <- robot_environment(wall = geom_box(c(0.2, 2, 1),
#'                                          origin = transl(1.5, 0, 0)))
#' env
#' @export
robot_environment <- function(...) {
  obs <- list(...)
  if (length(obs) == 1 && is.list(obs[[1]]) &&
      !inherits(obs[[1]], "robot_geometry")) {
    obs <- obs[[1]]
  }
  if (!all(vapply(obs, inherits, logical(1), "robot_geometry"))) {
    stop("Obstacles must be robot_geometry objects.", call. = FALSE)
  }
  nm <- names(obs)
  if (is.null(nm)) nm <- rep("", length(obs))
  nm[!nzchar(nm)] <- paste0("obstacle", which(!nzchar(nm)))
  names(obs) <- nm
  structure(list(obstacles = obs), class = "robot_environment")
}

#' @export
print.robot_environment <- function(x, ...) {
  cat(sprintf("<robot_environment> %d obstacle%s\n", length(x$obstacles),
              if (length(x$obstacles) == 1) "" else "s"))
  for (nm in names(x$obstacles)) {
    cat("  ", nm, ": ", sep = "")
    print(x$obstacles[[nm]])
  }
  invisible(x)
}

robot_collision_objects <- function(robot, fk) {
  objs <- list()
  for (k in seq_along(robot$links)) {
    for (g in robot$links[[k]]$collision) {
      if (!collidable(g)) next
      objs[[length(objs) + 1]] <- list(link = k, geometry = g,
                                       T = fk$link[[k]] %*% g$origin)
    }
  }
  objs
}

default_self_pairs <- function(robot, objs) {
  pairs <- list()
  if (length(objs) < 2) return(pairs)
  adjacent <- function(a, b) {
    robot$parent_index[a] == b || robot$parent_index[b] == a
  }
  for (i in seq_len(length(objs) - 1)) {
    for (j in (i + 1):length(objs)) {
      la <- objs[[i]]$link
      lb <- objs[[j]]$link
      if (la == lb || adjacent(la, lb)) next
      pairs[[length(pairs) + 1]] <- c(i, j)
    }
  }
  pairs
}

#' Collision checking at a configuration
#'
#' Checks the robot's collision geometry against the obstacles of an
#' environment and, with `self = TRUE`, against itself. Self-collision
#' skips geometry pairs on the same link and on links connected by a
#' joint; further link pairs can be excluded with `ignore_pairs`. A cheap
#' bounding-sphere test prunes pairs that are farther apart than `margin`.
#' This checks one configuration only: a collision-free start and goal do
#' not imply that the motion between them is collision-free; use
#' [robot_check_path()] for that.
#'
#' @param robot A `robot_model` with collision geometry.
#' @param q Configuration.
#' @param environment Optional `robot_environment`.
#' @param self Check self-collision.
#' @param margin Safety margin: pairs closer than `margin` count as
#'   colliding.
#' @param ignore_pairs Optional list of `c(link_a, link_b)` name pairs to
#'   skip in self-collision checking.
#' @param distances Compute exact distances for every pair (slower).
#' @return A `robot_collision_result` with `in_collision`,
#'   `min_distance` (over evaluated pairs, `NA` if none) and a data frame
#'   `pairs` of evaluated pairs.
#' @examples
#' robot <- robot_capsule_links(robot_example("planar2"), 0.05)
#' env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4),
#'                                         origin = transl(1.5, 0, 0)))
#' robot_check_collision(robot, c(0, 0), env)$in_collision
#' robot_check_collision(robot, c(pi / 2, 0), env)$in_collision
#' @export
robot_check_collision <- function(robot, q, environment = NULL, self = TRUE,
                                  margin = 0, ignore_pairs = NULL,
                                  distances = FALSE) {
  check_robot(robot)
  q <- check_q(robot, q)
  margin <- check_scalar(margin, "margin", nonnegative = TRUE)
  fk <- fk_internal(robot, q)
  objs <- robot_collision_objects(robot, fk)
  rows <- list()
  hit <- FALSE
  eval_pair <- function(ga, TA, gb, TB) {
    if (!distances) {
      gap <- vnorm(TA[1:3, 4] - TB[1:3, 4]) - geometry_bound(ga) -
        geometry_bound(gb)
      if (gap > margin) return(NULL)
    }
    geometry_pair(ga, TA, gb, TB)
  }
  if (!is.null(environment)) {
    stopifnot(inherits(environment, "robot_environment"))
    for (o in objs) {
      for (nm in names(environment$obstacles)) {
        ob <- environment$obstacles[[nm]]
        if (!collidable(ob)) next
        r <- eval_pair(o$geometry, o$T, ob, ob$origin)
        if (is.null(r)) next
        col <- r$distance <= margin
        hit <- hit || col
        rows[[length(rows) + 1]] <- data.frame(
          object_a = robot$link_names[o$link], object_b = nm,
          type = "environment", distance = r$distance, collision = col,
          stringsAsFactors = FALSE)
      }
    }
  }
  if (self) {
    ign <- lapply(ignore_pairs %||% list(), function(p)
      sort(match(p, robot$link_names)))
    for (pr in default_self_pairs(robot, objs)) {
      a <- objs[[pr[1]]]
      b <- objs[[pr[2]]]
      key <- sort(c(a$link, b$link))
      if (any(vapply(ign, function(z) identical(z, key), logical(1)))) next
      r <- eval_pair(a$geometry, a$T, b$geometry, b$T)
      if (is.null(r)) next
      col <- r$distance <= margin
      hit <- hit || col
      rows[[length(rows) + 1]] <- data.frame(
        object_a = robot$link_names[a$link], object_b = robot$link_names[b$link],
        type = "self", distance = r$distance, collision = col,
        stringsAsFactors = FALSE)
    }
  }
  pairs <- if (length(rows)) do.call(rbind, rows) else
    data.frame(object_a = character(0), object_b = character(0),
               type = character(0), distance = numeric(0),
               collision = logical(0))
  structure(list(in_collision = hit,
                 min_distance = if (nrow(pairs)) min(pairs$distance) else NA_real_,
                 pairs = pairs, q = q),
            class = "robot_collision_result")
}

#' @export
print.robot_collision_result <- function(x, ...) {
  cat(sprintf("<robot_collision_result> %s; %d pair%s evaluated",
              if (x$in_collision) "IN COLLISION" else "collision free",
              nrow(x$pairs), if (nrow(x$pairs) == 1) "" else "s"))
  if (!is.na(x$min_distance)) cat(sprintf(", min distance %.4g", x$min_distance))
  cat("\n")
  if (any(x$pairs$collision)) print(x$pairs[x$pairs$collision, ], row.names = FALSE)
  invisible(x)
}

# Fast boolean collision test used by planners: early exit, no data frames.
in_collision_fast <- function(robot, q, environment, self, margin,
                              self_pairs = NULL) {
  fk <- fk_internal(robot, q)
  objs <- robot_collision_objects(robot, fk)
  if (!length(objs)) return(FALSE)
  bound <- vapply(objs, function(o) geometry_bound(o$geometry), 0)
  near <- function(TA, bA, TB, bB) {
    vnorm(TA[1:3, 4] - TB[1:3, 4]) - bA - bB <= margin
  }
  if (!is.null(environment)) {
    for (ob in environment$obstacles) {
      if (!collidable(ob)) next
      bo <- geometry_bound(ob)
      for (k in seq_along(objs)) {
        o <- objs[[k]]
        if (!near(o$T, bound[k], ob$origin, bo)) next
        if (geometry_pair(o$geometry, o$T, ob, ob$origin)$distance <= margin) {
          return(TRUE)
        }
      }
    }
  }
  if (self) {
    pairs <- self_pairs %||% default_self_pairs(robot, objs)
    for (pr in pairs) {
      a <- objs[[pr[1]]]
      b <- objs[[pr[2]]]
      if (!near(a$T, bound[pr[1]], b$T, bound[pr[2]])) next
      if (geometry_pair(a$geometry, a$T, b$geometry, b$T)$distance <= margin) {
        return(TRUE)
      }
    }
  }
  FALSE
}

interpolate_segment <- function(a, b, resolution) {
  n <- max(1, ceiling(max(abs(b - a)) / resolution))
  s <- seq(0, 1, length.out = n + 1)
  outer(s, b - a) + matrix(a, n + 1, length(a), byrow = TRUE)
}

#' Collision checking along a path
#'
#' Checks the straight joint-space segments between consecutive waypoints
#' by sampling them so that no joint moves more than `resolution` between
#' samples (radians or meters). This discrete check can miss collisions
#' with obstacles thinner than the swept motion between samples; it is not
#' continuous swept-volume checking.
#'
#' @inheritParams robot_check_collision
#' @param path Matrix of waypoints (one per row) or a `robot_plan`.
#' @param resolution Maximum joint step between checked samples.
#' @param check_limits Also require samples to respect joint limits.
#' @return A list with `valid`, the index of the first invalid sample
#'   (`NA` when valid), the invalid configuration, and the number of
#'   samples checked.
#' @examples
#' robot <- robot_capsule_links(robot_example("planar2"), 0.05)
#' env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4),
#'                                         origin = transl(1.5, 0, 0)))
#' robot_check_path(robot, rbind(c(pi / 2, 0), c(-pi / 2, 0)), env)$valid
#' @export
robot_check_path <- function(robot, path, environment = NULL, self = TRUE,
                             resolution = 0.02, margin = 0,
                             check_limits = TRUE) {
  check_robot(robot)
  if (inherits(path, "robot_plan")) path <- path$path
  P <- check_matrix(path, NULL, robot$dof, "path")
  resolution <- check_scalar(resolution, "resolution", positive = TRUE)
  samples <- if (nrow(P) == 1) P else do.call(rbind, lapply(seq_len(nrow(P) - 1),
    function(i) {
      S <- interpolate_segment(P[i, ], P[i + 1, ], resolution)
      if (i > 1) S[-1, , drop = FALSE] else S
    }))
  for (i in seq_len(nrow(samples))) {
    q <- samples[i, ]
    bad <- (check_limits && !robot_within_limits(robot, q)) ||
      in_collision_fast(robot, q, environment, self, margin)
    if (bad) {
      return(list(valid = FALSE, first_invalid = i, q = q,
                  samples = nrow(samples)))
    }
  }
  list(valid = TRUE, first_invalid = NA_integer_, q = NULL,
       samples = nrow(samples))
}
