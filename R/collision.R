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
      # Least squares on the edge vectors themselves (QR), not the normal
      # equations, so thin but genuine faces are kept: squaring the
      # conditioning used to discard them.
      D <- t(P[-1, , drop = FALSE]) - P[1, ]
      qd <- qr(D, tol = 1e-13)
      if (qd$rank < ncol(D)) next
      mu <- qr.coef(qd, -P[1, ])
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
  scale <- max(geometry_extent(gA), geometry_extent(gB), .Machine$double.xmin)
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
    if (vn2 <= tol^2) {
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
  if (dist <= tol) {
    return(list(distance = dist, pa = pa, pb = pb, intersect = TRUE,
                lower = -Inf))
  }
  # |v| is only an upper bound: v is a point of A - B. Separation is
  # certified by the support plane: every point of A - B lies on the far
  # side of the plane through the support point along -v, so its offset
  # bounds the distance from below whatever stopped the iteration
  # (convergence, a repeated support point or the budget).
  w <- geometry_support(gA, TA, -v) - geometry_support(gB, TB, v)
  lower <- sum(v * w) / dist - 8 * .Machine$double.eps * (scale + vnorm(cA - cB))
  list(distance = dist, pa = pa, pb = pb, intersect = FALSE, lower = lower)
}

# Support value h(n) = max over A - B of n . x for unit directions n (the
# rows of N). For every n the signed distance between the cores is at
# least -h(n), whether they overlap or not, so the smallest value over a
# set of directions caps the penetration depth from above.
depth_cap <- function(gA, TA, gB, TB, N = NULL) {
  N <- rbind(diag(3), -diag(3), t(TA[1:3, 1:3]), -t(TA[1:3, 1:3]),
             t(TB[1:3, 1:3]), -t(TB[1:3, 1:3]), N)
  N <- N / sqrt(rowSums(N^2))
  h <- vapply(seq_len(nrow(N)), function(i) {
    n <- N[i, ]
    sum(n * (geometry_support(gA, TA, n) - geometry_support(gB, TB, -n)))
  }, 0)
  scale <- max(geometry_extent(gA), geometry_extent(gB)) +
    vnorm(TA[1:3, 4] - TB[1:3, 4])
  min(h) + 8 * .Machine$double.eps * scale
}

# Expanding polytope algorithm (van den Bergen, 2001) for the penetration
# depth of two intersecting convex cores. The polytope is the convex hull
# of support points of the Minkowski difference A - B. Faces are kept with
# outward normals; while the origin lies outside the hull the face it lies
# beyond is expanded, and once inside, the face nearest the origin is
# expanded until its support point does not move the face outward. Returns
# the depth (distance from the origin to the boundary of A - B), the unit
# direction `n` (moving A by -depth * n separates the cores) and the
# extreme points of each core along that direction, or NULL when A - B is
# flat to tolerance (then the depth is bounded by its thickness and flagged
# as not converged unless that is zero). The nearest face distance is a lower bound
# on the depth and the smallest support value seen along a face normal is
# an upper bound; `converged` is TRUE when they agree to `tol`. Otherwise
# the iteration budget ran out and only the bounds are certified.
epa_depth <- function(gA, TA, gB, TB, max_iter = 200, rel_tol = 1e-9) {
  scale <- max(geometry_extent(gA), geometry_extent(gB), .Machine$double.xmin)
  tol <- rel_tol * scale
  support <- function(d) {
    a <- geometry_support(gA, TA, d)
    b <- geometry_support(gB, TB, -d)
    list(w = a - b, a = a, b = b)
  }
  dirs <- rbind(diag(3), -diag(3), as.matrix(expand.grid(c(-1, 1), c(-1, 1), c(-1, 1))))
  pts <- lapply(seq_len(nrow(dirs)), function(i) support(dirs[i, ]))
  W <- t(vapply(pts, `[[`, numeric(3), "w"))
  # Initial tetrahedron: two farthest points, then farthest from their line,
  # then farthest from their plane.
  i1 <- 1L
  i2 <- which.max(colSums((t(W) - W[i1, ])^2))
  u <- W[i2, ] - W[i1, ]
  if (sum(u * u) <= tol^2) return(NULL)
  dl <- apply(W, 1, function(x) vnorm(cross3(u, x - W[i1, ])))
  i3 <- which.max(dl)
  if (dl[i3] < tol * vnorm(u)) return(NULL)
  nrm <- cross3(u, W[i3, ] - W[i1, ])
  nrm <- nrm / vnorm(nrm)
  # Query both sides of the plane: fixed directions can miss the apex of
  # a full-dimensional difference.
  pts <- c(pts, list(support(nrm), support(-nrm)))
  W <- rbind(W, pts[[length(pts) - 1]]$w, pts[[length(pts)]]$w)
  off <- as.numeric(sweep(W, 2, W[i1, ]) %*% nrm)
  i4 <- which.max(abs(off))
  if (abs(off[i4]) < tol) {
    # A - B is flat to tolerance. The depth lies between 0 and the support
    # value of A - B (measured from the origin, not from a sampled point)
    # along the flat normal or any other direction.
    up <- max(0, depth_cap(gA, TA, gB, TB, rbind(nrm, -nrm, dirs)))
    return(list(depth = 0, n = nrm, pa = pts[[i1]]$a, pb = pts[[i1]]$b,
                converged = up == 0, iterations = 0L, bounds = c(0, up),
                flat = TRUE, inside = FALSE))
  }
  V <- pts[c(i1, i2, i3, i4)]
  P <- t(vapply(V, `[[`, numeric(3), "w"))
  inside <- colMeans(P)
  make_face <- function(ids) {
    a <- P[ids[1], ]; b <- P[ids[2], ]; c_ <- P[ids[3], ]
    n <- cross3(b - a, c_ - a)
    ln <- vnorm(n)
    if (ln < 1e-300) return(NULL)
    n <- n / ln
    if (sum(n * (a - inside)) < 0) {
      ids <- ids[c(1, 3, 2)]
      n <- -n
    }
    list(ids = ids, n = n, c = sum(n * a))
  }
  faces <- Filter(Negate(is.null), lapply(list(c(1, 2, 3), c(1, 2, 4),
                                               c(1, 3, 4), c(2, 3, 4)),
                                          make_face))
  best <- NULL
  upper <- Inf
  converged <- FALSE
  for (it in seq_len(max_iter)) {
    cs <- vapply(faces, `[[`, 0, "c")
    f <- faces[[which.min(cs)]]
    s <- support(f$n)
    upper <- min(upper, sum(f$n * s$w))
    gain <- sum(f$n * s$w) - f$c
    best <- f
    if (gain <= tol) {
      converged <- TRUE
      break
    }
    # Add the support point: drop faces it can see, stitch the horizon.
    P <- rbind(P, s$w)
    V[[length(V) + 1]] <- s
    k <- nrow(P)
    vis <- vapply(faces, function(g) sum(g$n * (s$w - P[g$ids[1], ])) > tol * 1e-3,
                  logical(1))
    edges <- do.call(rbind, lapply(faces[vis], function(g)
      rbind(g$ids[1:2], g$ids[2:3], g$ids[c(3, 1)])))
    key <- paste(edges[, 1], edges[, 2])
    rev_key <- paste(edges[, 2], edges[, 1])
    horizon <- edges[!(key %in% rev_key), , drop = FALSE]
    newf <- lapply(seq_len(nrow(horizon)), function(r)
      make_face(c(horizon[r, ], k)))
    faces <- c(faces[!vis], Filter(Negate(is.null), newf))
  }
  f <- best
  depth <- max(f$c, 0)
  # Barycentric coordinates of the closest boundary point on the face.
  A3 <- t(P[f$ids, ])
  lam <- tryCatch(as.numeric(qr.solve(rbind(A3, 1), c(f$n * f$c, 1))),
                  error = function(e) c(1, 1, 1) / 3)
  lam <- pmax(lam, 0)
  lam <- lam / sum(lam)
  pa <- Reduce(`+`, Map(function(v, l) v$a * l, V[f$ids], lam))
  pb <- Reduce(`+`, Map(function(v, l) v$b * l, V[f$ids], lam))
  # The polytope lies inside A - B, so a final nearest face in front of
  # the origin proves that the cores overlap by at least that much.
  list(depth = depth, n = f$n, pa = pa, pb = pb, converged = converged,
       iterations = it, bounds = c(depth, max(depth, upper)),
       inside = f$c > 0)
}

# Half the diameter of the core, independent of where its local origin is:
# the length scale for numerical tolerances.
geometry_extent <- function(g) {
  switch(g$type,
    sphere = 0,
    capsule = g$params$length / 2,
    box = vnorm(g$params$size) / 2,
    cylinder = sqrt(g$params$radius^2 + (g$params$length / 2)^2),
    convex = {
      V <- g$params$vertices
      ctr <- (apply(V, 2, max) + apply(V, 2, min)) / 2
      sqrt(max(rowSums(sweep(V, 2, ctr)^2)))
    },
    0) + geometry_radius(g)
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
  # The minimum is either an interior critical point of both segments or
  # lies on an endpoint of one of them. All candidates are evaluated; the
  # interior point uses the cross product |d1 x d2|^2, which does not
  # cancel for nearly parallel segments the way a e - b^2 does.
  proj <- function(p, o, d, len2) {
    if (len2 <= 0) 0 else min(1, max(0, sum((p - o) * d) / len2))
  }
  cand <- list(c(0, proj(p1, p2, d2, e)), c(1, proj(q1, p2, d2, e)),
               c(proj(p2, p1, d1, a), 0), c(proj(q2, p1, d1, a), 1))
  nx <- cross3(d1, d2)
  den <- sum(nx * nx)
  if (den > 0) {
    w <- p2 - p1
    s <- sum(cross3(w, d2) * nx) / den
    t <- sum(cross3(w, d1) * nx) / den
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) cand <- c(cand, list(c(s, t)))
  }
  best <- NULL
  for (st in cand) {
    pa <- p1 + d1 * st[1]
    pb <- p2 + d2 * st[2]
    dd <- vnorm(pa - pb)
    if (is.null(best) || dd < best$distance) best <- list(distance = dd, pa = pa, pb = pb)
  }
  best
}

core_distance <- function(gA, TA, gB, TB) {
  swept <- c("sphere", "capsule")
  if (gA$type %in% swept && gB$type %in% swept) {
    sa <- core_segment(gA, TA)
    sb <- core_segment(gB, TB)
    r <- segment_closest(sa$a, sa$b, sb$a, sb$b)
    r$intersect <- r$distance <= 1e-12 * max(geometry_extent(gA), geometry_extent(gB))
    r$lower <- r$distance
    return(r)
  }
  gjk_distance(gA, TA, gB, TB)
}

# A deterministic unit normal for cores that touch or overlap at a point:
# the direction between the core witnesses when they differ, otherwise
# perpendicular to the capsule axes, otherwise the world x axis.
touch_normal <- function(gA, TA, gB, TB, core) {
  d <- core$pb - core$pa
  if (vnorm(d) > 0) return(d / vnorm(d))
  axes <- list(TA[1:3, 3], TB[1:3, 3])[c(gA$type == "capsule", gB$type == "capsule")]
  if (length(axes) == 2) {
    n <- cross3(axes[[1]], axes[[2]])
    if (vnorm(n) > 0) return(n / vnorm(n))
  }
  if (length(axes)) {
    n <- cross3(axes[[1]], c(1, 0, 0))
    if (vnorm(n) < 1e-8) n <- cross3(axes[[1]], c(0, 1, 0))
    return(n / vnorm(n))
  }
  c(1, 0, 0)
}

geometry_pair <- function(gA, TA, gB, TB, depth = TRUE) {
  core <- core_distance(gA, TA, gB, TB)
  rA <- geometry_radius(gA)
  rB <- geometry_radius(gB)
  if (core$intersect) {
    # Swept pairs (spheres and capsules) have point or segment cores, whose
    # penetration is fully carried by the radii; other cores need EPA.
    # Callers that only need a yes/no answer skip EPA (depth = FALSE).
    ep <- if (!depth || (gA$type %in% c("sphere", "capsule") &&
                         gB$type %in% c("sphere", "capsule"))) NULL
          else epa_depth(gA, TA, gB, TB)
    if (is.null(ep)) {
      d <- -(rA + rB)
      n <- touch_normal(gA, TA, gB, TB, core)
      return(list(distance = d, collision = TRUE, depth_known = depth,
                  depth_bounds = if (depth) c(d, d) else c(NA_real_, NA_real_),
                  pa = core$pa + rA * n, pb = core$pb - rB * n))
    }
    return(list(distance = -(ep$depth + rA + rB), collision = TRUE,
                depth_known = ep$converged,
                depth_bounds = -(rev(ep$bounds) + rA + rB),
                pa = ep$pa + rA * ep$n, pb = ep$pb - rB * ep$n))
  }
  n <- (core$pb - core$pa) / core$distance
  dist <- core$distance - rA - rB
  if (core$lower <= 0) return(unresolved_pair(gA, TA, gB, TB, core, depth))
  lo <- min(core$lower, core$distance) - rA - rB
  list(distance = dist, collision = dist <= 0, depth_known = TRUE,
       depth_bounds = c(lo, dist), pa = core$pa + rA * n,
       pb = core$pb - rB * n)
}

# GJK found a positive candidate distance but could not prove separation
# (thin or extreme-aspect cores). EPA decides when it proves overlap;
# otherwise the pair counts as colliding and the reported interval rests
# only on support-function bounds, which hold whether the cores overlap or
# not.
unresolved_pair <- function(gA, TA, gB, TB, core, depth) {
  rA <- geometry_radius(gA)
  rB <- geometry_radius(gB)
  ep <- if (depth) epa_depth(gA, TA, gB, TB) else NULL
  if (!is.null(ep) && isTRUE(ep$inside)) {
    return(list(distance = -(ep$depth + rA + rB), collision = TRUE,
                depth_known = ep$converged,
                depth_bounds = -(rev(ep$bounds) + rA + rB),
                pa = ep$pa + rA * ep$n, pb = ep$pb - rB * ep$n))
  }
  v <- core$pa - core$pb
  lo <- -depth_cap(gA, TA, gB, TB, rbind(v, -v)) - rA - rB
  hi <- core$distance - rA - rB
  n <- (core$pb - core$pa) / core$distance
  list(distance = max(lo, min(hi, -(rA + rB))), collision = TRUE,
       depth_known = FALSE, depth_bounds = c(lo, hi),
       pa = core$pa + rA * n, pb = core$pb - rB * n)
}

#' Distance between two geometric primitives
#'
#' Computes the separation distance and closest points between two convex
#' primitives placed in a common frame. Each primitive is a convex core
#' plus a radius: GJK (Gilbert, Johnson and Keerthi, 1988) computes the
#' distance between the cores, and the radii are subtracted. The result
#' is accurate to the GJK tolerance for separated shapes. When shapes
#' with a radius overlap but their cores do not, the negative distance is
#' the exact penetration depth. When the cores themselves overlap, the
#' expanding polytope algorithm (EPA; van den Bergen, 2001) computes the
#' penetration depth of the cores, and the negative distance is that depth
#' plus both radii: the length of the shortest translation that separates
#' the shapes.
#' Separation is reported only when it is proved: GJK's distance is an
#' upper bound, and a support plane gives a lower bound that must be
#' positive. For extremely thin or high aspect ratio cores the lower bound
#' can fail to be positive although the candidate distance is; if EPA
#' then proves overlap its depth is used, and otherwise the pair is
#' reported as colliding with `depth_known = FALSE` and an interval from
#' support-function bounds. EPA stops when its lower and upper bounds on the depth
#' agree to a relative tolerance of about \eqn{10^{-9}} of the object size,
#' and `depth_known` is then `TRUE`. Curved cores in symmetric
#' configurations (for example two identical coaxial cylinders) can exhaust
#' the iteration budget first: `depth_known` is then `FALSE`, `distance`
#' is the conservative (shallowest) estimate and `distance_bounds` brackets
#' the true signed distance. A pair is never reported as separated unless
#' separation is proved; unresolved contact counts as a collision. When the
#' cores intersect, `point_a` and `point_b` are the extreme
#' points of each shape along the separating direction; when only the
#' radii overlap they are the deepest points of each shape. Meshes are not
#' supported.
#'
#' @param a,b `robot_geometry` objects.
#' @param pose_a,pose_b Poses of the frames the geometries are attached to.
#' @return A list with `distance`, `collision`, `depth_known`,
#'   `distance_bounds` (a length-2 interval containing the exact signed
#'   distance up to floating point roundoff; for separated shapes it runs
#'   from the certified lower bound to `distance`) and the closest
#'   points `point_a` and `point_b`.
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
       depth_known = r$depth_known, distance_bounds = r$depth_bounds,
       point_a = r$pa, point_b = r$pb)
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
#' origins are interpreted in the world frame. Names must be unique;
#' unnamed obstacles get the name `obstacle<i>` (with a suffix if that name
#' is taken).
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
  nm[is.na(nm)] <- ""
  given <- nm[nzchar(nm)]
  if (anyDuplicated(given)) {
    stop(sprintf("Obstacle names must be unique; duplicated: %s.",
                 paste(unique(given[duplicated(given)]), collapse = ", ")),
         call. = FALSE)
  }
  for (i in which(!nzchar(nm))) nm[i] <- unique_name(paste0("obstacle", i), nm)
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
  # A link pair is excluded when no movable joint separates the links
  # (their relative pose never changes), or when exactly one movable joint
  # does and every link between them carries no collision geometry. The
  # second case covers joint neighbors and the geometry-free fixed frames
  # that URDF export inserts, while a link with geometry in between (a tool
  # fixed to the link after a joint) keeps its pair with the far link.
  has_geom <- logical(length(robot$link_names))
  has_geom[vapply(objs, function(o) o$link, integer(1))] <- TRUE
  chain <- function(k) {
    out <- k
    while (robot$parent_index[k] > 0) {
      k <- robot$parent_index[k]
      out <- c(out, k)
    }
    out
  }
  adjacent <- function(a, b) {
    ca <- chain(a)
    cb <- chain(b)
    top <- ca[ca %in% cb][1]
    up_a <- ca[seq_len(match(top, ca))]
    up_b <- cb[seq_len(match(top, cb))]
    below <- c(up_a[-length(up_a)], up_b[-length(up_b)])
    n_movable <- sum(robot$movable[below - 1L])
    inner <- setdiff(c(up_a, up_b), c(a, b))
    n_movable == 0 || (n_movable == 1 && !any(has_geom[inner]))
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
#' skips link pairs that no movable joint separates, and pairs separated
#' by one movable joint when no link between them carries collision
#' geometry (joint neighbors, possibly through geometry-free fixed frames);
#' a tool fixed to a link is still checked against the link before that
#' link's joint. Further link
#' pairs can be excluded with `ignore_pairs`. A cheap
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
      for (oi in seq_along(environment$obstacles)) {
        nm <- names(environment$obstacles)[oi]
        ob <- environment$obstacles[[oi]]
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
    ign <- lapply(ignore_pairs %||% list(), function(p) {
      m <- match(p, robot$link_names)
      if (length(p) != 2L || anyNA(m)) {
        stop("`ignore_pairs` must hold pairs of existing link names.",
             call. = FALSE)
      }
      sort(m)
    })
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
        if (geometry_pair(o$geometry, o$T, ob, ob$origin, FALSE)$distance <= margin) {
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
      if (geometry_pair(a$geometry, a$T, b$geometry, b$T, FALSE)$distance <= margin) {
        return(TRUE)
      }
    }
  }
  FALSE
}

interpolate_segment <- function(a, b, resolution) {
  n <- max(1, ceiling(max(abs(b - a)) / resolution))
  s <- seq(0, 1, length.out = n + 1)
  S <- outer(s, b - a) + matrix(a, n + 1, length(a), byrow = TRUE)
  # The endpoints are the waypoints themselves, never a + (b - a).
  S[1, ] <- a
  S[n + 1, ] <- b
  S
}

# Upper bounds on how far any point of each link's collision geometry can
# move per unit change of each joint: rho[j, k] for movable joint j and link
# k. A revolute joint moves a point at distance r from its axis origin by
# at most r per radian, and r is bounded by the triangle inequality over the
# fixed frame offsets between the joint and the link (prismatic joints in
# between add their largest extension over the segment from qa to qb,
# which is reached at an endpoint because the motion is linear). A
# prismatic joint moves points by at most one meter per meter.
motion_bounds <- function(robot, qa, qb) {
  nl <- length(robot$links)
  rho <- matrix(0, length(robot$joints), nl)
  tnorm <- function(M) if (is.null(M)) 0 else vnorm(M[1:3, 4])
  for (k in seq_len(nl)) {
    geoms <- Filter(collidable, robot$links[[k]]$collision)
    if (!length(geoms)) next
    acc <- max(vapply(geoms, function(g) tnorm(g$origin) + geometry_bound(g), 0))
    l <- k
    while (l > 1) {
      jm <- l - 1L
      J <- robot$joints[[jm]]
      if (J$type == "prismatic") {
        rho[jm, k] <- 1
      } else if (J$type != "fixed") {
        rho[jm, k] <- tnorm(J$post) + acc
      }
      ext <- if (J$type == "prismatic") {
        i <- robot$q_index[jm]
        max(abs(c(qa[i], qb[i]) + J$offset))
      } else 0
      acc <- tnorm(J$origin) + ext + tnorm(J$post) + acc
      l <- robot$parent_index[l]
    }
  }
  rho
}

# Conservative advancement (Mirtich, 1996; Zhang et al., 2006) along the
# straight joint-space segment from qa to qb. Returns NULL when the segment
# is free, otherwise the first parameter s in [0, 1] at which the clearance
# reaches the margin.
segment_conservative <- function(robot, qa, qb, environment, self, margin,
                                 tol, max_steps) {
  rho <- motion_bounds(robot, qa, qb)
  dq <- abs(qb - qa)
  jm <- which(robot$movable)
  # Motion bound per link for s running over [0, 1].
  M <- as.numeric(crossprod(rho[jm, , drop = FALSE], dq[robot$q_index[jm]]))
  s <- 0
  for (step in seq_len(max_steps)) {
    q <- if (s >= 1) qb else qa + s * (qb - qa)
    fk <- fk_internal(robot, q)
    objs <- robot_collision_objects(robot, fk)
    remaining <- 1 - s
    adv <- Inf
    clear <- function(ga, TA, gb, TB, motion) {
      if (motion <= 0) {
        lb <- -Inf
      } else {
        lb <- vnorm(TA[1:3, 4] - TB[1:3, 4]) - geometry_bound(ga) -
          geometry_bound(gb) - margin
        # The bound only skips the exact query when the pair cannot come
        # within `tol` of contact before the end of the segment.
        if (lb > tol && lb / motion > remaining) return(lb / motion)
      }
      # Advance only by the certified lower bound on the distance: the
      # reported distance can exceed it, and an unresolved pair counts as
      # contact.
      pr <- geometry_pair(ga, TA, gb, TB, FALSE)
      if (pr$collision) return(-1)
      d <- pr$depth_bounds[1] - margin
      if (d <= tol) return(-1)
      if (motion <= 0) Inf else d / motion
    }
    if (!is.null(environment)) {
      for (ob in environment$obstacles) {
        if (!collidable(ob)) next
        for (o in objs) {
          a <- clear(o$geometry, o$T, ob, ob$origin, M[o$link])
          if (a < 0) return(s)
          adv <- min(adv, a)
        }
      }
    }
    if (self) {
      for (pr in default_self_pairs(robot, objs)) {
        a_ <- objs[[pr[1]]]
        b_ <- objs[[pr[2]]]
        a <- clear(a_$geometry, a_$T, b_$geometry, b_$T, M[a_$link] + M[b_$link])
        if (a < 0) return(s)
        adv <- min(adv, a)
      }
    }
    # The final configuration is always checked explicitly: an advance
    # that exactly reaches the end of the segment may end in contact.
    if (remaining <= 0) return(NULL)
    s <- if (adv >= remaining) 1 else s + adv
  }
  warning("Conservative advancement reached `max_steps`; the segment is ",
          "reported as invalid.", call. = FALSE)
  s
}

#' Collision checking along a path
#'
#' Checks the straight joint-space segments between consecutive waypoints.
#'
#' With `method = "discrete"` (the default) each segment is sampled so that
#' no joint moves more than `resolution` between samples (radians or
#' meters). This can miss collisions with obstacles thinner than the swept
#' motion between samples.
#'
#' With `method = "continuous"` each segment is checked by conservative
#' advancement: at the current configuration the clearance \eqn{d} of
#' every geometry pair is computed, together with an upper bound \eqn{M}
#' on how far any point of the pair can move per unit of the segment
#' parameter (from the joint displacements and fixed bounds on the lever
#' arms of the kinematic chain). Advancing by \eqn{d / M} cannot create a
#' contact, so the motion is collision-free when the steps reach the end
#' of the segment and in collision as soon as a clearance drops to
#' `margin`. Both ends of every segment are checked with the same contact
#' rule as [robot_check_collision()], so a path ending in contact is
#' invalid. The lever-arm bounds use the prismatic extensions actually
#' reached on each segment, so they hold with `check_limits = FALSE` and
#' for joints without finite limits. The test is exact for the continuous
#' straight-line motion, up to `tol`.
#'
#' @inheritParams robot_check_collision
#' @param path Matrix of waypoints (one per row) or a `robot_plan`.
#' @param resolution Maximum joint step between checked samples
#'   (discrete method).
#' @param check_limits Also require samples to respect joint limits.
#' @param method `"discrete"` or `"continuous"`.
#' @param tol Clearance treated as contact by the continuous method.
#' @param max_steps Maximum advancement steps per segment (continuous
#'   method); reaching it reports the segment as invalid with a warning.
#' @return A list with `valid`, `first_invalid` (the index of the first
#'   invalid sample for the discrete method, or of the first invalid
#'   segment for the continuous method; `NA` when valid), the invalid
#'   configuration `q`, and `samples`, the number of configurations
#'   checked.
#' @examples
#' robot <- robot_capsule_links(robot_example("planar2"), 0.05)
#' env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4),
#'                                         origin = transl(1.5, 0, 0)))
#' robot_check_path(robot, rbind(c(pi / 2, 0), c(-pi / 2, 0)), env)$valid
#' @export
robot_check_path <- function(robot, path, environment = NULL, self = TRUE,
                             resolution = 0.02, margin = 0,
                             check_limits = TRUE,
                             method = c("discrete", "continuous"),
                             tol = 1e-9, max_steps = 10000) {
  check_robot(robot)
  method <- match.arg(method)
  if (inherits(path, "robot_plan")) path <- path$path
  P <- check_q_matrix(robot, path, "path")
  if (!nrow(P)) stop("`path` must have at least one row.", call. = FALSE)
  resolution <- check_scalar(resolution, "resolution", positive = TRUE)
  margin <- check_scalar(margin, "margin", nonnegative = TRUE)
  if (!is.null(environment) && !inherits(environment, "robot_environment")) {
    stop("`environment` must be a robot_environment (see robot_environment()).",
         call. = FALSE)
  }
  if (method == "continuous") {
    tol <- check_scalar(tol, "tol", nonnegative = TRUE)
    max_steps <- check_count(max_steps, "max_steps")
    # Problems are reported in path order: limits form a box, so a segment
    # respects them iff its ends do, and a collision inside segment i
    # comes before a limit violation at its end.
    off_limits <- function(i) check_limits && !robot_within_limits(robot, P[i, ])
    if (off_limits(1)) {
      return(list(valid = FALSE, first_invalid = 1L, q = P[1, ], samples = 1L))
    }
    if (nrow(P) == 1) P <- rbind(P, P)
    for (i in seq_len(nrow(P) - 1)) {
      s <- segment_conservative(robot, P[i, ], P[i + 1, ], environment, self,
                                margin, tol, max_steps)
      if (!is.null(s)) {
        q <- if (s >= 1) P[i + 1, ] else P[i, ] + s * (P[i + 1, ] - P[i, ])
        return(list(valid = FALSE, first_invalid = i, q = q,
                    samples = NA_integer_))
      }
      if (off_limits(i + 1)) {
        return(list(valid = FALSE, first_invalid = i, q = P[i + 1, ],
                    samples = NA_integer_))
      }
    }
    return(list(valid = TRUE, first_invalid = NA_integer_, q = NULL,
                samples = NA_integer_))
  }
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
