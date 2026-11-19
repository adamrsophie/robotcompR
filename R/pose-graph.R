# SE(2) and SE(3) pose graph optimization.

v2t <- function(p) {
  M <- diag(3)
  M[1:2, 1:2] <- so2_exp(p[3])
  M[1:2, 3] <- p[1:2]
  M
}

se3_inverse_matrix <- function(T) {
  R <- T[1:3, 1:3]
  M <- diag(4)
  M[1:3, 1:3] <- t(R)
  M[1:3, 4] <- -t(R) %*% T[1:3, 4]
  M
}

as_se3_matrix <- function(x, name) {
  if (!is.matrix(x) && !inherits(x, "robot_pose")) {
    stop(sprintf("`%s` must be an SE(3) pose or 4 by 4 matrix.", name), call. = FALSE)
  }
  M <- as_plain_matrix(x)
  if (!identical(dim(M), c(4L, 4L))) {
    stop(sprintf("`%s` must be an SE(3) pose or 4 by 4 matrix.", name), call. = FALSE)
  }
  check_transform_matrix(M, name = name)
  M
}

#' Pose graphs
#'
#' `robot_pose_graph()` creates a graph with initial node poses: either a
#' matrix with one planar pose `c(x, y, theta)` per row (an SE(2) graph) or
#' a list of SE(3) `robot_pose` objects or 4 by 4 homogeneous matrices (an
#' SE(3) graph). `pose_graph_add_edge()` adds a relative pose measurement
#' \eqn{z_{ij}}, the pose of node j in the frame of node i, with an
#' information matrix (inverse covariance). For SE(2) the measurement is
#' `c(dx, dy, dtheta)` and the information matrix is 3 by 3 (x, y,
#' theta); for SE(3) the measurement is a `robot_pose` (or 4 by 4 matrix)
#' and the information matrix is 6 by 6, translation first, matching the
#' tangent ordering of [se3_log()].
#'
#' @param nodes Matrix of initial planar poses, or a list of SE(3) poses.
#' @param graph A `robot_pose_graph`.
#' @param from,to Node indices.
#' @param measurement Relative pose.
#' @param information Information matrix; defaults to the identity.
#' @return A `robot_pose_graph` with `nodes`, `edges` and `group`
#'   (`"SE2"` or `"SE3"`).
#' @examples
#' g <- robot_pose_graph(rbind(c(0, 0, 0), c(1.1, 0.1, 0.05)))
#' g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0), diag(3))
#' g3 <- robot_pose_graph(list(robot_pose(), transl(1.1, 0.1, 0)))
#' g3 <- pose_graph_add_edge(g3, 1, 2, transl(1, 0, 0))
#' @export
robot_pose_graph <- function(nodes) {
  if (is.list(nodes) && !is.data.frame(nodes) && !inherits(nodes, "robot_pose")) {
    X <- lapply(seq_along(nodes), function(i)
      as_se3_matrix(nodes[[i]], sprintf("nodes[[%d]]", i)))
    return(structure(list(nodes = lapply(X, function(M) robot_pose(matrix = M)),
                          edges = list(), group = "SE3"),
                     class = "robot_pose_graph"))
  }
  X <- check_matrix(nodes, NULL, 3, "nodes")
  X[, 3] <- wrap_angle_internal(X[, 3])
  structure(list(nodes = X, edges = list(), group = "SE2"),
            class = "robot_pose_graph")
}

pose_graph_group <- function(graph) graph$group %||% "SE2"

pose_graph_size <- function(graph) {
  if (pose_graph_group(graph) == "SE3") length(graph$nodes) else nrow(graph$nodes)
}

#' @rdname robot_pose_graph
#' @export
pose_graph_add_edge <- function(graph, from, to, measurement,
                                information = NULL) {
  stopifnot(inherits(graph, "robot_pose_graph"))
  se3 <- pose_graph_group(graph) == "SE3"
  dim <- if (se3) 6 else 3
  n <- pose_graph_size(graph)
  from <- check_count(from, "from")
  to <- check_count(to, "to")
  if (from > n || to > n || from == to) {
    stop("Edge endpoints must be distinct existing nodes.", call. = FALSE)
  }
  Om <- check_matrix(information %||% diag(dim), dim, dim, "information")
  if (min(eigen((Om + t(Om)) / 2, only.values = TRUE)$values) <= 0) {
    stop("`information` must be positive definite.", call. = FALSE)
  }
  z <- if (se3) as_se3_matrix(measurement, "measurement")
       else check_numeric_vector(measurement, 3, "measurement")
  graph$edges[[length(graph$edges) + 1]] <- list(
    i = from, j = to, z = z, omega = (Om + t(Om)) / 2)
  graph
}

edge_error <- function(xi, xj, z) {
  Ri <- so2_exp(xi[3])
  Rz <- so2_exp(z[3])
  dt <- as.numeric(t(Ri) %*% (xj[1:2] - xi[1:2]))
  c(as.numeric(t(Rz) %*% (dt - z[1:2])),
    wrap_angle_internal(xj[3] - xi[3] - z[3]))
}

edge_jacobians <- function(xi, xj, z) {
  Ri <- so2_exp(xi[3])
  Rz <- so2_exp(z[3])
  dRi <- matrix(c(-sin(xi[3]), -cos(xi[3]), cos(xi[3]), -sin(xi[3])), 2)
  dtt <- xj[1:2] - xi[1:2]
  A <- matrix(0, 3, 3)
  A[1:2, 1:2] <- -t(Rz) %*% t(Ri)
  A[1:2, 3] <- t(Rz) %*% dRi %*% dtt
  A[3, 3] <- -1
  B <- matrix(0, 3, 3)
  B[1:2, 1:2] <- t(Rz) %*% t(Ri)
  B[3, 3] <- 1
  list(A = A, B = B)
}

edge_error3 <- function(Xi, Xj, Z) {
  se3_log(se3_inverse_matrix(Z) %*% se3_inverse_matrix(Xi) %*% Xj)
}

# Right Jacobian of the SE(3) exponential, J with
# exp(xi + d) = exp(xi) exp(J d) to first order. Only exponentials of xi
# and logarithms near the identity are evaluated, so it is smooth even
# when xi is a rotation by pi.
# ponytail: central differences of exp (12 small logs); use the closed
# form with the Q block if large graphs make this the bottleneck.
se3_right_jacobian <- function(xi, h = 1e-6) {
  E0inv <- se3_exp(-xi)
  J <- matrix(0, 6, 6)
  for (k in 1:6) {
    # The step is relative to the component so a large translation does
    # not round it away.
    d <- replace(numeric(6), k, h * max(1, abs(xi[k])))
    J[, k] <- (se3_log(E0inv %*% se3_exp(xi + d)) -
                 se3_log(E0inv %*% se3_exp(xi - d))) / (2 * d[k])
  }
  J
}

# Jacobians of e = log(Z^-1 Xi^-1 Xj) with respect to right perturbations
# X exp(delta) of each endpoint: B = Jr(e)^-1 and A = -B Ad(Xj^-1 Xi).
# They are taken in the chart of the residual actually returned by the
# logarithm, so they stay consistent across the branch cut at pi, where
# differencing residuals on opposite sides of the cut would not.
edge_jacobians3 <- function(Xi, Xj, Z) {
  e <- edge_error3(Xi, Xj, Z)
  # J = [Jr Q; 0 Jr] (translation first): invert by blocks, which stays
  # exact when Q is huge next to Jr (large translations).
  J <- se3_right_jacobian(e)
  Ai <- solve(J[4:6, 4:6])
  B <- matrix(0, 6, 6)
  B[1:3, 1:3] <- solve(J[1:3, 1:3])
  B[4:6, 4:6] <- Ai
  B[1:3, 4:6] <- -B[1:3, 1:3] %*% J[1:3, 4:6] %*% Ai
  A <- -B %*% se3_adjoint_matrix(se3_inverse_matrix(Xj) %*% Xi)
  list(A = A, B = B)
}

#' Optimize a pose graph
#'
#' Minimizes \eqn{\sum e_{ij}^\top \Omega_{ij} e_{ij}} over the node poses
#' with Gauss-Newton iterations and Levenberg-Marquardt damping. For SE(2)
#' graphs the residual is \eqn{e_{ij} = (R_z^\top(R_i^\top(t_j - t_i) -
#' t_z), \mathrm{wrap}(\theta_j - \theta_i - \theta_z))}, analytic
#' Jacobians (Grisetti et al., 2010, "A tutorial on graph-based SLAM") and
#' sparse linear algebra from 'Matrix', and headings are wrapped after
#' each update. For SE(3) graphs the residual is
#' \eqn{e_{ij} = \log(Z_{ij}^{-1} X_i^{-1} X_j)} (translation first), nodes
#' are updated on the manifold as \eqn{X \exp(\delta)}, and the
#' Jacobians with respect to \eqn{\delta} are
#' \eqn{J_r(e)^{-1}} and \eqn{-J_r(e)^{-1} \mathrm{Ad}(X_j^{-1} X_i)},
#' taken in the chart of the residual returned by the logarithm so that
#' they stay valid when a residual rotation is near \eqn{\pi}. Node
#' `fixed` is held constant to remove the gauge freedom; every node must be
#' connected to it through edges.
#'
#' Convergence means stationarity, not merely a small change: the
#' optimizer stops with `status = "converged"` when every residual
#' component is at roundoff level for the magnitudes involved, or every
#' gradient component is negligible on its own scale
#' (\eqn{|g_k| \le 10^{-6} \sqrt{H_{kk}} \sqrt{c}}). A very precise
#' coordinate therefore cannot hide an error in another. A graph with a
#' single node is returned unchanged as converged. Other outcomes are
#' `"stalled"` (the relative decrease fell below `tol`, or no damped step
#' decreased the cost, while the gradient is not negligible),
#' `"max_iterations"`, `"linear_solve_failed"` and `"invalid_graph"`
#' (nodes not connected to `fixed`; the graph is returned unchanged with a
#' warning). A noisy graph converges to a positive cost.
#'
#' @param graph A `robot_pose_graph`.
#' @param fixed Index of the anchored node.
#' @param max_iterations Maximum iterations (0 only evaluates the graph).
#' @param tol Relative cost decrease treated as no progress.
#' @return A list with the optimized `graph`, `cost` history, `iterations`,
#'   `converged` (`TRUE` only for `status == "converged"`), `status` and
#'   `gradient`, the largest scaled gradient component at the start of
#'   the last iteration.
#' @examples
#' g <- robot_pose_graph(rbind(c(0, 0, 0), c(1.2, 0.1, 0.1), c(1.9, 1.2, 1.7)))
#' g <- pose_graph_add_edge(g, 1, 2, c(1, 0, pi / 2))
#' g <- pose_graph_add_edge(g, 2, 3, c(1, 0, pi / 2))
#' res <- pose_graph_optimize(g)
#' round(res$graph$nodes, 6)
#' @export
pose_graph_optimize <- function(graph, fixed = 1, max_iterations = 50,
                                tol = 1e-12) {
  stopifnot(inherits(graph, "robot_pose_graph"))
  se3 <- pose_graph_group(graph) == "SE3"
  bs <- if (se3) 6L else 3L
  n <- pose_graph_size(graph)
  fixed <- check_count(fixed, "fixed")
  if (fixed > n) stop("`fixed` must be an existing node.", call. = FALSE)
  max_iterations <- check_count(max_iterations, "max_iterations", min = 0)
  tol <- check_scalar(tol, "tol", nonnegative = TRUE)
  if (n == 1L) {
    return(list(graph = graph, cost = 0, iterations = 0L, converged = TRUE,
                status = "converged", gradient = 0))
  }
  X <- if (se3) lapply(graph$nodes, as_plain_matrix) else graph$nodes
  residual <- function(X, e) {
    if (se3) edge_error3(X[[e$i]], X[[e$j]], e$z)
    else edge_error(X[e$i, ], X[e$j, ], e$z)
  }
  jacobians <- function(X, e) {
    if (se3) edge_jacobians3(X[[e$i]], X[[e$j]], e$z)
    else edge_jacobians(X[e$i, ], X[e$j, ], e$z)
  }
  retract <- function(X, step) {
    if (se3) {
      return(lapply(seq_len(n), function(a) {
        d <- step[bs * (a - 1) + 1:bs]
        if (all(d == 0)) X[[a]] else X[[a]] %*% se3_exp(d)
      }))
    }
    Xn <- X + matrix(step, n, 3, byrow = TRUE)
    Xn[, 3] <- wrap_angle_internal(Xn[, 3])
    Xn
  }
  cost_of <- function(X) sum(vapply(graph$edges, function(e) {
    r <- residual(X, e)
    sum(r * (e$omega %*% r))
  }, 0))
  # Every node must be tied to the anchor, or its gauge is free:
  # breadth-first search over an adjacency list.
  ei <- vapply(graph$edges, `[[`, 0, "i")
  ej <- vapply(graph$edges, `[[`, 0, "j")
  adj <- split(c(ej, ei), factor(c(ei, ej), levels = seq_len(n)))
  seen <- replace(logical(n), fixed, TRUE)
  frontier <- fixed
  while (length(frontier)) {
    nb <- unique(unlist(adj[frontier], use.names = FALSE))
    frontier <- nb[!seen[nb]]
    seen[frontier] <- TRUE
  }
  if (!all(seen)) {
    lost <- which(!seen)
    warning(sprintf("Node%s %s %s not connected to the fixed node %d.",
                    if (length(lost) > 1) "s" else "",
                    paste(lost, collapse = ", "),
                    if (length(lost) > 1) "are" else "is", fixed), call. = FALSE)
    return(list(graph = graph, cost = cost_of(X), iterations = 0L,
                converged = FALSE, status = "invalid_graph",
                gradient = NA_real_))
  }
  # Residuals at roundoff level: each component is compared with about
  # 1000 machine epsilons times the magnitudes it was computed from, so a
  # precise coordinate never sets the tolerance of another.
  position <- function(X, a) if (se3) X[[a]][1:3, 4] else X[a, 1:2]
  at_roundoff <- function(X) all(vapply(graph$edges, function(e) {
    r <- residual(X, e)
    zt <- if (se3) e$z[1:3, 4] else e$z[1:2]
    mag <- 1 + vnorm(zt) + vnorm(position(X, e$i)) + vnorm(position(X, e$j))
    scale <- 1e3 * .Machine$double.eps *
      (if (se3) c(rep(mag, 3), rep(1, 3)) else c(mag, mag, 1))
    all(abs(r) <= scale)
  }, logical(1)))
  cost <- cost_of(X)
  hist <- cost
  lambda <- 1e-6
  status <- "max_iterations"
  grad <- NA_real_
  prev_small <- FALSE
  it <- 0L
  nb2 <- 4L * bs * bs
  keep <- setdiff(seq_len(bs * n), bs * (fixed - 1) + seq_len(bs))
  while (it < max_iterations) {
    it <- it + 1L
    ii <- jj <- integer(nb2 * length(graph$edges))
    vv <- numeric(nb2 * length(graph$edges))
    pos <- 0L
    b <- numeric(bs * n)
    lin_failed <- FALSE
    for (e in graph$edges) {
      r <- residual(X, e)
      J <- tryCatch(jacobians(X, e), error = function(err) NULL)
      if (is.null(J) || any(!is.finite(J$A)) || any(!is.finite(J$B))) {
        lin_failed <- TRUE
        break
      }
      blocks <- list(list(a = e$i, Ja = J$A), list(a = e$j, Ja = J$B))
      for (p in blocks) {
        ia <- bs * (p$a - 1) + seq_len(bs)
        b[ia] <- b[ia] + as.numeric(t(p$Ja) %*% e$omega %*% r)
        for (q in blocks) {
          ib <- bs * (q$a - 1) + seq_len(bs)
          idx <- pos + seq_len(bs * bs)
          ii[idx] <- rep(ia, bs)
          jj[idx] <- rep(ib, each = bs)
          vv[idx] <- as.numeric(t(p$Ja) %*% e$omega %*% q$Ja)
          pos <- pos + bs * bs
        }
      }
    }
    if (lin_failed) {
      status <- "linear_solve_failed"
      break
    }
    H <- Matrix::sparseMatrix(i = ii, j = jj, x = vv, dims = c(bs * n, bs * n))
    Hk <- H[keep, keep]
    hd <- Matrix::diag(Hk)
    # Componentwise scaled gradient, formed without squaring so extreme
    # information neither overflows nor underflows.
    sc <- sqrt(pmax(hd, .Machine$double.xmin)) * sqrt(max(cost, .Machine$double.xmin))
    grad <- max(abs(b[keep]) / sc)
    if (at_roundoff(X) || grad <= 1e-6) {
      status <- "converged"
      break
    }
    improved <- FALSE
    solve_failed <- FALSE
    for (attempt in 1:20) {
      # Marquardt damping on each diagonal entry, solved after symmetric
      # Jacobi equilibration so information spanning many orders of
      # magnitude stays well conditioned.
      dd <- ifelse(hd > 0, hd, 1)
      Di <- Matrix::Diagonal(x = 1 / sqrt(dd))
      Hs <- Di %*% Hk %*% Di + Matrix::Diagonal(length(keep), lambda)
      dx <- tryCatch(as.numeric(Di %*% Matrix::solve(Hs, -as.numeric(Di %*% b[keep]))),
                     error = function(e) NULL)
      if (is.null(dx) || any(!is.finite(dx))) {
        solve_failed <- TRUE
        break
      }
      step <- numeric(bs * n)
      step[keep] <- dx
      Xn <- retract(X, step)
      cn <- cost_of(Xn)
      if (cn <= cost) {
        improved <- TRUE
        break
      }
      lambda <- lambda * 10
    }
    if (solve_failed) {
      status <- "linear_solve_failed"
      break
    }
    if (!improved) {
      status <- if (at_roundoff(X)) "converged" else "stalled"
      break
    }
    rel <- (cost - cn) / max(cost, .Machine$double.eps)
    X <- Xn
    cost <- cn
    hist <- c(hist, cost)
    lambda <- max(lambda / 10, 1e-12)
    if (at_roundoff(X)) {
      status <- "converged"
      break
    }
    if (rel < tol) {
      # Tiny progress is success only at a stationary point; the next
      # iteration's gradient test decides, otherwise the run has stalled.
      status <- if (it < max_iterations) "pending" else "max_iterations"
      if (identical(prev_small, TRUE)) {
        status <- "stalled"
        break
      }
      prev_small <- TRUE
    } else {
      prev_small <- FALSE
    }
  }
  if (identical(status, "pending")) status <- "max_iterations"
  graph$nodes <- if (se3) lapply(X, function(M) robot_pose(matrix = orthonormalize_pose(M)))
                 else X
  list(graph = graph, cost = hist, iterations = it,
       converged = status == "converged", status = status, gradient = grad)
}

#' @export
plot.robot_pose_graph <- function(x, ...) {
  X <- if (pose_graph_group(x) == "SE3") {
    t(vapply(x$nodes, function(p) as_plain_matrix(p)[1:3, 4], numeric(3)))
  } else x$nodes
  graphics::plot(X[, 1], X[, 2], type = "n", asp = 1, xlab = "x (m)",
                 ylab = "y (m)", ...)
  for (e in x$edges) {
    consecutive <- abs(e$j - e$i) == 1
    graphics::segments(X[e$i, 1], X[e$i, 2], X[e$j, 1], X[e$j, 2],
                       col = if (consecutive) "grey40" else "firebrick",
                       lty = if (consecutive) 1 else 2)
  }
  graphics::points(X[, 1], X[, 2], pch = 19, cex = 0.6)
  invisible(x)
}

# Re-orthonormalize the rotation block after many retractions.
orthonormalize_pose <- function(M) {
  M[1:3, 1:3] <- as_plain_matrix(orthonormalize(M[1:3, 1:3]))
  M
}
