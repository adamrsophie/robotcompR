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

# Jacobians of the SE(3) residual with respect to right perturbations
# X exp(delta) of each endpoint, by central differences in the tangent
# space.
# ponytail: numerical 6 x 12 Jacobians; switch to the closed form with the
# SE(3) right Jacobian if large graphs make this the bottleneck.
edge_jacobians3 <- function(Xi, Xj, Z, h = 1e-7) {
  A <- B <- matrix(0, 6, 6)
  for (k in 1:6) {
    d <- replace(numeric(6), k, h)
    Ep <- se3_exp(d)
    Em <- se3_exp(-d)
    A[, k] <- (edge_error3(Xi %*% Ep, Xj, Z) - edge_error3(Xi %*% Em, Xj, Z)) / (2 * h)
    B[, k] <- (edge_error3(Xi, Xj %*% Ep, Z) - edge_error3(Xi, Xj %*% Em, Z)) / (2 * h)
  }
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
#' Jacobians with respect to \eqn{\delta} are computed by central
#' differences. Node `fixed` is held constant to remove the gauge freedom.
#'
#' @param graph A `robot_pose_graph`.
#' @param fixed Index of the anchored node.
#' @param max_iterations Maximum iterations.
#' @param tol Stop when the relative decrease of the cost is below `tol`.
#' @return A list with the optimized `graph`, `cost` history, `iterations`
#'   and `converged`.
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
  if (!length(graph$edges)) stop("The graph has no edges.", call. = FALSE)
  se3 <- pose_graph_group(graph) == "SE3"
  bs <- if (se3) 6L else 3L
  n <- pose_graph_size(graph)
  fixed <- check_count(fixed, "fixed")
  if (fixed > n) stop("`fixed` must be an existing node.", call. = FALSE)
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
  cost <- cost_of(X)
  hist <- cost
  lambda <- 1e-6
  converged <- FALSE
  it <- 0L
  for (it in seq_len(max_iterations)) {
    ii <- integer(0); jj <- integer(0); vv <- numeric(0)
    b <- numeric(bs * n)
    for (e in graph$edges) {
      r <- residual(X, e)
      J <- jacobians(X, e)
      blocks <- list(list(a = e$i, Ja = J$A), list(a = e$j, Ja = J$B))
      for (p in blocks) {
        ia <- bs * (p$a - 1) + seq_len(bs)
        b[ia] <- b[ia] + as.numeric(t(p$Ja) %*% e$omega %*% r)
        for (q in blocks) {
          ib <- bs * (q$a - 1) + seq_len(bs)
          H <- t(p$Ja) %*% e$omega %*% q$Ja
          ii <- c(ii, rep(ia, bs))
          jj <- c(jj, rep(ib, each = bs))
          vv <- c(vv, as.numeric(H))
        }
      }
    }
    H <- Matrix::sparseMatrix(i = ii, j = jj, x = vv, dims = c(bs * n, bs * n))
    keep <- setdiff(seq_len(bs * n), bs * (fixed - 1) + seq_len(bs))
    Hk <- H[keep, keep]
    improved <- FALSE
    for (attempt in 1:20) {
      Hd <- Hk + Matrix::Diagonal(length(keep), lambda * max(Matrix::diag(Hk)))
      dx <- as.numeric(Matrix::solve(Hd, -b[keep]))
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
    if (!improved) break
    rel <- (cost - cn) / max(cost, .Machine$double.eps)
    X <- Xn
    cost <- cn
    hist <- c(hist, cost)
    lambda <- max(lambda / 10, 1e-12)
    if (rel < tol || cost < 1e-24) {
      converged <- TRUE
      break
    }
  }
  graph$nodes <- if (se3) lapply(X, function(M) robot_pose(matrix = orthonormalize_pose(M)))
                 else X
  list(graph = graph, cost = hist, iterations = it, converged = converged)
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
