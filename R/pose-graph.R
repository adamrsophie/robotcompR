# SE(2) pose graph optimization.

v2t <- function(p) {
  M <- diag(3)
  M[1:2, 1:2] <- so2_exp(p[3])
  M[1:2, 3] <- p[1:2]
  M
}

#' SE(2) pose graphs
#'
#' `robot_pose_graph()` creates a graph with initial node poses (one
#' `c(x, y, theta)` per row). `pose_graph_add_edge()` adds a relative pose
#' measurement \eqn{z_{ij}}, the pose of node j in the frame of node i,
#' with a 3 by 3 information matrix (inverse covariance, ordered
#' x, y, theta).
#'
#' @param nodes Matrix of initial poses.
#' @param graph A `robot_pose_graph`.
#' @param from,to Node indices.
#' @param measurement Relative pose `c(dx, dy, dtheta)`.
#' @param information Information matrix.
#' @return A `robot_pose_graph`.
#' @examples
#' g <- robot_pose_graph(rbind(c(0, 0, 0), c(1.1, 0.1, 0.05)))
#' g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0), diag(3))
#' @export
robot_pose_graph <- function(nodes) {
  X <- check_matrix(nodes, NULL, 3, "nodes")
  X[, 3] <- wrap_angle_internal(X[, 3])
  structure(list(nodes = X, edges = list()), class = "robot_pose_graph")
}

#' @rdname robot_pose_graph
#' @export
pose_graph_add_edge <- function(graph, from, to, measurement,
                                information = diag(3)) {
  stopifnot(inherits(graph, "robot_pose_graph"))
  n <- nrow(graph$nodes)
  from <- check_count(from, "from")
  to <- check_count(to, "to")
  if (from > n || to > n || from == to) {
    stop("Edge endpoints must be distinct existing nodes.", call. = FALSE)
  }
  Om <- check_matrix(information, 3, 3, "information")
  if (min(eigen((Om + t(Om)) / 2, only.values = TRUE)$values) <= 0) {
    stop("`information` must be positive definite.", call. = FALSE)
  }
  graph$edges[[length(graph$edges) + 1]] <- list(
    i = from, j = to, z = check_numeric_vector(measurement, 3, "measurement"),
    omega = (Om + t(Om)) / 2)
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

#' Optimize an SE(2) pose graph
#'
#' Minimizes \eqn{\sum e_{ij}^\top \Omega_{ij} e_{ij}} over the node poses
#' with Gauss-Newton iterations and Levenberg-Marquardt damping, using the
#' relative-pose residual \eqn{e_{ij} = (R_z^\top(R_i^\top(t_j - t_i) -
#' t_z), \mathrm{wrap}(\theta_j - \theta_i - \theta_z))}, analytic
#' Jacobians (Grisetti et al., 2010, "A tutorial on graph-based SLAM") and
#' sparse linear algebra from 'Matrix'. Node `fixed` is held constant to
#' remove the gauge freedom. Headings are wrapped after each update.
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
  X <- graph$nodes
  n <- nrow(X)
  cost_of <- function(X) sum(vapply(graph$edges, function(e) {
    r <- edge_error(X[e$i, ], X[e$j, ], e$z)
    sum(r * (e$omega %*% r))
  }, 0))
  cost <- cost_of(X)
  hist <- cost
  lambda <- 1e-6
  converged <- FALSE
  it <- 0L
  for (it in seq_len(max_iterations)) {
    ii <- integer(0); jj <- integer(0); vv <- numeric(0)
    b <- numeric(3 * n)
    for (e in graph$edges) {
      r <- edge_error(X[e$i, ], X[e$j, ], e$z)
      J <- edge_jacobians(X[e$i, ], X[e$j, ], e$z)
      blocks <- list(list(a = e$i, Ja = J$A), list(a = e$j, Ja = J$B))
      for (p in blocks) {
        ia <- 3 * (p$a - 1) + 1:3
        b[ia] <- b[ia] + as.numeric(t(p$Ja) %*% e$omega %*% r)
        for (q in blocks) {
          ib <- 3 * (q$a - 1) + 1:3
          H <- t(p$Ja) %*% e$omega %*% q$Ja
          ii <- c(ii, rep(ia, 3))
          jj <- c(jj, rep(ib, each = 3))
          vv <- c(vv, as.numeric(H))
        }
      }
    }
    H <- Matrix::sparseMatrix(i = ii, j = jj, x = vv, dims = c(3 * n, 3 * n))
    keep <- setdiff(seq_len(3 * n), 3 * (fixed - 1) + 1:3)
    Hk <- H[keep, keep]
    improved <- FALSE
    for (attempt in 1:20) {
      Hd <- Hk + Matrix::Diagonal(length(keep), lambda * max(Matrix::diag(Hk)))
      dx <- as.numeric(Matrix::solve(Hd, -b[keep]))
      Xn <- X
      step <- numeric(3 * n)
      step[keep] <- dx
      Xn <- Xn + matrix(step, n, 3, byrow = TRUE)
      Xn[, 3] <- wrap_angle_internal(Xn[, 3])
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
  graph$nodes <- X
  list(graph = graph, cost = hist, iterations = it, converged = converged)
}

#' @export
plot.robot_pose_graph <- function(x, ...) {
  X <- x$nodes
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
