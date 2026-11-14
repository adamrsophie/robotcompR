# Sampling-based motion planning in configuration space.

planner_algorithms <- c("rrt", "rrt_connect", "rrt_star", "prm")

#' Motion planners
#'
#' Configures a sampling-based planner in joint space. Validity of a
#' configuration means: inside `bounds` (the joint limits by default) and,
#' when a robot is given, free of collisions with `environment` and of
#' self-collisions. A custom `validity` function `function(q)` returning
#' `TRUE` for valid states can replace or refine this, which also allows
#' planning for abstract point robots without a `robot_model`.
#'
#' Edges between states are straight lines in joint space, checked at
#' samples no farther apart than `resolution` in any coordinate. This is a
#' discrete check (see [robot_check_path()]). Distances are Euclidean in
#' joint space; continuous joints are treated as bounded to
#' \eqn{[-\pi, \pi]} without wrap-around.
#'
#' Algorithms:
#' * `"rrt"`: rapidly exploring random tree (LaValle, 1998) with goal bias.
#' * `"rrt_connect"`: bidirectional RRT with greedy connection (Kuffner and
#'   LaValle, 2000).
#' * `"rrt_star"`: RRT* (Karaman and Frazzoli, 2011) with rewiring in a
#'   shrinking radius. It is asymptotically optimal only as the number of
#'   samples grows without bound; with a finite `max_iterations` it returns
#'   the best path found so far.
#' * `"prm"`: probabilistic roadmap with `prm_nodes` samples and
#'   `prm_neighbors` nearest-neighbor connections, queried with Dijkstra's
#'   algorithm.
#'
#' @param robot Optional `robot_model`.
#' @param algorithm Planner name.
#' @param environment Optional `robot_environment`.
#' @param validity Optional function of a configuration.
#' @param bounds Matrix of `c(lower, upper)` rows; defaults to the robot's
#'   sampling bounds (see [robot_random_configuration()]).
#' @param self Check self-collision.
#' @param margin Collision margin.
#' @param step_size Maximum extension per tree step.
#' @param goal_bias Probability of sampling the goal (RRT and RRT*).
#' @param max_iterations Maximum number of samples.
#' @param resolution Edge checking resolution.
#' @param rewire_factor Scale \eqn{\gamma} of the RRT* radius
#'   \eqn{\min(\gamma (\log n / n)^{1/d}, \text{step\_size})}.
#' @param prm_nodes,prm_neighbors Roadmap size and connectivity.
#' @param time_limit Wall-clock budget in seconds for each call to
#'   [robot_plan()]; planning stops with `success = FALSE` once it is spent.
#' @return A `robot_planner`.
#' @examples
#' planner <- robot_planner(validity = function(q) sum((q - c(0.5, 0.5))^2) > 0.04,
#'                          bounds = rbind(c(0, 1), c(0, 1)),
#'                          algorithm = "rrt_connect")
#' plan <- robot_plan(planner, c(0.1, 0.1), c(0.9, 0.9), seed = 1)
#' plan$success
#' @export
robot_planner <- function(robot = NULL, algorithm = c("rrt_connect", "rrt",
                                                      "rrt_star", "prm"),
                          environment = NULL, validity = NULL, bounds = NULL,
                          self = TRUE, margin = 0, step_size = 0.3,
                          goal_bias = 0.05, max_iterations = 5000,
                          resolution = 0.02, rewire_factor = 2,
                          prm_nodes = 200, prm_neighbors = 8,
                          time_limit = Inf) {
  algorithm <- match.arg(algorithm)
  if (!is.null(robot)) check_robot(robot)
  if (is.null(robot) && is.null(validity)) {
    stop("Supply a `robot` or a `validity` function.", call. = FALSE)
  }
  if (is.null(bounds)) {
    if (is.null(robot)) stop("`bounds` are required without a robot.",
                             call. = FALSE)
    bounds <- sampling_bounds(robot)
  }
  bounds <- check_matrix(bounds, NULL, 2, "bounds")
  if (any(!is.finite(bounds)) || any(bounds[, 1] > bounds[, 2])) {
    stop("`bounds` must be finite with lower <= upper.", call. = FALSE)
  }
  if (!is.null(robot) && nrow(bounds) != robot$dof) {
    stop("`bounds` needs one row per degree of freedom.", call. = FALSE)
  }
  if (!is.null(validity) && !is.function(validity)) {
    stop("`validity` must be a function.", call. = FALSE)
  }
  structure(list(
    robot = robot, algorithm = algorithm, environment = environment,
    validity = validity, bounds = bounds, self = self,
    margin = check_scalar(margin, "margin", nonnegative = TRUE),
    step_size = check_scalar(step_size, "step_size", positive = TRUE),
    goal_bias = check_scalar(goal_bias, "goal_bias", nonnegative = TRUE),
    max_iterations = check_count(max_iterations, "max_iterations"),
    resolution = check_scalar(resolution, "resolution", positive = TRUE),
    rewire_factor = check_scalar(rewire_factor, "rewire_factor", positive = TRUE),
    prm_nodes = check_count(prm_nodes, "prm_nodes"),
    prm_neighbors = check_count(prm_neighbors, "prm_neighbors"),
    time_limit = if (identical(time_limit, Inf)) Inf else
      check_scalar(time_limit, "time_limit", positive = TRUE),
    deadline = Inf
  ), class = "robot_planner")
}

#' @export
print.robot_planner <- function(x, ...) {
  cat(sprintf("<robot_planner> %s in %d dimensions, step %g, resolution %g\n",
              x$algorithm, nrow(x$bounds), x$step_size, x$resolution))
  invisible(x)
}

state_valid <- function(pl, q) {
  if (any(q < pl$bounds[, 1] - 1e-12 | q > pl$bounds[, 2] + 1e-12)) {
    return(FALSE)
  }
  if (!is.null(pl$robot) && !robot_within_limits(pl$robot, q)) return(FALSE)
  if (!is.null(pl$robot) &&
      in_collision_fast(pl$robot, q, pl$environment, pl$self, pl$margin)) {
    return(FALSE)
  }
  if (!is.null(pl$validity) && !isTRUE(pl$validity(q))) return(FALSE)
  TRUE
}

edge_valid <- function(pl, a, b) {
  S <- interpolate_segment(a, b, pl$resolution)
  for (i in seq_len(nrow(S))[-1]) if (!state_valid(pl, S[i, ])) return(FALSE)
  TRUE
}

out_of_time <- function(pl) proc.time()[["elapsed"]] > pl$deadline

sample_state <- function(pl) {
  stats::runif(nrow(pl$bounds), pl$bounds[, 1], pl$bounds[, 2])
}

steer <- function(from, to, step) {
  d <- to - from
  n <- vnorm(d)
  if (n <= step) to else from + d * (step / n)
}

nearest_index <- function(nodes, n, q) {
  d <- colSums((t(nodes[seq_len(n), , drop = FALSE]) - q)^2)
  which.min(d)
}

path_from_tree <- function(nodes, parent, i) {
  idx <- integer(0)
  while (i > 0) {
    idx <- c(i, idx)
    i <- parent[i]
  }
  nodes[idx, , drop = FALSE]
}

path_length <- function(P) {
  if (nrow(P) < 2) return(0)
  sum(sqrt(rowSums(diff(P)^2)))
}

grow_rrt <- function(pl, start, goal) {
  d <- length(start)
  N <- pl$max_iterations + 2
  nodes <- matrix(0, N, d)
  parent <- integer(N)
  nodes[1, ] <- start
  n <- 1L
  for (it in seq_len(pl$max_iterations)) {
    if (out_of_time(pl)) break
    target <- if (stats::runif(1) < pl$goal_bias) goal else sample_state(pl)
    i <- nearest_index(nodes, n, target)
    qn <- steer(nodes[i, ], target, pl$step_size)
    if (!state_valid(pl, qn) || !edge_valid(pl, nodes[i, ], qn)) next
    n <- n + 1L
    nodes[n, ] <- qn
    parent[n] <- i
    if (vnorm(qn - goal) <= pl$step_size && edge_valid(pl, qn, goal)) {
      if (vnorm(qn - goal) > 0) {
        n <- n + 1L
        nodes[n, ] <- goal
        parent[n] <- n - 1L
      }
      return(list(path = path_from_tree(nodes, parent, n), iterations = it,
                  nodes = n))
    }
  }
  list(path = NULL, iterations = pl$max_iterations, nodes = n)
}

grow_rrt_connect <- function(pl, start, goal) {
  d <- length(start)
  N <- pl$max_iterations + 2
  mk <- function(root) {
    t <- list(nodes = matrix(0, N, d), parent = integer(N), n = 1L)
    t$nodes[1, ] <- root
    t
  }
  ta <- mk(start)
  tb <- mk(goal)
  a_is_start <- TRUE
  extend <- function(tr, target) {
    if (tr$n == nrow(tr$nodes)) {
      tr$nodes <- rbind(tr$nodes, matrix(0, nrow(tr$nodes), d))
      tr$parent <- c(tr$parent, integer(length(tr$parent)))
    }
    i <- nearest_index(tr$nodes, tr$n, target)
    qn <- steer(tr$nodes[i, ], target, pl$step_size)
    if (!state_valid(pl, qn) || !edge_valid(pl, tr$nodes[i, ], qn)) {
      return(list(tree = tr, status = "trapped"))
    }
    tr$n <- tr$n + 1L
    tr$nodes[tr$n, ] <- qn
    tr$parent[tr$n] <- i
    list(tree = tr, status = if (vnorm(qn - target) < 1e-12) "reached"
                               else "advanced")
  }
  for (it in seq_len(pl$max_iterations)) {
    if (out_of_time(pl)) break
    r <- extend(ta, sample_state(pl))
    ta <- r$tree
    if (r$status != "trapped") {
      qnew <- ta$nodes[ta$n, ]
      repeat {
        r2 <- extend(tb, qnew)
        tb <- r2$tree
        if (r2$status != "advanced" || out_of_time(pl)) break
      }
      if (r2$status == "reached") {
        pa <- path_from_tree(ta$nodes, ta$parent, ta$n)
        pb <- path_from_tree(tb$nodes, tb$parent, tb$n)
        pb <- pb[rev(seq_len(nrow(pb))), , drop = FALSE][-1, , drop = FALSE]
        path <- rbind(pa, pb)
        if (!a_is_start) path <- path[rev(seq_len(nrow(path))), , drop = FALSE]
        return(list(path = path, iterations = it, nodes = ta$n + tb$n))
      }
    }
    tmp <- ta
    ta <- tb
    tb <- tmp
    a_is_start <- !a_is_start
  }
  list(path = NULL, iterations = pl$max_iterations, nodes = ta$n + tb$n)
}

grow_rrt_star <- function(pl, start, goal) {
  d <- length(start)
  N <- pl$max_iterations + 2
  nodes <- matrix(0, N, d)
  parent <- integer(N)
  cost <- numeric(N)
  nodes[1, ] <- start
  n <- 1L
  goal_nodes <- integer(0)
  for (it in seq_len(pl$max_iterations)) {
    if (out_of_time(pl)) break
    target <- if (stats::runif(1) < pl$goal_bias) goal else sample_state(pl)
    i <- nearest_index(nodes, n, target)
    qn <- steer(nodes[i, ], target, pl$step_size)
    if (!state_valid(pl, qn) || !edge_valid(pl, nodes[i, ], qn)) next
    radius <- min(pl$rewire_factor * (log(n + 1) / (n + 1))^(1 / d),
                  pl$step_size)
    dist <- sqrt(colSums((t(nodes[seq_len(n), , drop = FALSE]) - qn)^2))
    near <- which(dist <= radius)
    best <- i
    best_cost <- cost[i] + vnorm(nodes[i, ] - qn)
    for (j in near) {
      cj <- cost[j] + dist[j]
      if (cj < best_cost - 1e-12 && edge_valid(pl, nodes[j, ], qn)) {
        best <- j
        best_cost <- cj
      }
    }
    n <- n + 1L
    nodes[n, ] <- qn
    parent[n] <- best
    cost[n] <- best_cost
    for (j in near) {
      if (j == best) next
      cj <- best_cost + dist[j]
      if (cj < cost[j] - 1e-12 && edge_valid(pl, qn, nodes[j, ])) {
        delta <- cost[j] - cj
        parent[j] <- n
        # Propagate the cost decrease to the subtree of j.
        stack <- j
        while (length(stack)) {
          s <- stack[1]
          stack <- stack[-1]
          cost[s] <- cost[s] - delta
          stack <- c(stack, which(parent[seq_len(n)] == s))
        }
      }
    }
    if (vnorm(qn - goal) <= pl$step_size && edge_valid(pl, qn, goal)) {
      goal_nodes <- c(goal_nodes, n)
    }
  }
  if (!length(goal_nodes)) {
    return(list(path = NULL, iterations = pl$max_iterations, nodes = n))
  }
  total <- cost[goal_nodes] + sqrt(colSums((t(nodes[goal_nodes, , drop = FALSE]) -
                                              goal)^2))
  g <- goal_nodes[which.min(total)]
  path <- path_from_tree(nodes, parent, g)
  if (vnorm(path[nrow(path), ] - goal) > 0) path <- rbind(path, goal)
  list(path = path, iterations = pl$max_iterations, nodes = n)
}

dijkstra <- function(adj, w, from, to) {
  nn <- length(adj)
  dist <- rep(Inf, nn)
  prev <- integer(nn)
  done <- logical(nn)
  dist[from] <- 0
  repeat {
    cand <- which(!done & is.finite(dist))
    if (!length(cand)) break
    u <- cand[which.min(dist[cand])]
    if (u == to) break
    done[u] <- TRUE
    for (k in seq_along(adj[[u]])) {
      v <- adj[[u]][k]
      alt <- dist[u] + w[[u]][k]
      if (alt < dist[v]) {
        dist[v] <- alt
        prev[v] <- u
      }
    }
  }
  if (!is.finite(dist[to])) return(NULL)
  idx <- to
  while (idx[1] != from) idx <- c(prev[idx[1]], idx)
  idx
}

grow_prm <- function(pl, start, goal) {
  d <- length(start)
  samples <- matrix(0, 0, d)
  tries <- 0L
  while (nrow(samples) < pl$prm_nodes && tries < pl$max_iterations &&
         !out_of_time(pl)) {
    tries <- tries + 1L
    q <- sample_state(pl)
    if (state_valid(pl, q)) samples <- rbind(samples, q)
  }
  V <- rbind(start, goal, samples)
  nn <- nrow(V)
  adj <- replicate(nn, integer(0), simplify = FALSE)
  w <- replicate(nn, numeric(0), simplify = FALSE)
  D <- as.matrix(stats::dist(V))
  k <- min(pl$prm_neighbors, nn - 1)
  for (i in seq_len(nn)) {
    if (out_of_time(pl)) break
    nb <- order(D[i, ])[seq_len(k + 1)][-1]
    for (j in nb) {
      if (j %in% adj[[i]]) next
      if (edge_valid(pl, V[i, ], V[j, ])) {
        adj[[i]] <- c(adj[[i]], j)
        w[[i]] <- c(w[[i]], D[i, j])
        adj[[j]] <- c(adj[[j]], i)
        w[[j]] <- c(w[[j]], D[i, j])
      }
    }
  }
  idx <- dijkstra(adj, w, 1L, 2L)
  edges <- sum(lengths(adj)) / 2
  list(path = if (is.null(idx)) NULL else V[idx, , drop = FALSE],
       iterations = tries, nodes = nn, edges = edges,
       roadmap = list(vertices = V, adjacency = adj))
}

#' Plan a path between two configurations
#'
#' Runs the configured planner from `start` to `goal`. Start and goal are
#' validated first. Randomness is reproducible: with `seed` the global
#' random number state is restored afterwards and the seed is recorded in
#' the result. A failure (`success = FALSE`) only means that no path was
#' found within the iteration budget; it does not prove that no path
#' exists. A successful path is collision-free only up to the discrete
#' edge-checking resolution.
#'
#' @param planner A `robot_planner`.
#' @param start,goal Configurations.
#' @param seed Optional integer seed.
#' @param smooth Apply [robot_shortcut()] to a successful path.
#' @return A `robot_plan` with `path` (one waypoint per row, or `NULL`),
#'   `success`, `message`, `algorithm`, `iterations`, `nodes`, `length`,
#'   `seed` and `time` (elapsed seconds).
#' @examples
#' planner <- robot_planner(validity = function(q) abs(q[1] - 0.5) > 0.1 || q[2] > 0.8,
#'                          bounds = rbind(c(0, 1), c(0, 1)))
#' plan <- robot_plan(planner, c(0.1, 0.1), c(0.9, 0.1), seed = 2)
#' plan
#' @export
robot_plan <- function(planner, start, goal, seed = NULL, smooth = FALSE) {
  stopifnot(inherits(planner, "robot_planner"))
  d <- nrow(planner$bounds)
  start <- check_numeric_vector(start, d, "start")
  goal <- check_numeric_vector(goal, d, "goal")
  t0 <- proc.time()[["elapsed"]]
  fail <- function(msg) {
    structure(list(path = NULL, success = FALSE, message = msg,
                   algorithm = planner$algorithm, iterations = 0L, nodes = 0L,
                   length = NA_real_, seed = seed, time = 0,
                   start = start, goal = goal, planner = planner),
              class = "robot_plan")
  }
  if (!state_valid(planner, start)) return(fail("start configuration is invalid"))
  if (!state_valid(planner, goal)) return(fail("goal configuration is invalid"))
  planner$deadline <- t0 + planner$time_limit
  res <- with_seed_internal(seed, switch(planner$algorithm,
    rrt = grow_rrt(planner, start, goal),
    rrt_connect = grow_rrt_connect(planner, start, goal),
    rrt_star = grow_rrt_star(planner, start, goal),
    prm = grow_prm(planner, start, goal)))
  ok <- !is.null(res$path)
  path <- res$path
  if (ok) {
    rownames(path) <- NULL
    if (!is.null(planner$robot)) colnames(path) <- robot_joint_names(planner$robot)
  }
  plan <- structure(list(
    path = path, success = ok,
    message = if (ok) "path found" else if (out_of_time(planner))
      "no path found within the time limit (this does not prove that none exists)" else
      "no path found within the iteration budget (this does not prove that none exists)",
    algorithm = planner$algorithm, iterations = res$iterations,
    nodes = res$nodes, length = if (ok) path_length(path) else NA_real_,
    seed = seed, time = proc.time()[["elapsed"]] - t0, start = start,
    goal = goal, planner = utils::modifyList(planner, list(deadline = Inf)),
    roadmap = res$roadmap
  ), class = "robot_plan")
  if (ok && smooth) plan <- robot_shortcut(plan, seed = seed)
  plan
}

#' @export
print.robot_plan <- function(x, ...) {
  cat(sprintf("<robot_plan> %s: %s\n", x$algorithm,
              if (x$success) "success" else "FAILED"))
  cat(x$message, "\n")
  if (x$success) {
    cat(sprintf("%d waypoints, length %.4g, %d nodes, %d iterations, %.3g s\n",
                nrow(x$path), x$length, x$nodes, x$iterations, x$time))
  }
  if (!is.null(x$seed)) cat("seed:", x$seed, "\n")
  invisible(x)
}

#' Shortcut smoothing of a planned path
#'
#' Repeatedly picks two random waypoints and replaces the path between
#' them with a straight segment when that segment is valid. The path
#' length never increases. Endpoints are preserved.
#'
#' @param plan A successful `robot_plan`.
#' @param iterations Number of shortcut attempts.
#' @param seed Optional seed.
#' @return A `robot_plan` with the shortened path.
#' @examples
#' planner <- robot_planner(validity = function(q) TRUE,
#'                          bounds = rbind(c(0, 1), c(0, 1)), algorithm = "rrt")
#' plan <- robot_plan(planner, c(0, 0), c(1, 1), seed = 3)
#' robot_shortcut(plan, seed = 3)$length <= plan$length
#' @export
robot_shortcut <- function(plan, iterations = 100, seed = NULL) {
  stopifnot(inherits(plan, "robot_plan"))
  if (!plan$success) return(plan)
  pl <- plan$planner
  P <- plan$path
  with_seed_internal(seed, {
    for (it in seq_len(check_count(iterations, "iterations"))) {
      if (nrow(P) < 3) break
      ij <- sort(sample.int(nrow(P), 2))
      if (ij[2] - ij[1] < 2) next
      if (edge_valid(pl, P[ij[1], ], P[ij[2], ])) {
        P <- P[-((ij[1] + 1):(ij[2] - 1)), , drop = FALSE]
      }
    }
  })
  plan$path <- P
  plan$length <- path_length(P)
  plan$message <- paste(plan$message, "(shortcut smoothed)")
  plan
}

#' @export
plot.robot_plan <- function(x, dims = c(1, 2), obstacles = NULL, ...) {
  b <- x$planner$bounds
  graphics::plot(NA, xlim = b[dims[1], ], ylim = b[dims[2], ],
                 xlab = paste0("q", dims[1]), ylab = paste0("q", dims[2]),
                 main = sprintf("%s plan", x$algorithm), asp = 1, ...)
  if (is.function(obstacles)) obstacles()
  if (!is.null(x$roadmap)) {
    V <- x$roadmap$vertices
    for (i in seq_along(x$roadmap$adjacency)) {
      for (j in x$roadmap$adjacency[[i]]) {
        if (j > i) graphics::segments(V[i, dims[1]], V[i, dims[2]],
                                      V[j, dims[1]], V[j, dims[2]],
                                      col = "grey85")
      }
    }
  }
  if (x$success) {
    graphics::lines(x$path[, dims], col = "firebrick", lwd = 2)
    graphics::points(x$path[, dims], pch = 19, cex = 0.6, col = "firebrick")
  }
  graphics::points(x$start[dims[1]], x$start[dims[2]], pch = 15, cex = 1.5,
                   col = "darkgreen")
  graphics::points(x$goal[dims[1]], x$goal[dims[2]], pch = 17, cex = 1.5,
                   col = "navy")
  invisible(x)
}
