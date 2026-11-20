test_that("default UKF cannot produce an indefinite posterior from an SPD prior", {
  P <- matrix(c(2,1,2,1,2,2,2,2,4),3)
  L <- rbind(a=c(5,0))
  f <- robot_ukf(c(0,0,0),P,L,diag(2),diag(.01,2))
  g <- ukf_update(f,landmark_measurements(c(0,0,0),L))
  expect_gte(min(eigen(g$cov,symmetric=TRUE)$values),-1e-10)
})

test_that("UKF identity propagation preserves a local signed-weight Gaussian", {
  f <- robot_ukf(c(0,0,0),diag(c(0,0,3)),rbind(a=c(5,0)),
                 matrix(0,2,2),diag(2),alpha=1e-3)
  g <- ukf_predict(f,c(0,0),0)
  expect_equal(unname(g$mean),unname(f$mean),tolerance=1e-10)
  expect_equal(unname(g$cov),unname(f$cov),tolerance=1e-8)
  expect_gte(min(eigen(g$cov,symmetric=TRUE)$values),-1e-10)
})

test_that("SE3 graph does not falsely converge at the log branch cut", {
  for (axis in 1:3) for (a in c(pi,pi-1e-8,pi-1e-7)) {
    d <- replace(numeric(6),3+axis,a)
    g <- robot_pose_graph(list(diag(4),se3_exp(d)))
    g <- pose_graph_add_edge(g,1,2,diag(4))
    ans <- pose_graph_optimize(g)
    # This consistent single-edge problem has an exact zero-cost solution.
    expect_lt(tail(ans$cost,1),1e-10)
  }
})

test_that("filter covariance contracts reject indefinite inputs", {
  L <- rbind(a=c(1,0))
  expect_error(robot_ekf(c(0,0,0),diag(c(-1,1,1)),L,diag(2),diag(2)))
  expect_error(robot_ekf(c(0,0,0),diag(3),L,diag(c(-1,1)),diag(2)))
  expect_error(robot_ekf(c(0,0,0),diag(3),L,diag(2),diag(c(-1,1))))
})

test_that("mobile simulation rejects unmatched duration vectors", {
  expect_error(mobile_simulate(robot_unicycle(),c(0,0,0),
                                         cbind(rep(1,3),0),dt=c(1,2)))
  expect_error(robot_unicycle(max_speed=-1))
})

test_that("mapping rejects invalid probabilities and ranges", {
  expect_error(robot_occupancy_grid(c(0,2),c(0,2),1,prior=1.2))
  g <- robot_occupancy_grid(c(0,5),c(0,1),1)
  expect_error(grid_update_scan(g,c(2.5,.5,0),0,-1,max_range=5))
})

test_that("UKF statistics stay consistent across the heading cut", {
  L <- rbind(a = c(5, 0), b = c(0, 5))
  min_ev <- function(P) min(eigen(P, symmetric = TRUE, only.values = TRUE)$values)
  f <- robot_ukf(c(0, 0, pi - 0.01), diag(c(0.1, 0.1, 0.5)), L, diag(0.01, 2),
                 diag(c(0.01, 0.001)))
  for (k in 1:5) {
    f <- ukf_predict(f, c(0.5, 0.1), 0.2)
    expect_gte(min_ev(f$cov), 0)
    f <- ukf_update(f, landmark_measurements(c(0.1 * k, 0, pi - 0.01 + 0.02 * k), L))
    expect_gte(min_ev(f$cov), -1e-12)
    expect_true(all(is.finite(f$mean)))
    expect_gte(f$mean[["theta"]], -pi)
    expect_lt(f$mean[["theta"]], pi)
  }
  # A broad, correlated prior and a singular one.
  P <- matrix(c(2, 1, 2, 1, 2, 2, 2, 2, 4), 3)
  g <- robot_ukf(c(1, 1, 3), P, L, diag(2), diag(0.01, 2))
  g <- ukf_update(ukf_predict(g, c(1, 0.5), 0.1),
                  landmark_measurements(c(1, 1, 3), L))
  expect_gte(min_ev(g$cov), -1e-10)
  s <- robot_ukf(c(0, 0, 0), diag(c(0, 0, 0.2)), L, matrix(0, 2, 2), diag(0.01, 2))
  s <- ukf_update(s, landmark_measurements(c(0, 0, 0.1), L))
  expect_gte(min_ev(s$cov), -1e-12)
})

test_that("UKF identity propagation holds for any accepted scaling", {
  P <- matrix(c(0.3, 0.1, 0.05, 0.1, 0.2, 0, 0.05, 0, 2.5), 3)
  for (a in c(1e-3, 0.1, 0.5, 1)) {
    f <- robot_ukf(c(1, -2, 3), P, rbind(a = c(5, 0)), matrix(0, 2, 2), diag(2),
                   alpha = a)
    g <- ukf_predict(f, c(0, 0), 0)
    expect_equal(unname(g$mean), c(1, -2, 3), tolerance = 1e-10)
    expect_equal(unname(g$cov), P, tolerance = 1e-8)
  }
  expect_error(robot_ukf(c(0, 0, 0), diag(3), rbind(a = c(5, 0)), diag(2), diag(2),
                         kappa = -3), "kappa")
})

test_that("covariance inputs must be symmetric and semidefinite", {
  L <- rbind(a = c(1, 0))
  expect_error(robot_ekf(c(0, 0, 0), diag(3), L, matrix(c(1, 0, 1, 1), 2), diag(2)),
               "symmetric")
  expect_error(robot_ukf(c(0, 0, 0), diag(c(1, 1, -1e-3)), L, diag(2), diag(2)),
               "semidefinite")
  expect_error(robot_ekf_slam(c(0, 0, 0), diag(3), diag(2), diag(c(1, -1))),
               "semidefinite")
  expect_error(robot_particle_filter(10, c(0, 0, 0), diag(c(1, 1, -1)), L,
                                     diag(2), diag(2)), "semidefinite")
  expect_silent(robot_ekf(c(0, 0, 0), matrix(0, 3, 3), L, matrix(0, 2, 2), diag(2)))
})

test_that("SE(3) pose graphs converge across the branch cut", {
  set.seed(8)
  for (k in 1:6) {
    u <- rnorm(3)
    u <- u / sqrt(sum(u^2))
    a <- pi - c(0, 1e-9, 1e-7, 1e-6, 1e-3, 0)[k]
    Z <- se3_exp(c(rnorm(3), 0.3 * u))
    X2 <- Z %*% se3_exp(c(rnorm(3), a * u))
    g <- robot_pose_graph(list(diag(4), X2))
    g <- pose_graph_add_edge(g, 1, 2, Z)
    ans <- pose_graph_optimize(g)
    expect_identical(ans$status, "converged")
    expect_lt(tail(ans$cost, 1), 1e-10)
    expect_equal(as_plain_matrix(ans$graph$nodes[[2]]), Z, tolerance = 1e-8)
  }
})

test_that("pose graph outcomes are reported, not assumed", {
  g <- robot_pose_graph(rbind(c(0, 0, 0), c(1, 0, 0), c(2, 0, 0)))
  g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0))
  expect_warning(res <- pose_graph_optimize(g), "not connected")
  expect_identical(res$status, "invalid_graph")
  expect_false(res$converged)
  # A noisy loop converges to a positive cost.
  h <- robot_pose_graph(rbind(c(0, 0, 0), c(1, 0, 0), c(1, 1, 0)))
  h <- pose_graph_add_edge(h, 1, 2, c(1, 0, pi / 2))
  h <- pose_graph_add_edge(h, 2, 3, c(1.1, 0, pi / 2))
  h <- pose_graph_add_edge(h, 3, 1, c(1.4, 0.05, 3 * pi / 4 + 0.1))
  r <- pose_graph_optimize(h)
  expect_identical(r$status, "converged")
  expect_gt(tail(r$cost, 1), 1e-6)
  expect_lte(r$gradient, 1e-6)
  r1 <- pose_graph_optimize(h, max_iterations = 1)
  expect_identical(r1$status, "max_iterations")
  expect_false(r1$converged)
})

test_that("mobile limits and controls are validated before use", {
  for (bad in list(NA_real_, c(1, 2), "1", -0.1)) {
    expect_error(robot_unicycle(max_speed = bad), "nonnegative")
    expect_error(robot_diffdrive(max_turn_rate = bad), "nonnegative")
  }
  expect_error(robot_bicycle(max_steer = pi / 2), "pi / 2")
  sim <- mobile_simulate(robot_unicycle(max_speed = 1), c(0, 0, 0),
                         rbind(c(2, 0), c(-3, 0)), dt = 1, saturate = TRUE)
  expect_equal(unname(sim[, "x"]), c(0, 1, 0))
  expect_error(mobile_simulate(robot_unicycle(), c(0, 0, 0), cbind(1, 0), dt = -1),
               "nonnegative")
  expect_error(mobile_simulate(robot_unicycle(), c(0, 0, 0), cbind(1, 0), 1,
                               saturate = NA), "saturate")
  dd <- robot_diffdrive()
  expect_error(diffdrive_forward(dd, c(1, 2), 1))
  expect_error(diffdrive_inverse(dd, 1, c(1, 2)))
  expect_equal(nrow(mobile_simulate(robot_unicycle(), c(0, 0, 0),
                                    matrix(numeric(0), 0, 2), dt = 1)), 1)
})

test_that("map inputs are validated before the map changes", {
  expect_error(robot_occupancy_grid(c(0, 1), c(0, 1), 0.5, prior = 0), "probability")
  expect_error(robot_occupancy_grid(c(0, 1), c(0, 1), 0.5,
                                    occupied = matrix(1, 2, 2)), "logical")
  g <- robot_occupancy_grid(c(0, 5), c(0, 1), 1)
  expect_error(grid_update_scan(g, c(2.5, 0.5, 0), 0, 1, max_range = -1), "max_range")
  expect_error(grid_update_scan(g, c(2.5, 0.5, 0), 0, 1, 5, clamp = c(5, -5)), "clamp")
  expect_error(grid_inflate(g, -1), "radius")
  expect_error(grid_occupied(g, 2), "probability")
  # An empty scan and a valid hit still behave.
  expect_identical(grid_update_scan(g, c(2.5, 0.5, 0), numeric(0), numeric(0), 5)$logodds,
                   g$logodds)
  h <- grid_update_scan(g, c(0.5, 0.5, 0), 0, 2, 5)
  expect_equal(h$logodds[1, ], c(-0.4, -0.4, 0.85, 0, 0))
})

test_that("measurement tables are validated by every filter", {
  L <- rbind(a = c(1, 0))
  bad <- list(data.frame(id = "a", range = Inf, bearing = 0),
              data.frame(id = "a", range = NA_real_, bearing = 0),
              data.frame(id = "a", bearing = 0))
  f <- robot_ekf(c(0, 0, 0), diag(3), L, diag(2), diag(2))
  u <- robot_ukf(c(0, 0, 0), diag(3), L, diag(2), diag(2))
  s <- robot_ekf_slam(c(0, 0, 0), diag(3), diag(2), diag(2))
  p <- robot_particle_filter(5, c(0, 0, 0), diag(3), L, diag(2), diag(2), seed = 1)
  for (m in bad) {
    expect_error(ekf_update(f, m), "easurements")
    expect_error(ukf_update(u, m), "easurements")
    expect_error(ekf_slam_update(s, m), "easurements")
    expect_error(pf_update(p, m), "easurements")
  }
})

test_that("noise-free graphs at large scale converge, not stall", {
  set.seed(4)
  truth <- list(diag(4))
  for (k in 2:6) truth[[k]] <- truth[[k - 1]] %*% se3_exp(c(rnorm(3, 0, 100), rnorm(3)))
  init <- lapply(1:6, function(k) {
    if (k == 1) truth[[1]] else truth[[k]] %*% se3_exp(rnorm(6, 0, 0.05))
  })
  g <- robot_pose_graph(init)
  rel <- function(a, b) se3_inverse_matrix(truth[[a]]) %*% truth[[b]]
  for (k in 1:5) g <- pose_graph_add_edge(g, k, k + 1, rel(k, k + 1), diag(1e6, 6))
  g <- pose_graph_add_edge(g, 1, 6, rel(1, 6), diag(1e6, 6))
  r <- pose_graph_optimize(g)
  expect_identical(r$status, "converged")
  expect_true(r$converged)
})

test_that("noise models are consistent between simulation and inference", {
  L <- rbind(a = c(0.01, 0))
  m <- landmark_measurements(c(0, 0, 0), L, diag(c(1, 0.01)), seed = 4)
  expect_lt(m$range, 0)
  f <- robot_ekf(c(0, 0, 0), diag(0.1, 3), L, diag(0, 2), diag(c(1, 0.01)))
  expect_true(all(is.finite(ekf_update(f, m)$mean)))
  expect_error(robot_ekf(numeric(3), diag(0, 3), L, diag(0, 2), diag(0, 2)),
               "positive definite")
  expect_error(robot_particle_filter(5, numeric(3), diag(3), L, diag(2),
                                     diag(c(1, 0))), "positive definite")
  expect_error(robot_ekf(numeric(3), diag(c(1e16, -1, 1)), L, diag(2), diag(2)),
               "semidefinite")
  expect_error(robot_ekf(numeric(3), diag(3), rbind(A = c(1, 0), A = c(2, 0)),
                         diag(2), diag(2)), "unique")
  expect_error(landmark_measurements(c(0, 0, 0), L, max_range = NA), "max_range")
  expect_error(ekf_slam_predict(robot_ekf_slam(c(0, 0, 0), diag(1e-4, 3), diag(2),
                                               diag(2)), c(1, 0), c(1, 2)), "dt")
  p <- robot_particle_filter(5, c(0, 0, 0), diag(3), L, diag(2), diag(2), seed = 1)
  expect_error(pf_update(p, landmark_measurements(c(0, 0, 0), L),
                         resample_threshold = NA), "resample_threshold")
})

test_that("a landmark on the estimate carries no heading information", {
  f <- robot_ukf(numeric(3), diag(c(0, 0, 1)), rbind(A = c(0, 0)), diag(2),
                 0.1 * diag(2))
  expect_warning(u <- ukf_update(f, data.frame(id = "A", range = 0, bearing = 1)),
                 "coincides")
  expect_equal(unname(u$mean[3]), 0)
  expect_equal(u$cov[3, 3], 1)
})

test_that("UKF spread survives a tiny alpha and SLAM stays semidefinite", {
  f <- robot_ukf(c(0, 0, 0), diag(3), rbind(a = c(1, 0)), diag(0, 2), diag(2),
                 alpha = 1e-9)
  expect_equal(unname(ukf_predict(f, c(0, 0), 0)$cov), diag(3), tolerance = 1e-10)
  z <- data.frame(id = "L", range = 1, bearing = 0)
  s <- robot_ekf_slam(c(0, 0, 0), diag(0, 3), diag(2), diag(1e-20, 2))
  s <- ekf_slam_predict(ekf_slam_update(s, z), c(0, 0), 1)
  P <- ekf_slam_update(s, z)$cov
  expect_gte(min(eigen(P, symmetric = TRUE, only.values = TRUE)$values), -1e-30)
})

test_that("a world offset never makes an unsolved graph look converged", {
  solve_y <- function(off) {
    g <- robot_pose_graph(rbind(c(off, 0, 0), c(off, 1e-4, 0)))
    g <- pose_graph_add_edge(g, 1, 2, c(0, 0, 0), information = diag(c(1, 1e12, 1)))
    pose_graph_optimize(g)
  }
  for (off in c(0, 1e9, 1e12)) {
    r <- solve_y(off)
    expect_true(r$converged)
    expect_lt(abs(r$graph$nodes[2, 2]), 1e-10)
    expect_lt(tail(r$cost, 1), 1e-8)
  }
  # An inconsistent heavy x subproblem keeps a positive cost but the
  # independent y direction is still solved.
  for (off in c(0, 1e9)) {
    g <- robot_pose_graph(rbind(c(off, 0, 0), c(off + 0.3, 1e-3, 0)))
    info <- diag(c(1e12, 1e12, 1))
    g <- pose_graph_add_edge(g, 1, 2, c(0.5, 0, 0), information = info)
    g <- pose_graph_add_edge(g, 1, 2, c(-0.5, 0, 0), information = info)
    r <- pose_graph_optimize(g)
    expect_true(r$converged)
    expect_lt(abs(r$graph$nodes[2, 2]), 1e-9)
    expect_lt(abs(r$graph$nodes[2, 1] - off), 1e-6)
    expect_equal(tail(r$cost, 1), 5e11, tolerance = 1e-6)
  }
})
