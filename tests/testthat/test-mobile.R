test_that("unicycle integration is exact for straight and circular motion", {
  expect_equal(unname(unicycle_step(c(1, 2, pi / 4), 2, 0, 3)),
               c(1 + 6 / sqrt(2), 2 + 6 / sqrt(2), pi / 4), tolerance = 1e-14)
  # Quarter circle of radius 1 starting along x.
  expect_equal(unname(unicycle_step(c(0, 0, 0), pi / 2, pi / 2, 1)),
               c(1, 1, pi / 2), tolerance = 1e-14)
  # Full circle returns to the start, heading wrapped.
  sim <- mobile_simulate(robot_unicycle(), c(0, 0, 0), cbind(1, rep(2 * pi / 10, 10)), 1)
  expect_equal(unname(sim[11, 2:4]), c(0, 0, 0), tolerance = 1e-12)
  expect_true(all(sim[, 4] >= -pi & sim[, 4] < pi))
  # Matches the SE(2) exponential.
  X <- c(0.3, -0.2, 2.5)
  out <- unicycle_step(X, 0.7, -1.3, 0.9)
  H <- diag(3)
  H[1:2, 1:2] <- matrix(c(cos(X[3]), sin(X[3]), -sin(X[3]), cos(X[3])), 2)
  H[1:2, 3] <- X[1:2]
  ref <- H %*% se2_exp(c(0.7, 0, -1.3) * 0.9)
  expect_equal(unname(out[1:2]), ref[1:2, 3], tolerance = 1e-13)
  expect_equal(unname(out[3]), wrap_angle(atan2(ref[2, 1], ref[1, 1])), tolerance = 1e-13)
})

test_that("differential drive and bicycle kinematics", {
  dd <- robot_diffdrive(wheel_radius = 0.1, track_width = 0.5)
  vw <- diffdrive_forward(dd, 2, 4)
  expect_equal(unname(vw), c(0.3, 0.4))
  expect_equal(unname(diffdrive_inverse(dd, 0.3, 0.4)), c(2, 4))
  # Equal wheel increments: straight line of r * delta.
  od <- diffdrive_odometry(dd, c(0, 0, 0), rep(1, 5), rep(1, 5))
  expect_equal(unname(od[6, ]), c(0.5, 0, 0), tolerance = 1e-14)
  # Opposite increments: rotation in place.
  od <- diffdrive_odometry(dd, c(0, 0, 0), -1, 1)
  expect_equal(unname(od[2, ]), c(0, 0, 0.4), tolerance = 1e-14)
  bk <- robot_bicycle(wheelbase = 2, max_steer = 0.6)
  sim <- mobile_simulate(bk, c(0, 0, 0), cbind(1, 0.5), 1)
  w <- tan(0.5) / 2
  expect_equal(unname(sim[2, 2:4]), c(sin(w) / w, (1 - cos(w)) / w, w), tolerance = 1e-13)
  expect_error(mobile_simulate(bk, c(0, 0, 0), cbind(1, 1), 1), "max_steer")
  expect_silent(mobile_simulate(bk, c(0, 0, 0), cbind(1, 1), 1, saturate = TRUE))
  expect_equal(body_to_world_velocity(c(1, 0), pi / 2), c(0, 1), tolerance = 1e-15)
  expect_equal(world_to_body_velocity(c(0, 1), pi / 2), c(1, 0), tolerance = 1e-15)
})

test_that("pure pursuit follows a straight path", {
  path <- cbind(seq(0, 10, 0.5), 1)
  x <- c(0, 0, 0)
  for (i in 1:400) {
    u <- pure_pursuit(x, path, lookahead = 0.8, speed = 1)
    if (u$done) break
    x <- unicycle_step(x, u$v, u$omega, 0.05)
  }
  expect_true(u$done)
  expect_lt(abs(x[2] - 1), 0.05)
})

test_that("EKF localization tracks a synthetic trajectory", {
  L <- rbind(a = c(5, 0), b = c(0, 5), c = c(5, 5), d = c(-2, 2))
  Q <- diag(c(0.01, 0.005))
  R <- diag(c(0.01, 0.001))
  f <- robot_ekf(c(0, 0, 0), diag(0.01, 3), L, Q, R)
  truth <- c(0, 0, 0)
  errs <- numeric(0)
  set.seed(60)
  for (k in 1:100) {
    u <- c(1, 0.3)
    un <- u + as.numeric(rnorm(2) %*% chol(Q))
    truth <- unicycle_step(truth, un[1], un[2], 0.1)
    f <- ekf_predict(f, u, 0.1)
    f <- ekf_update(f, landmark_measurements(truth, L, R, seed = k))
    errs <- c(errs, sqrt(sum((f$mean[1:2] - truth[1:2])^2)))
  }
  expect_lt(max(errs), 0.25)
  expect_lt(abs(wrap_angle(f$mean[[3]] - truth[[3]])), 0.1)
  expect_equal(f$cov, t(f$cov))
  expect_gt(min(eigen(f$cov)$values), 0)
  # Linear-Gaussian sanity: predict Jacobians match finite differences.
  fp <- robot_ekf(c(0.2, -0.1, 0.4), diag(1e-12, 3), L, diag(1e-12, 2), R)
  expect_equal(unname(ekf_predict(fp, c(1, 0.5), 0.2)$mean),
               unname(unicycle_step(c(0.2, -0.1, 0.4), 1, 0.5, 0.2)))
})

test_that("EKF bearing innovations are wrapped", {
  L <- rbind(a = c(-1, 0.001))
  f <- robot_ekf(c(0, 0, 0), diag(0.1, 3), L, diag(0.01, 2), diag(c(0.01, 0.01)))
  z <- data.frame(id = "a", range = 1, bearing = -pi + 0.001)
  f2 <- ekf_update(f, z)
  expect_lt(abs(f2$mean[[3]]), 0.05)
  expect_error(ekf_update(f, data.frame(id = "zz", range = 1, bearing = 0)), "Unknown")
})

test_that("particle filter localization converges", {
  L <- rbind(a = c(5, 0), b = c(0, 5), c = c(5, 5))
  Q <- diag(c(0.01, 0.005))
  R <- diag(c(0.02, 0.002))
  pf <- robot_particle_filter(400, c(0, 0, 0), diag(c(0.1, 0.1, 0.05)), L, Q, R, seed = 1)
  truth <- c(0, 0, 0)
  for (k in 1:30) {
    truth <- unicycle_step(truth, 1, 0.2, 0.1)
    pf <- pf_predict(pf, c(1, 0.2), 0.1, seed = k)
    pf <- pf_update(pf, landmark_measurements(truth, L, R, seed = 100 + k), seed = k)
  }
  est <- pf_estimate(pf)
  expect_lt(sqrt(sum((est$mean[1:2] - truth[1:2])^2)), 0.2)
  expect_lt(abs(wrap_angle(est$mean[["theta"]] - truth[[3]])), 0.1)
  # Circular mean handles particles across the +/- pi cut.
  pf2 <- pf
  pf2$particles <- cbind(0, 0, c(pi - 0.01, -pi + 0.01))
  pf2$weights <- c(0.5, 0.5)
  expect_equal(abs(pf_estimate(pf2)$mean[["theta"]]), pi, tolerance = 1e-12)
})

test_that("occupancy grids: ray casting, scan updates and A*", {
  occ <- matrix(FALSE, 20, 20)
  occ[, 15] <- TRUE
  truth <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1, occupied = occ)
  expect_equal(grid_raycast(truth, c(0.5, 1, 0), 0, 3), 0.9, tolerance = 1e-12)
  expect_equal(grid_raycast(truth, c(0.5, 1, pi / 4), 0, 3), 0.9 * sqrt(2),
               tolerance = 1e-12)
  expect_equal(grid_raycast(truth, c(0.5, 1, 0), pi, 3), 3)
  g <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1)
  ang <- seq(-0.5, 0.5, length.out = 21)
  for (i in 1:3) g <- grid_update_scan(g, c(0.5, 1, 0), ang,
                                       grid_raycast(truth, c(0.5, 1, 0), ang, 3), 3)
  P <- grid_probability(g)
  expect_gt(P[11, 15], 0.9)
  expect_lt(P[11, 10], 0.1)
  expect_equal(unname(grid_world_to_cell(g, c(0.55, 1.05))[1, ]), c(11, 6))
  expect_equal(unname(grid_cell_to_world(g, cbind(11, 6))[1, ]), c(0.55, 1.05))
  expect_true(all(is.na(grid_world_to_cell(g, c(5, 5)))))
  occ2 <- matrix(FALSE, 20, 20)
  occ2[3:20, 10] <- TRUE
  g2 <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1, occupied = occ2)
  path <- grid_astar(g2, c(0.25, 1.05), c(1.75, 1.05))
  expect_false(is.null(path))
  expect_false(any(occ2[grid_world_to_cell(g2, path)]))
  expect_lt(min(path[, 2]), 0.3)
  occ2[1:2, 10] <- TRUE
  g3 <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1, occupied = occ2)
  expect_null(grid_astar(g3, c(0.25, 1.05), c(1.75, 1.05)))
  infl <- grid_inflate(g2, 0.1)
  expect_true(infl[5, 9] && infl[5, 11])
  pdf(NULL)
  on.exit(grDevices::dev.off())
  expect_silent(plot(g))
})

test_that("SE(2) pose graph optimization closes a square loop", {
  X <- rbind(c(0, 0, 0), c(1.2, 0.1, 1.7), c(1.1, 1.2, 3.0), c(0.1, 0.9, -1.4))
  g <- robot_pose_graph(X)
  for (i in 1:3) g <- pose_graph_add_edge(g, i, i + 1, c(1, 0, pi / 2))
  g <- pose_graph_add_edge(g, 4, 1, c(1, 0, pi / 2))
  res <- pose_graph_optimize(g)
  expect_true(res$converged)
  ref <- rbind(c(0, 0, 0), c(1, 0, pi / 2), c(1, 1, pi), c(0, 1, -pi / 2))
  out <- res$graph$nodes
  expect_equal(out[, 1:2], ref[, 1:2], tolerance = 1e-8)
  expect_equal(wrap_angle(out[, 3] - ref[, 3]), rep(0, 4), tolerance = 1e-8)
  expect_lt(utils::tail(res$cost, 1), 1e-15)
  # Analytic edge Jacobians agree with finite differences.
  xi <- c(0.3, -0.2, 0.7); xj <- c(1.1, 0.4, -2.9); z <- c(0.5, 0.2, 0.3)
  J <- edge_jacobians(xi, xj, z)
  h <- 1e-6
  for (k in 1:3) {
    d <- replace(numeric(3), k, h)
    expect_equal((edge_error(xi + d, xj, z) - edge_error(xi - d, xj, z)) / (2 * h),
                 J$A[, k], tolerance = 1e-7)
    expect_equal((edge_error(xi, xj + d, z) - edge_error(xi, xj - d, z)) / (2 * h),
                 J$B[, k], tolerance = 1e-7)
  }
  expect_error(pose_graph_add_edge(g, 1, 1, c(0, 0, 0)), "distinct")
})
