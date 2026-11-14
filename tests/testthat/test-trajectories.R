test_that("quintic and cubic polynomials meet their boundary conditions", {
  tr <- robot_traj_polynomial(c(0, 1), c(1, -1), duration = 2, qd0 = c(0.5, 0),
                              qd1 = c(-0.2, 0.3), qdd0 = c(1, 0), qdd1 = c(0.2, -1))
  e <- tr$evaluate(c(0, 2))
  expect_equal(e$q, rbind(c(0, 1), c(1, -1)), tolerance = 1e-12, ignore_attr = TRUE)
  expect_equal(e$qd, rbind(c(0.5, 0), c(-0.2, 0.3)), tolerance = 1e-12, ignore_attr = TRUE)
  expect_equal(e$qdd, rbind(c(1, 0), c(0.2, -1)), tolerance = 1e-12, ignore_attr = TRUE)
  # Velocities and accelerations are derivatives.
  h <- 1e-6
  t <- 0.7
  fd <- (tr$evaluate(t + h)$q - tr$evaluate(t - h)$q) / (2 * h)
  expect_equal(fd, tr$evaluate(t)$qd, tolerance = 1e-8)
  fd2 <- (tr$evaluate(t + h)$qd - tr$evaluate(t - h)$qd) / (2 * h)
  expect_equal(fd2, tr$evaluate(t)$qdd, tolerance = 1e-7)
  cu <- robot_traj_polynomial(0, 1, 1, qd1 = 1, order = "cubic")
  ec <- cu$evaluate(c(0, 1))
  expect_equal(as.numeric(ec$q), c(0, 1))
  expect_equal(as.numeric(ec$qd), c(0, 1))
  # Rest to rest quintic: s = 10t^3 - 15t^4 + 6t^5.
  s <- robot_traj_polynomial(0, 1, 1)$evaluate(0.3)$q
  expect_equal(as.numeric(s), 10 * 0.3^3 - 15 * 0.3^4 + 6 * 0.3^5, tolerance = 1e-14)
  ex <- tr$evaluate(3)
  expect_equal(as.numeric(ex$q), c(1, -1))
  expect_equal(as.numeric(ex$qd), c(0, 0))
})

test_that("trapezoidal profiles respect limits and synchronize joints", {
  tr <- robot_traj_trapezoidal(c(0, 0), c(1, 2), vmax = 1, amax = 2, n = 2001)
  # Joint 2 limits: cruise at 1, accelerate 0.5 s; duration 2/1 + 0.5 = 2.5.
  expect_equal(tr$duration, 2.5, tolerance = 1e-12)
  expect_lte(max(abs(tr$qd)), 1 + 1e-12)
  expect_lte(max(abs(tr$qdd)), 2 + 1e-12)
  expect_equal(unname(tr$q[nrow(tr$q), ]), c(1, 2), tolerance = 1e-12)
  # Both joints share the time scaling, so the path is a straight line.
  expect_equal(tr$q[, 2], 2 * tr$q[, 1], tolerance = 1e-12)
  tri <- robot_traj_trapezoidal(0, 0.1, vmax = 1, amax = 1)
  expect_equal(tri$duration, 2 * sqrt(0.1), tolerance = 1e-12)
  expect_lt(max(abs(tri$qd)), 1)
  # Positions integrate velocities.
  dt <- diff(tr$time)
  integ <- cumsum(c(0, (tr$qd[-1, 1] + tr$qd[-nrow(tr$qd), 1]) / 2 * dt))
  expect_equal(integ, unname(tr$q[, 1]), tolerance = 1e-5)
})

test_that("waypoint paths stop at each waypoint", {
  path <- rbind(c(0, 0), c(1, 0), c(1, 1))
  tr <- robot_traj_path(path, vmax = 1, amax = 2, dt = 0.01)
  expect_equal(tr$duration, 3, tolerance = 1e-12)
  expect_equal(unname(tr$q[nrow(tr$q), ]), c(1, 1), tolerance = 1e-12)
  expect_equal(unname(tr$qd[c(1, nrow(tr$qd)), ]), matrix(0, 2, 2), tolerance = 1e-12)
  robot <- robot_example("planar2")
  expect_true(robot_traj_check(robot, tr)$ok)
  bad <- robot_traj_polynomial(c(0, 0), c(4, 0), 1)
  chk <- robot_traj_check(robot, bad)
  expect_false(chk$ok)
})

test_that("Cartesian trajectories interpolate position and orientation", {
  a <- robot_pose(rotz(0), c(0, 0, 0))
  b <- robot_pose(rotz(1), c(1, 2, 0))
  ct <- robot_traj_cartesian(a, b, duration = 1, n = 11)
  expect_equal(pose_translation(ct$poses[[6]]), c(0.5, 1, 0), tolerance = 1e-12)
  expect_equal(as.matrix(ct$poses[[11]]), as.matrix(b), tolerance = 1e-12)
  pdf(NULL)
  on.exit(grDevices::dev.off())
  expect_silent(plot(robot_traj_trapezoidal(c(0, 0), c(1, 1), 1, 1)))
})
