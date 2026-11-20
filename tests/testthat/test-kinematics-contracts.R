test_that("masked orientation IK follows the true residual derivative", {
  r <- robot_model(joints = robot_joint("j", "continuous", "a", "b", axis = c(1, 0, 0)))
  R <- roty(pi / 2) %*% rotx(pi / 2)
  s <- robot_ik(r, robot_pose(R), initial = 0, weights = c(0, 0, 0, 0, 0, 1),
                restarts = 0)
  expect_true(s$converged)
})

test_that("Cartesian joint paths are continuous and consistent", {
  r <- robot_model(joints = robot_joint("spin", "continuous", "base", "tip"))
  t <- robot_traj_cartesian(trotz(pi - 0.2), trotz(pi + 0.2), 1, n = 5, robot = r,
                            initial = pi - 0.2)
  expect_lt(max(abs(diff(drop(t$joint$q)))), 0.2)
  expect_equal(unname(t$joint$q[5, 1]), pi + 0.2, tolerance = 1e-8)
  arm <- robot_example("planar3")
  q0 <- c(0.3, -0.8, 1)
  a <- robot_fk(arm, q0)
  b <- robot_pose(translation = pose_translation(a) + c(-0.15, 0.12, 0))
  # Redundant arm: the task part of qd and qdd is exact and the whole
  # derivative matches the returned positions differentiated independently.
  u <- robot_traj_cartesian(a, b, 1, n = 201, robot = arm, initial = q0,
                            weights = c(1, 1, 0, 0, 0, 0), restarts = 0)
  expect_true(all(u$derivatives_ok))
  tt <- u$time
  for (i in c(50, 101, 150)) {
    J <- robot_jacobian(arm, u$joint$q[i, ])[1:2, ]
    expect_equal(unname(drop(J %*% u$joint$qd[i, ])), unname(u$velocity[i, 1:2]), tolerance = 1e-8)
    h <- tt[i + 1] - tt[i]
    fd1 <- (u$joint$q[i + 1, ] - u$joint$q[i - 1, ]) / (2 * h)
    fd2 <- (u$joint$q[i + 1, ] - 2 * u$joint$q[i, ] + u$joint$q[i - 1, ]) / h^2
    expect_lt(max(abs(u$joint$qd[i, ] - fd1)), 1e-3)
    expect_lt(max(abs(u$joint$qdd[i, ] - fd2)), 1e-2)
  }
  fixed <- robot_model(joints = robot_joint("w", "fixed", "a", "b"))
  expect_identical(dim(robot_traj_cartesian(robot_pose(), robot_pose(), 1, 3,
                                            robot = fixed)$joint$q), c(3L, 0L))
})

test_that("trajectory, simulation and dynamics inputs are checked", {
  r <- robot_example("planar2")
  t <- robot_traj_polynomial(c(0, 0), c(100, 0), 1, n = 3)
  expect_error(robot_traj_check(r, t, tol = NA_real_), "tol")
  expect_error(robot_traj_check(r, t, tol = Inf), "tol")
  expect_error(robot_traj_trapezoidal(0, 0, 1, 1, times = c(2, 1)), "nondecreasing")
  expect_error(robot_traj_polynomial(0, 1, 1, times = numeric()), "empty")
  s <- robot_model(list(robot_link("tip", mass = 1)),
                   robot_joint("slide", "prismatic", "base", "tip"))
  expect_error(suppressWarnings(robot_simulate(s, 0, 1, c(0, 0.5, 2, 3),
                                               torque = function(t, q, qd) qd^2,
                                               gravity = c(0, 0, 0))), "stopped")
  m <- robot_model(list(robot_link("mid", 1, inertia = diag(1e-13, 3)),
                        robot_link("tip", 1, inertia = diag(3))),
                   list(robot_joint("a", "revolute", "base", "mid"),
                        robot_joint("b", "revolute", "mid", "tip")))
  expect_silent(robot_forward_dynamics(m, c(0, 0), tau = c(1, 0)))
  expect_error(robot_forward_dynamics(robot_example("puma560"), rep(0.1, 6),
                                      rep(0, 6), rep(0, 6)), "no inertia")
  p <- robot_model(links = robot_link("b", 1),
                   joints = robot_joint("j", "prismatic", "a", "b", axis = c(1, 0, 0)))
  w <- c(1, 0, 0, 0, 0, 0)
  expect_equal(unname(robot_inverse_dynamics(p, 0, gravity = numeric(3),
                                             f_ext = list(b = w, b = 2 * w))), -3)
})

test_that("geometric helpers are scale invariant and keep their intervals", {
  expect_equal(nrow(ik_planar2(1 + 1e-5, 0, 1, 1e-5)), 1)
  expect_equal(as_rotvec(robot_rotation(axis = c(1e200, 0, 0), angle = 1)), c(1, 0, 0))
  expect_equal(as.numeric(robot_quaternion(c(1e200, 0, 0, 0), canonical = FALSE)),
               c(1, 0, 0, 0))
  x <- pi - 2 * .Machine$double.eps
  expect_identical(wrap_angle(x), x)
  w <- wrap_angle(c(1e16, 1e18, -pi, pi))
  expect_true(all(w >= -pi & w < pi))
  expect_equal(w[2], atan2(sin(1e18), cos(1e18)))
  expect_identical(so2_log(so2_exp(-pi)), pi)
  r <- robot_model(joints = robot_joint("j", "continuous", "base", "tip"))
  expect_error(robot_jacobian_analytic(r, pi, representation = "rotvec"), "pi")
  for (th in c(1.1e-5, 2e-5, 1e-4)) {
    xi <- c(0.3, -0.2, 0.5, th * c(1, 2, 2) / 3)
    expect_lt(max(abs(se3_log(se3_exp(xi)) - xi)), 1e-14)
  }
})

test_that("limits, frames, adjoints and inertias are validated", {
  expect_error(robot_joint("j", "prismatic", "a", "b", limits = c(Inf, Inf)), "finite")
  expect_equal(unname(robot_limits(robot_dh(1, 0, 0, 0, limits = rbind(c(0, Inf))))[1, ]),
               c(0, Inf))
  a <- robot_pose(parent = "world")
  b <- robot_pose(translation = c(1, 0, 0), parent = "camera")
  expect_error(robot_pose_interp(a, b, 1), "frames")
  expect_error(se3_adjoint(2 * diag(4)))
  expect_error(se2_adjoint(2 * diag(3)))
  expect_error(inertia_rod(-1, 1), "mass")
  expect_error(inertia_sphere(c(1, 2), 1), "mass")
  arm <- robot_example("planar2")
  expect_error(robot_manipulability(arm, c(0.3, 1), rows = 7), "rows")
  expect_equal(robot_fk(arm, c(joint2 = 0.5, joint1 = 0)), robot_fk(arm, c(0, 0.5)))
  expect_error(robot_fk(arm, c(joint2 = 0.5, other = 0)), "joint names")
})

test_that("roundoff-sized inertia violations are stored as physical tensors", {
  l <- robot_link("tip", 1, inertia = diag(c(-5e-10, 1, 1)))
  expect_gte(min(eigen(l$inertia, symmetric = TRUE, only.values = TRUE)$values), 0)
  r <- robot_model(l, robot_joint("j", "continuous", "base", "tip", axis = c(1, 0, 0)))
  expect_gte(robot_mass_matrix(r, 0)[1, 1], 0)
  expect_gte(unname(robot_energy(r, 0, 1)["kinetic"]), 0)
  expect_error(robot_link("tip", 1, inertia = diag(c(-1e-3, 1, 1))), "semidefinite")
  tri <- robot_link("t", 1, inertia = diag(c(2 + 1e-10, 1, 1)))
  ev <- sort(eigen(tri$inertia, only.values = TRUE)$values)
  expect_lte(ev[3], ev[1] + ev[2])
  expect_equal(robot_link("ok", 1, inertia = diag(c(1, 2, 3)))$inertia, diag(c(1, 2, 3)))
})

test_that("Cartesian derivatives describe a path held at a joint limit", {
  r <- robot_model(joints = list(
    robot_joint("a", "prismatic", "base", "mid", axis = c(1, 0, 0), limits = c(0, 1)),
    robot_joint("b", "prismatic", "mid", "tip", axis = c(1, 0, 0), limits = c(-2, 2))))
  z <- robot_traj_cartesian(robot_pose(), transl(-1, 0, 0), 1, n = 21, robot = r,
                            initial = c(0, 0), weights = c(1, 0, 0, 0, 0, 0), restarts = 0)
  # Joint a rests on its lower limit, so b alone follows x(t) = -s(t).
  expect_lt(max(abs(z$joint$q - cbind(0, -z$s))), 1e-7)
  expect_lt(max(abs(z$joint$qd - cbind(0, z$velocity[, 1]))), 1e-6)
  expect_lt(max(abs(z$joint$qdd - cbind(0, z$acceleration[, 1]))), 1e-5)
  # Moving away from the limit the redundant pair shares the motion again.
  away <- robot_traj_cartesian(robot_pose(), transl(1, 0, 0), 1, n = 21, robot = r,
                               initial = c(0, 0), weights = c(1, 0, 0, 0, 0, 0),
                               restarts = 0)
  expect_equal(unname(rowSums(away$joint$qd)), unname(away$velocity[, 1]), tolerance = 1e-8)
  # An unreachable target leaves derivatives flagged where IK fails.
  far <- suppressWarnings(robot_traj_cartesian(robot_pose(), transl(-5, 0, 0), 1, n = 11,
                                               robot = r, initial = c(0, 0),
                                               weights = c(1, 0, 0, 0, 0, 0), restarts = 0))
  expect_identical(far$derivatives_ok, far$derivatives_ok & far$converged)
  expect_false(all(far$derivatives_ok))
})
