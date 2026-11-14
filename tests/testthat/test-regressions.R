# Edge cases found in review, each pinned by a concrete input.

test_that("GJK tolerances scale with the size of the geometry", {
  tiny <- geom_box(rep(1e-6, 3))
  expect_true(geom_distance(tiny, tiny, pose_b = transl(5e-7, 0, 0))$collision)
  expect_equal(geom_distance(tiny, tiny, pose_b = transl(3e-6, 0, 0))$distance, 2e-6,
               tolerance = 1e-9)
  big <- geom_box(rep(1e3, 3))
  d <- geom_distance(big, big, pose_b = transl(1000.5, 0, 0))
  expect_false(d$collision)
  expect_equal(d$distance, 0.5, tolerance = 1e-6)
})

test_that("DH capsules follow both conventions and cover d offsets", {
  r <- robot_capsule_links(robot_dh(a = c(0, 1, 1), alpha = c(0, 0, 0), d = c(0, 0, 0),
                                    theta = c(0, 0, 0), convention = "modified"), 0.05)
  env <- robot_environment(s = geom_sphere(0.1, origin = transl(0.5, 0, 0)))
  expect_true(robot_check_collision(r, c(0, pi / 2, 0), env)$in_collision)
  puma <- robot_capsule_links(robot_example("puma560"), 0.05)
  n <- vapply(puma$links, function(l) length(l$collision), 0L)
  expect_equal(n[["link4"]], 1L)  # the forearm, along d4
  expect_false(robot_check_collision(puma, rep(0, 6))$in_collision)
  pr <- robot_dh(c(0, 0), c(0, 0), c(0, 0), c(0, 0), c("revolute", "prismatic"))
  expect_warning(robot_capsule_links(pr, 0.05), "prismatic")
})

test_that("unicycle motion and EKF Jacobians are accurate near zero turn rate", {
  expect_equal(unname(unicycle_step(c(0, 0, 1), 1, 1e-11, 1)), c(cos(1), sin(1), 1),
               tolerance = 1e-10)
  f <- robot_ekf(c(0, 0, 0), matrix(0, 3, 3), rbind(a = c(2, 0)), diag(c(0, 1)), diag(2))
  P <- ekf_predict(f, c(1, 1e-9), 1)$cov
  expect_equal(P[2, 2], 0.25, tolerance = 1e-6)
  expect_equal(P[2, 3], 0.5, tolerance = 1e-6)
  x <- c(0.3, -0.2, 0.7)
  h <- 1e-6
  for (w in c(0.6, 1e-7)) {
    m <- unicycle_motion(x, 0.8, w, 0.5)
    fd <- sapply(1:2, function(k) {
      d <- replace(c(0, 0), k, h)
      (unicycle_step(x, 0.8 + d[1], w + d[2], 0.5) -
         unicycle_step(x, 0.8 - d[1], w - d[2], 0.5)) / (2 * h)
    })
    expect_equal(unname(fd), m$G, tolerance = 1e-7)
  }
})

test_that("bicycle saturation keeps the steering geometry", {
  sim <- mobile_simulate(robot_bicycle(wheelbase = 1, max_speed = 1), c(0, 0, 0),
                         matrix(c(2, pi / 4), 1), dt = 1, saturate = TRUE)
  expect_equal(unname(sim[2, 2:4]), c(sin(1), 1 - cos(1), 1), tolerance = 1e-12)
  expect_error(mobile_simulate(robot_unicycle(max_turn_rate = 1), c(0, 0, 0),
                               matrix(c(1, 2), 1), 1), "max_turn_rate")
  expect_error(pure_pursuit(c(0, 0, 0), matrix(numeric(0), 0, 2), 1, 1), "waypoint")
})

test_that("planners respect joint limits, grow trees and honor time limits", {
  r <- robot_example("planar1")
  p <- robot_planner(r, algorithm = "rrt", bounds = matrix(c(-10, 10), 1),
                     goal_bias = 1, step_size = 1, max_iterations = 1)
  expect_match(robot_plan(p, 4, 5)$message, "start")
  p <- robot_planner(validity = function(q) TRUE, bounds = matrix(c(0, 1), 1),
                     step_size = 0.01, max_iterations = 1)
  expect_true(robot_plan(p, 0, 1, seed = 1)$success)
  p <- robot_planner(validity = function(q) abs(q[1] - 0.5) > 0.05,
                     bounds = rbind(c(0, 1), c(0, 1)), max_iterations = 1e5,
                     time_limit = 0.2)
  res <- robot_plan(p, c(0.1, 0.1), c(0.9, 0.1), seed = 1)
  expect_false(res$success)
  expect_match(res$message, "time limit")
})

test_that("one-sided limits give valid samples", {
  r <- robot_model(joints = list(robot_joint("slide", "prismatic", "a", "b",
                                             limits = c(2, Inf))))
  Q <- robot_random_configuration(r, 20, seed = 1)
  expect_true(all(Q >= 2))
  expect_equal(unname(robot_planner(r)$bounds[1, ]), c(2, 4))
})

test_that("scan updates mark the hit cell for beams in any direction", {
  occ <- matrix(c(TRUE, FALSE), 1, 2)
  w <- robot_occupancy_grid(c(0, 2), c(0, 1), 1, occupied = occ)
  z <- grid_raycast(w, c(1.5, 0.5, pi), 0, 2)
  g <- grid_update_scan(robot_occupancy_grid(c(0, 2), c(0, 1), 1),
                        c(1.5, 0.5, pi), 0, z, 2)
  expect_equal(g$logodds, matrix(c(0.85, -0.4), 1))
  # A hit beyond the map edge clears every traversed cell.
  g <- grid_update_scan(robot_occupancy_grid(c(0, 1), c(0, 1), 0.1),
                        c(0.55, 0.55, 0), 0, 2, 3)
  expect_true(all(g$logodds[6, 6:10] < 0))
  # Inflation treats occupied cells as squares.
  w2 <- robot_occupancy_grid(c(0, 2), c(0, 2), 1,
                             occupied = matrix(c(TRUE, FALSE, FALSE, FALSE), 2))
  expect_equal(sum(grid_inflate(w2, 0.6)), 3)
  expect_equal(sum(grid_inflate(w2, 0.4)), 1)
})

test_that("inverse kinematics edge cases", {
  fixed <- robot_model(joints = list(robot_joint("f", "fixed", "base", "tool")))
  s <- robot_ik(fixed, c(0, 0, 0))
  expect_true(s$converged)
  expect_length(s$q, 0)
  s <- robot_ik(robot_example("planar3"), c(1.2, 0.9, 0))
  expect_true(s$converged)
  expect_false(s$singular)
  s <- robot_ik(robot_example("planar2"), robot_pose(rotz(1), c(1, 1, 0)),
                weights = c(1, 1, 0, 0, 0, 0))
  expect_true(is.na(s$orientation_error))
  expect_equal(unname(ik_planar2(0, 0, 1, 1)[1, ]), c(0, pi))
  puma <- robot_example("puma560")
  Q <- robot_random_configuration(puma, 6, seed = 5)
  conv <- vapply(1:6, function(i) robot_ik(puma, robot_fk(puma, Q[i, ]),
                                           initial = rep(0.1, 6))$converged, TRUE)
  expect_gte(sum(conv), 5)
  set.seed(9)
  before <- .Random.seed
  robot_ik(puma, robot_fk(puma, Q[1, ]), initial = rep(0.1, 6))
  expect_identical(.Random.seed, before)
})

test_that("small rotations and proper Euler middle angles", {
  expect_equal(quaternion_distance(c(1, 0, 0, 0), c(1, 5e-10, 0, 0)), 1e-9,
               tolerance = 1e-6)
  expect_equal(as_euler(roty(pi), "ZYZ")[2], pi)
  a <- as_euler(robot_rotation(euler = c(0.4, pi / 2, -0.3)), "ZYX")
  expect_equal(a[3], 0)
})

test_that("filters accept singular covariances and reject unknown ids", {
  pf <- robot_particle_filter(4, c(0, 0, 0), diag(c(1, 1, 0)), rbind(a = c(1, 1)),
                              diag(c(0.1, 0)), diag(2), seed = 1)
  expect_true(all(pf$particles[, 3] == 0))
  expect_error(pf_update(pf, data.frame(id = "z", range = 1, bearing = 0)), "Unknown")
  f <- robot_ekf(c(1, 1, 0), diag(3), rbind(a = c(1, 1)), diag(2), diag(2))
  expect_warning(out <- ekf_update(f, data.frame(id = "a", range = 0.1, bearing = 0)),
                 "coincides")
  expect_false(anyNA(out$mean))
})

test_that("trajectories, simulation and plotting edge cases", {
  st <- robot_traj_trapezoidal(1, 1, 1, 1)
  expect_equal(as.numeric(st$evaluate(1)$q), 1)
  e <- robot_traj_trapezoidal(0, 1, 1, 1)$evaluate(c(-1, 9))
  expect_equal(as.numeric(e$qdd), c(0, 0))
  sim <- robot_simulate(robot_example("planar1"), 0, times = c(0, 0.1), torque = 1,
                        gravity = c(0, 0, 0))
  expect_equal(unname(sim$q[2, 1]), 0.015, tolerance = 1e-6)
  pdf(NULL)
  on.exit(grDevices::dev.off())
  expect_silent(robot_plot(robot_dh(0, 0, 0, 0, joint_type = "prismatic"), 0.3))
})

test_that("URDF parsing never follows file content and export round trips", {
  dir <- withr::local_tempdir()
  target <- file.path(dir, "target.urdf")
  writeLines('<robot name="x"><link name="a"/></robot>', target)
  pointer <- file.path(dir, "pointer.urdf")
  writeLines(target, pointer)
  expect_error(robot_read_urdf(pointer))
  expect_error(robot_read_urdf(paste0(
    '<robot name="r"><link name="a"/><joint name="f" type="fixed">',
    '<parent link="a"/><child link="typo"/></joint></robot>')), "undeclared")
  a <- robot_example("planar1")
  b <- robot_read_urdf(robot_write_urdf(a))
  expect_true(robot_traj_check(b, robot_traj_polynomial(0, 1, 1))$ok)
  named <- a
  named$name <- "R&D <arm>"
  expect_equal(robot_read_urdf(robot_write_urdf(named))$name, "R&D <arm>")
  clash <- robot_add_frame(robot_example("planar1"), "link1_motion", "base",
                           set_end_effector = FALSE)
  expect_equal(robot_joint_names(robot_read_urdf(robot_write_urdf(clash))), "joint1")
  m <- robot_model(joints = list(
    robot_joint("ja", "continuous", "base", "a", post = robot_pose()),
    robot_joint("jb", "continuous", "base", "b"),
    robot_joint("jc", "continuous", "a", "c"),
    robot_joint("jd", "continuous", "b", "d")))
  expect_warning(robot_write_urdf(m), "reorder q")
})
