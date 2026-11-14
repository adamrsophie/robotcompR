test_that("closed-form two-link IK returns both elbow solutions", {
  sol <- ik_planar2(-1, 1, 1, 1)
  expect_equal(nrow(sol), 2L)
  for (i in 1:2) {
    expect_equal(ref_planar_fk(c(1, 1), sol[i, ]), c(-1, 1), tolerance = 1e-14)
  }
  expect_equal(unname(sol["q2_positive", ]), c(pi / 2, pi / 2), tolerance = 1e-14)
  expect_equal(nrow(ik_planar2(3, 0, 1, 1)), 0L)
  # Boundary of the workspace: a single solution.
  b <- ik_planar2(2, 0, 1, 1)
  expect_equal(nrow(b), 1L)
  expect_equal(unname(b[1, ]), c(0, 0))
  set.seed(20)
  for (i in 1:20) {
    q <- c(runif(1, -pi, pi), runif(1, 0.1, 3))
    p <- ref_planar_fk(c(0.7, 0.4), q)
    s <- ik_planar2(p[1], p[2], 0.7, 0.4)
    expect_equal(unname(s["q2_positive", ]), c(wrap_angle(q[1]), q[2]),
                 tolerance = 1e-10)
  }
})

test_that("numerical IK reproduces both analytic two-link solutions", {
  robot <- robot_example("planar2")
  ref <- ik_planar2(-1, 1, 1, 1)
  for (m in c("levenberg_marquardt", "damped_least_squares", "pseudoinverse")) {
    up <- robot_ik(robot, c(-1, 1, 0), initial = c(1.2, 1.2), method = m)
    expect_true(up$converged)
    expect_equal(unname(up$q), unname(ref["q2_positive", ]), tolerance = 1e-6)
    down <- robot_ik(robot, c(-1, 1, 0), initial = c(3, -1.2), method = m)
    expect_true(down$converged)
    expect_equal(ref_planar_fk(c(1, 1), down$q), c(-1, 1), tolerance = 1e-8)
    expect_lt(down$q[2], 0)
  }
})

test_that("numerical IK solves random reachable planar and spatial targets", {
  set.seed(21)
  robot <- robot_example("planar3")
  for (i in 1:15) {
    q <- robot_random_configuration(robot)
    p <- pose_translation(robot_fk(robot, q))
    sol <- robot_ik(robot, p, restarts = 5, seed = i)
    expect_true(sol$converged)
    expect_equal(ref_planar_fk(c(1, 0.8, 0.6), sol$q), p[1:2], tolerance = 1e-8)
  }
  puma <- robot_example("puma560")
  for (i in 1:10) {
    q <- robot_random_configuration(puma)
    T <- robot_fk(puma, q)
    sol <- robot_ik(puma, T, initial = robot_clamp(puma, q + rnorm(6, sd = 0.4)),
                    restarts = 5, seed = i)
    expect_true(sol$converged)
    d <- robot_pose_distance(robot_fk(puma, sol$q), T)
    expect_lt(d[["translation"]], 1e-8)
    expect_lt(d[["rotation"]], 1e-8)
    expect_true(robot_within_limits(puma, sol$q))
  }
})

test_that("planar pose targets work with weights", {
  robot <- robot_example("planar3")
  q <- c(0.4, -0.6, 0.9)
  T <- robot_fk(robot, q)
  sol <- robot_ik(robot, T, initial = c(0, 0, 0), weights = c(1, 1, 0, 0, 0, 1))
  expect_true(sol$converged)
  expect_lt(max(abs(as.matrix(robot_fk(robot, sol$q)) - as.matrix(T))), 1e-8)
})

test_that("unreachable targets are reported, not claimed solved", {
  robot <- robot_example("planar2")
  sol <- robot_ik(robot, c(3, 0, 0))
  expect_false(sol$converged)
  expect_gt(sol$target_distance, sol$reach_bound)
  expect_equal(sol$position_error, 1, tolerance = 1e-6)
  # The best effort reaches straight towards the target.
  expect_equal(ref_planar_fk(c(1, 1), sol$q), c(2, 0), tolerance = 1e-6)
  expect_true(sol$singular)
  expect_output(print(sol), "NOT converged")
})

test_that("joint limits are enforced and reported", {
  robot <- robot_set_limits(robot_example("planar2"),
                            rbind(c(-pi, pi), c(0, pi / 4)))
  # (-1, 1) needs q2 = +/- pi/2; with q2 <= pi/4 it is not reachable.
  sol <- robot_ik(robot, c(-1, 1, 0), initial = c(1.5, 0.1), restarts = 3, seed = 1)
  expect_false(sol$converged)
  expect_true(robot_within_limits(robot, sol$q))
  expect_true("joint2" %in% sol$active_limits)
  # A target that is reachable inside the limits.
  q <- c(0.5, 0.5)
  p <- ref_planar_fk(c(1, 1), q)
  sol2 <- robot_ik(robot, c(p, 0), initial = c(0, 0.2))
  expect_true(sol2$converged)
  expect_equal(unname(sol2$q), q, tolerance = 1e-6)
})

test_that("IK started at a singular configuration still converges", {
  robot <- robot_example("planar2")
  sol <- robot_ik(robot, c(1, 1, 0), initial = c(0, 0))
  expect_true(sol$converged)
  expect_equal(ref_planar_fk(c(1, 1), sol$q), c(1, 1), tolerance = 1e-8)
})

test_that("IK restarts do not change the global random state", {
  robot <- robot_example("planar2")
  set.seed(99)
  before <- .Random.seed
  robot_ik(robot, c(3, 0, 0), restarts = 3, seed = 5)
  expect_identical(.Random.seed, before)
})
