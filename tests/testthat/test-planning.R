# Point robot in the unit square with a wall at x = 0.5 that has a gap.
wall_valid <- function(gap = c(0.45, 0.55)) {
  function(q) !(abs(q[1] - 0.5) < 0.05 && (q[2] < gap[1] || q[2] > gap[2]))
}
unit_box <- rbind(c(0, 1), c(0, 1))

path_ok <- function(path, valid, res = 0.005) {
  for (i in seq_len(nrow(path) - 1)) {
    n <- ceiling(max(abs(path[i + 1, ] - path[i, ])) / res)
    for (s in seq(0, 1, length.out = n + 1)) {
      if (!valid(path[i, ] + s * (path[i + 1, ] - path[i, ]))) return(FALSE)
    }
  }
  TRUE
}

test_that("all planners solve a free space and a narrow passage problem", {
  valid <- wall_valid()
  for (alg in c("rrt", "rrt_connect", "rrt_star", "prm")) {
    free <- robot_planner(validity = function(q) TRUE, bounds = unit_box,
                          algorithm = alg, max_iterations = 300, prm_nodes = 50)
    p <- robot_plan(free, c(0.1, 0.1), c(0.9, 0.9), seed = 1)
    expect_true(p$success)
    expect_equal(p$path[1, ], c(0.1, 0.1))
    expect_equal(p$path[nrow(p$path), ], c(0.9, 0.9))
    pl <- robot_planner(validity = valid, bounds = unit_box, algorithm = alg,
                        step_size = 0.1, resolution = 0.005,
                        max_iterations = 3000, prm_nodes = 500, prm_neighbors = 15)
    p <- robot_plan(pl, c(0.1, 0.1), c(0.9, 0.1), seed = 2)
    expect_true(p$success, info = alg)
    expect_true(path_ok(p$path, valid), info = alg)
    expect_gt(p$length, 0.8)
  }
})

test_that("planners report invalid starts and goals and reproducible seeds", {
  pl <- robot_planner(validity = wall_valid(), bounds = unit_box)
  bad <- robot_plan(pl, c(0.5, 0.1), c(0.9, 0.9))
  expect_false(bad$success)
  expect_match(bad$message, "start")
  expect_match(robot_plan(pl, c(0.1, 0.1), c(2, 0.5))$message, "goal")
  a <- robot_plan(pl, c(0.1, 0.1), c(0.9, 0.1), seed = 7)
  set.seed(123)
  before <- .Random.seed
  b <- robot_plan(pl, c(0.1, 0.1), c(0.9, 0.1), seed = 7)
  expect_identical(.Random.seed, before)
  expect_identical(a$path, b$path)
  expect_equal(b$seed, 7)
  # A closed wall: failure is reported without claiming impossibility.
  closed <- robot_planner(validity = function(q) abs(q[1] - 0.5) >= 0.05,
                          bounds = unit_box, max_iterations = 200)
  f <- robot_plan(closed, c(0.1, 0.1), c(0.9, 0.1), seed = 1)
  expect_false(f$success)
  expect_match(f$message, "does not prove")
})

test_that("shortcutting never lengthens a path and keeps it valid", {
  valid <- wall_valid()
  pl <- robot_planner(validity = valid, bounds = unit_box, algorithm = "rrt",
                      step_size = 0.1, resolution = 0.005)
  p <- robot_plan(pl, c(0.1, 0.1), c(0.9, 0.1), seed = 3)
  s <- robot_shortcut(p, iterations = 200, seed = 3)
  expect_lte(s$length, p$length + 1e-12)
  expect_true(path_ok(s$path, valid))
  expect_equal(s$path[1, ], p$path[1, ])
})

test_that("RRT* finds a near-straight path in free space", {
  pl <- robot_planner(validity = function(q) TRUE, bounds = unit_box,
                      algorithm = "rrt_star", max_iterations = 600,
                      step_size = 0.2, goal_bias = 0.1)
  p <- robot_plan(pl, c(0, 0), c(1, 1), seed = 4)
  expect_true(p$success)
  expect_lt(p$length, 1.15 * sqrt(2))
})

test_that("planning a two-link arm around an obstacle respects collisions", {
  robot <- robot_capsule_links(robot_example("planar2"), 0.05)
  env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4), origin = transl(1.5, 0, 0)))
  pl <- robot_planner(robot, "rrt_connect", env, max_iterations = 500)
  p <- robot_plan(pl, c(-0.8, 0), c(0.8, 0), seed = 1)
  expect_true(p$success)
  expect_true(robot_check_path(robot, p$path, env, resolution = 0.01)$valid)
  expect_equal(colnames(p$path), c("joint1", "joint2"))
  pdf(NULL)
  on.exit(grDevices::dev.off())
  expect_silent(plot(p))
})
