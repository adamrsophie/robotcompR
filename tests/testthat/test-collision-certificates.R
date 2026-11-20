spin_robot <- function(geometry) {
  robot_attach_geometry(
    robot_model(joints = robot_joint("j", "continuous", "a", "b")), "b", geometry)
}

test_that("thin and nearly parallel geometry is never reported clear", {
  b <- geom_box(c(2, 2, 2e-8))
  expect_true(geom_distance(b, b, pose_b = transl(0.25, 0.25, 5e-9))$collision)
  a <- geom_capsule(1e-10, 2)
  d <- geom_distance(a, a, pose_b = trotx(1e-8))
  expect_true(d$collision)
  expect_equal(d$distance, -2e-10, tolerance = 1e-6)
  g <- geom_capsule(1e-9, 2)
  ob <- geom_capsule(1e-9, 2, origin = troty(1e-8))
  expect_false(robot_check_path(spin_robot(g), matrix(0, 1), robot_environment(ob),
                                method = "continuous")$valid)
  h <- geom_convex(rbind(c(0, 0, 0), c(1, 0, 0), c(0, 1e-8, 0)))
  s <- geom_sphere(1e-10, origin = transl(0.3, 3e-9, 0))
  expect_true(geom_distance(h, s)$collision)
  expect_false(robot_check_path(spin_robot(s), matrix(0, 1), robot_environment(h),
                                method = "continuous")$valid)
})

test_that("results do not depend on scale, rotation or the local origin", {
  cyl <- function(s) {
    g <- geom_cylinder(s, 2 * s)
    geom_distance(g, g, pose_b = transl(1.5 * s, 0, 0))$distance / s
  }
  expect_equal(cyl(1e-16), cyl(1), tolerance = 1e-9)
  h <- geom_convex(rbind(c(-1, -1, 0), c(1, -1, 0), c(0, 1, 0), c(0, 0, 0.1)))
  sp <- geom_sphere(0.1)
  depth <- function(T) {
    geom_distance(h, sp, T, T %*% as_plain_matrix(transl(0, 0, 0.025)))$distance_bounds
  }
  expect_equal(depth(as_plain_matrix(troty(0.4))), depth(diag(4)), tolerance = 1e-9)
  gap <- function(offset) {
    a <- geom_convex(as.matrix(expand.grid(c(-0.5, 0.5) + offset, c(-0.5, 0.5),
                                           c(-0.5, 0.5))))
    geom_distance(a, a, pose_b = transl(1.005, 0, 0))$distance
  }
  expect_equal(gap(1e8), gap(0), tolerance = 1e-5)
})

test_that("unresolved depth is reported with honest bounds and surface witnesses", {
  b <- geom_box(c(1, 1, 1e-10))
  d <- geom_distance(b, b)
  expect_false(d$depth_known)
  expect_lte(d$distance_bounds[1], -1e-10 * (1 - 1e-6))
  r <- geom_distance(geom_sphere(1), geom_sphere(1))
  expect_equal(vnorm(r$point_a), 1)
  expect_equal(vnorm(r$point_b), 1)
})

test_that("path checks validate inputs and report problems in path order", {
  j <- robot_joint("slide", "prismatic", "base", "tip", axis = c(1, 0, 0))
  r <- robot_attach_geometry(robot_model(joints = j), "tip", geom_sphere(0.1))
  e <- robot_environment(geom_sphere(0.1, transl(1, 0, 0)))
  p <- matrix(c(-1e16, 1), ncol = 1)
  expect_false(robot_check_path(r, p, e, method = "continuous")$valid)
  expect_false(robot_check_path(r, p, e, resolution = 1e16)$valid)
  expect_error(robot_check_path(r, matrix(c(0, 1), ncol = 1), list(geom_sphere(1))),
               "robot_environment")
  expect_error(robot_check_path(r, matrix(0, 1), e, margin = -3), "margin")
  expect_error(robot_check_path(r, matrix(numeric(0), 0, 1)), "row")
  expect_error(robot_planner(r, environment = list(geom_sphere(1))), "robot_environment")
  arm <- robot_capsule_links(robot_example("planar2"), 0.05)
  box <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4), origin = transl(1.5, 0, 0)))
  x <- robot_check_path(arm, rbind(c(pi / 2, 0), c(-pi / 2, 0), c(-pi / 2, 4)), box,
                        method = "continuous")
  expect_identical(x$first_invalid, 1L)
  expect_lt(abs(x$q[2]), 1e-12)
  expect_error(robot_check_collision(arm, c(0, 0.5),
                                     ignore_pairs = list(c("link1", "nope"))), "link")
  # Fixed frames inserted by URDF export do not create new self pairs.
  b <- robot_read_urdf(robot_write_urdf(arm, fill_limits = c(effort = 5, velocity = 1)))
  expect_false(robot_check_collision(b, c(0, 0))$in_collision)
})

test_that("planners respect deadlines, progress and trivial queries", {
  calls <- 0L
  v <- function(q) {
    if (q == 1) {
      calls <<- calls + 1L
      if (calls == 4L) Sys.sleep(0.2)
    }
    TRUE
  }
  p <- robot_planner(algorithm = "rrt", validity = v, bounds = rbind(c(0, 1)),
                     goal_bias = 1, step_size = 2, resolution = 2, time_limit = 0.1)
  expect_identical(robot_plan(p, 0, 1, seed = 1)$termination, "time_limit")
  n <- 0L
  q <- robot_planner(bounds = matrix(c(1, 2), 1), step_size = 1e-17, max_iterations = 1,
                     validity = function(q) {
                       n <<- n + 1L
                       if (n > 100) stop("no progress")
                       TRUE
                     })
  expect_identical(robot_plan(q, 1, 2, seed = 1)$termination, "iteration_limit")
  for (a in c("rrt", "rrt_connect", "rrt_star", "prm")) {
    z <- robot_plan(robot_planner(bounds = matrix(0:1, 1), algorithm = a,
                                  validity = function(q) TRUE), 0.5, 0.5, seed = 1)
    expect_true(z$success)
    expect_identical(z$length, 0)
  }
})

test_that("a tool fixed to a link is still checked against the link before it", {
  r <- robot_model(joints = list(
    robot_joint("j1", "revolute", "base", "l1"),
    robot_joint("j2", "revolute", "l1", "l2", origin = transl(1, 0, 0)),
    robot_joint("t", "fixed", "l2", "tool", origin = transl(1, 0, 0)),
    robot_joint("f", "fixed", "base", "mount"),
    robot_joint("j3", "revolute", "mount", "l3", origin = transl(0, 0, 5))))
  for (k in c("base", "l1", "l2", "tool", "mount", "l3")) {
    r <- robot_attach_geometry(r, k, geom_sphere(0.1))
  }
  objs <- robotcompR:::robot_collision_objects(r, robotcompR:::fk_internal(r, c(0, 0, 0)))
  links <- r$link_names[vapply(objs, function(o) o$link, integer(1))]
  pairs <- vapply(robotcompR:::default_self_pairs(r, objs), function(p)
    paste(sort(links[p]), collapse = "-"), "")
  expect_true("l1-tool" %in% pairs)
  expect_true("base-l3" %in% pairs)
  expect_false("l1-l2" %in% pairs)
  expect_false("l2-tool" %in% pairs)
  expect_false("base-mount" %in% pairs)
  expect_false("l3-mount" %in% pairs)
  # Folding the second joint back puts the tool on the first link.
  expect_true(robot_check_collision(r, c(0, pi, 0))$in_collision)
})

box_truth <- function(sa, sb, delta) max(abs(delta) - (sa + sb) / 2)

test_that("thin overlapping boxes are never certified separated", {
  sa <- c(4.6, 4.2, 1.7e-7)
  sb <- c(4.4, 1.34, 4e-7)
  delta <- c(2, 2.2, 1.28e-7)
  truth <- box_truth(sa, sb, delta)
  R <- rotz(0.7) %*% rotx(-0.3)
  for (common in list(robot_pose(), robot_pose(R, c(1e3, -50, 7)))) {
    Ta <- common
    Tb <- common * transl(delta)
    for (swap in c(FALSE, TRUE)) {
      d <- if (swap) geom_distance(geom_box(sb), geom_box(sa), Tb, Ta)
           else geom_distance(geom_box(sa), geom_box(sb), Ta, Tb)
      expect_true(d$collision)
      expect_lte(d$distance_bounds[1], truth + 1e-12)
      expect_gte(d$distance_bounds[2], truth - 1e-12)
    }
  }
  r <- robot_attach_geometry(robot_model(joints = robot_joint("j", "prismatic", "base",
                                                              "tip", axis = c(1, 0, 0))),
                             "tip", geom_box(sa))
  e <- robot_environment(geom_box(sb, origin = transl(delta)))
  expect_true(robot_check_collision(r, 0, e, self = FALSE)$in_collision)
  for (m in c("discrete", "continuous")) {
    expect_false(robot_check_path(r, matrix(0, 1), e, self = FALSE, method = m)$valid)
  }
})

test_that("separation needs a positive certified lower bound", {
  set.seed(11)
  for (i in 1:30) {
    sa <- c(runif(2, 0.1, 3), 1e-9 * runif(1, 0.2, 2))
    sb <- c(runif(2, 0.1, 3), 1e-9 * runif(1, 0.2, 2))
    delta <- runif(3, -0.8, 0.8) * (sa + sb) / 2
    truth <- box_truth(sa, sb, delta)
    d <- geom_distance(geom_box(sa), geom_box(sb), pose_b = transl(delta))
    expect_lte(d$distance_bounds[1], truth + 1e-12)
    expect_gte(d$distance_bounds[2], truth - 1e-12)
    if (truth <= 0) expect_true(d$collision)
  }
  # Genuinely separated shapes keep an exact positive certificate.
  d <- geom_distance(geom_box(c(1, 1, 1e-8)), geom_box(c(1, 1, 1e-8)),
                     pose_b = transl(1.5, 0, 0))
  expect_false(d$collision)
  expect_true(d$depth_known)
  expect_gt(d$distance_bounds[1], 0)
  expect_equal(d$distance, 0.5, tolerance = 1e-9)
  # A truncated GJK run still returns a valid lower bound.
  a <- geom_cylinder(0.5, 1)
  b <- geom_cylinder(0.5, 1)
  g <- gjk_distance(a, diag(4), b, transl(1.3, 0.2, 0)$matrix, max_iter = 1)
  expect_lte(g$lower, (sqrt(1.73) - 1) * (1 + 1e-9))
  expect_lte(g$lower, g$distance)
})

test_that("flat penetration intervals are measured from the Minkowski origin", {
  sa <- c(2, 2, 5e-11)
  sb <- c(0.9, 3, 2e-10)
  delta <- c(1, 0.1, 1e-11)
  truth <- box_truth(sa, sb, delta)
  for (swap in c(FALSE, TRUE)) {
    d <- if (swap) geom_distance(geom_box(sb), geom_box(sa), transl(delta), robot_pose())
         else geom_distance(geom_box(sa), geom_box(sb), pose_b = transl(delta))
    expect_true(d$collision)
    expect_lte(d$distance_bounds[1], truth + 1e-18)
    expect_gte(d$distance_bounds[2], truth - 1e-18)
  }
})
