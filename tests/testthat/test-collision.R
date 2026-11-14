test_that("primitive distances match closed forms", {
  d <- geom_distance(geom_sphere(0.5), geom_sphere(0.3), robot_pose(), transl(2, 0, 0))
  expect_equal(d$distance, 1.2, tolerance = 1e-12)
  expect_equal(d$point_a, c(0.5, 0, 0), tolerance = 1e-12)
  expect_equal(d$point_b, c(1.7, 0, 0), tolerance = 1e-12)
  # Sphere and box corner region.
  d <- geom_distance(geom_box(c(1, 1, 1)), geom_sphere(0.2), robot_pose(), transl(1.5, 1.5, 0))
  expect_equal(d$distance, sqrt(2) - 0.2, tolerance = 1e-9)
  # Parallel capsules (degenerate segment pair).
  d <- geom_distance(geom_capsule(0.1, 1), geom_capsule(0.1, 1), robot_pose(), transl(1, 0, 0))
  expect_equal(d$distance, 0.8, tolerance = 1e-12)
  # Crossed capsules.
  d <- geom_distance(geom_capsule(0.1, 2), geom_capsule(0.1, 2), robot_pose(),
                     transl(0, 0, 0) * trotx(pi / 2) * transl(0.5, 0, 0))
  expect_equal(d$distance, 0.3, tolerance = 1e-12)
  # Cylinder end cap and a sphere above it.
  d <- geom_distance(geom_cylinder(0.5, 1), geom_sphere(0.2), robot_pose(), transl(0, 0, 1))
  expect_equal(d$distance, 0.3, tolerance = 1e-9)
  # Rotated boxes face to face.
  d <- geom_distance(geom_box(c(1, 1, 1)), geom_box(c(1, 1, 1)), trotz(0.7), transl(3, 0, 0))
  expect_equal(d$distance, 2.5 - sqrt(2) / 2 * cos(pi / 4 - 0.7), tolerance = 1e-8)
  # Penetrating spheres: exact depth.
  d <- geom_distance(geom_sphere(0.5), geom_sphere(0.5), robot_pose(), transl(0.6, 0, 0))
  expect_true(d$collision)
  expect_equal(d$distance, -0.4, tolerance = 1e-12)
  expect_true(d$depth_known)
  # Overlapping boxes: collision reported, depth not computed.
  d <- geom_distance(geom_box(c(1, 1, 1)), geom_box(c(1, 1, 1)), robot_pose(), transl(0.5, 0, 0))
  expect_true(d$collision)
  expect_false(d$depth_known)
})

test_that("GJK agrees with brute force for random convex hulls", {
  set.seed(50)
  for (i in 1:20) {
    A <- matrix(rnorm(24), 8)
    B <- matrix(rnorm(24), 8) + matrix(c(4, 1, 0), 8, 3, byrow = TRUE)
    d <- geom_distance(geom_convex(A), geom_convex(B))
    # The minimum over vertex pairs bounds the hull distance from above.
    vv <- min(as.matrix(stats::dist(rbind(A, B)))[1:8, 9:16])
    expect_lte(d$distance, vv + 1e-9)
    # Closest points realize the distance and separate the hulls.
    expect_equal(sqrt(sum((d$point_a - d$point_b)^2)), d$distance, tolerance = 1e-8)
    n <- (d$point_b - d$point_a) / d$distance
    expect_lte(max(A %*% n), sum(d$point_a * n) + 1e-8)
    expect_gte(min(B %*% n), sum(d$point_b * n) - 1e-8)
  }
  expect_error(geom_distance(geom_mesh("x.stl"), geom_sphere(1)), "Mesh")
})

test_that("robot environment and self collision checks", {
  robot <- robot_capsule_links(robot_example("planar2"), 0.05)
  env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4), origin = transl(1.5, 0, 0)))
  expect_true(robot_check_collision(robot, c(0, 0), env)$in_collision)
  expect_false(robot_check_collision(robot, c(pi / 2, 0), env)$in_collision)
  r <- robot_check_collision(robot, c(pi / 2, 0), env, distances = TRUE)
  # Link 2 tip at (0, 2): nearest box face is x = 1.3.
  expect_equal(min(r$pairs$distance[r$pairs$type == "environment"]), sqrt(1.3^2) - 0.05 - 0,
               tolerance = 1e-6)
  # Folded arm: links are adjacent, so no self pair is evaluated.
  expect_false(robot_check_collision(robot_capsule_links(robot_example("planar2"), 0.05),
                                     c(0, 3.1))$in_collision)
  # Three-link arm folding link 3 back onto link 1 self-collides.
  r3 <- robot_capsule_links(robot_example("planar3"), 0.05)
  expect_true(robot_check_collision(r3, c(0, 3, 3))$in_collision)
  expect_false(robot_check_collision(r3, c(0, 3, 3),
                                     ignore_pairs = list(c("link1", "link3")))$in_collision)
  expect_true(robot_check_collision(robot, c(pi / 2, 0), env, margin = 2)$in_collision)
  expect_output(print(robot_check_collision(robot, c(0, 0), env)), "IN COLLISION")
})

test_that("path checks find collisions between collision-free endpoints", {
  robot <- robot_capsule_links(robot_example("planar2"), 0.05)
  env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4), origin = transl(1.5, 0, 0)))
  path <- rbind(c(pi / 2, 0), c(-pi / 2, 0))
  expect_false(robot_check_collision(robot, path[1, ], env)$in_collision)
  expect_false(robot_check_collision(robot, path[2, ], env)$in_collision)
  res <- robot_check_path(robot, path, env)
  expect_false(res$valid)
  expect_true(robot_check_collision(robot, res$q, env)$in_collision)
  expect_true(robot_check_path(robot, rbind(c(pi / 2, 0), c(pi, 0)), env)$valid)
})
