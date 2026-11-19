# EPA depth, continuous path checks, Cartesian velocities, UKF, EKF-SLAM
# and SE(3) pose graphs.

test_that("EPA penetration depth matches closed forms", {
  b <- geom_box(c(1, 1, 1))
  expect_equal(geom_distance(b, b, pose_b = transl(0.7, 0.1, 0))$distance, -0.3,
               tolerance = 1e-8)
  expect_equal(geom_distance(b, b, pose_b = robot_pose(rotz(pi / 4), c(0.9, 0, 0)))$distance,
               0.9 - 0.5 - sqrt(2) / 2, tolerance = 1e-8)
  # Sphere center inside a box: depth is the radius plus the distance to
  # the nearest face.
  expect_equal(geom_distance(geom_sphere(0.5), b, pose_b = transl(0.2, 0, 0))$distance,
               -0.8, tolerance = 1e-8)
  cyl <- geom_cylinder(0.5, 2)
  expect_equal(geom_distance(cyl, cyl, pose_b = transl(0.6, 0, 0))$distance, -0.4,
               tolerance = 1e-6)
  cap <- geom_capsule(0.1, 1)
  d <- geom_distance(cap, b, pose_a = transl(0, 0, 0.3))
  expect_true(d$collision)
  expect_equal(d$distance, -(0.5 + 0.1), tolerance = 1e-8)
  # Moving b by the depth along the witness direction separates the shapes.
  d <- geom_distance(b, b, pose_b = transl(0.7, 0.2, 0))
  n <- (d$point_a - d$point_b) / sqrt(sum((d$point_a - d$point_b)^2))
  shift <- c(0.7, 0.2, 0) + (-d$distance + 1e-6) * n
  moved <- geom_distance(b, b, pose_b = transl(shift[1], shift[2], shift[3]))
  expect_false(moved$collision)
  expect_lt(moved$distance, 1e-5)
})

test_that("continuous path checks catch thin obstacles between samples", {
  arm <- robot_capsule_links(robot_example("planar2"), 0.05)
  post <- robot_environment(post = geom_box(c(0.02, 0.02, 1),
                                            origin = transl(cos(0.6), sin(0.6), 0)))
  P <- rbind(c(0, 0), c(1.2, 0))
  expect_true(robot_check_path(arm, P, post, resolution = 0.5)$valid)
  cc <- robot_check_path(arm, P, post, method = "continuous")
  expect_false(cc$valid)
  expect_equal(cc$first_invalid, 1)
  expect_lt(abs(robot_check_collision(arm, cc$q, post)$min_distance), 1e-6)
  # The first contact: everything before it is free.
  expect_true(robot_check_path(arm, rbind(c(0, 0), cc$q * 0.999), post,
                               resolution = 1e-3)$valid)
  far <- robot_environment(w = geom_box(c(0.2, 0.2, 0.2), origin = transl(-1.5, -1.5, 0)))
  expect_true(robot_check_path(arm, P, far, method = "continuous")$valid)
  slide <- robot_model(joints = list(
    robot_joint("r", "revolute", "base", "l1", axis = c(0, 0, 1)),
    robot_joint("s", "prismatic", "l1", "l2", axis = c(1, 0, 0))))
  slide <- robot_attach_geometry(slide, "l2", geom_sphere(0.1, origin = transl(1, 0, 0)))
  # Bounds come from the extensions reached on the segment, so joints
  # without finite limits are supported.
  far_post <- robot_environment(p = geom_sphere(0.1, origin = transl(0, 3, 0)))
  expect_true(robot_check_path(slide, rbind(c(0, 0), c(0.1, 0)), far_post,
                               method = "continuous")$valid)
  expect_false(robot_check_path(slide, rbind(c(0, 0), c(pi / 2, 1.9)), far_post,
                                method = "continuous")$valid)
})

test_that("Cartesian trajectories report consistent velocities", {
  a <- robot_pose(rotx(0.2), c(0, 0, 0))
  b <- robot_pose(rotz(1) %*% rotx(0.2), c(1, 2, 0))
  ct <- robot_traj_cartesian(a, b, duration = 2, n = 401)
  P <- t(vapply(ct$poses, pose_translation, numeric(3)))
  fd <- apply(P, 2, function(x) diff(x) / diff(ct$time))
  mid <- (ct$velocity[-1, 1:3] + ct$velocity[-401, 1:3]) / 2
  expect_lt(max(abs(fd - mid)), 1e-4)
  # Angular velocity from successive rotations: log(R_{k+1} R_k^T) / dt.
  Rk <- lapply(ct$poses, function(p) as_plain_matrix(pose_rotation(p)))
  w <- t(vapply(1:400, function(k) so3_log(Rk[[k + 1]] %*% t(Rk[[k]])) /
                  diff(ct$time)[k], numeric(3)))
  expect_lt(max(abs(w - (ct$velocity[-1, 4:6] + ct$velocity[-401, 4:6]) / 2)), 1e-4)
  expect_equal(unname(ct$velocity[c(1, 401), ]), matrix(0, 2, 6))
  arm <- robot_example("planar2")
  from <- robot_fk(arm, c(0.3, 1.2))
  to <- robot_pose(translation = c(1.2, 0.6, 0))
  jt <- robot_traj_cartesian(from, to, 1, n = 201, robot = arm, initial = c(0.3, 1.2),
                             weights = c(1, 1, 0, 0, 0, 0))
  expect_true(all(jt$converged))
  expect_s3_class(jt$joint, "robot_trajectory")
  fq <- apply(jt$joint$q, 2, function(x) diff(x) / diff(jt$time))
  expect_lt(max(abs(fq - (jt$joint$qd[-1, ] + jt$joint$qd[-201, ]) / 2)), 1e-3)
  fqd <- apply(jt$joint$qd, 2, function(x) diff(x) / diff(jt$time))
  expect_lt(max(abs(fqd - (jt$joint$qdd[-1, ] + jt$joint$qdd[-201, ]) / 2)), 1e-2)
})

test_that("UKF and EKF agree on a well-observed problem", {
  L <- rbind(a = c(5, 0), b = c(0, 5), c = c(5, 5), d = c(-2, 3))
  Q <- diag(c(0.02, 0.01))
  R <- diag(c(0.05, 0.002))
  fe <- robot_ekf(c(0, 0, 0), diag(0.01, 3), L, Q, R)
  fu <- robot_ukf(c(0, 0, 0), diag(0.01, 3), L, Q, R)
  truth <- c(0, 0, 0)
  set.seed(7)
  e <- numeric(0)
  for (k in 1:100) {
    u <- c(1, 0.25)
    un <- u + as.numeric(rnorm(2) %*% chol(Q))
    truth <- unicycle_step(truth, un[1], un[2], 0.1)
    fe <- ekf_predict(fe, u, 0.1)
    fu <- ukf_predict(fu, u, 0.1)
    if (k %% 5 == 0) {
      z <- landmark_measurements(truth, L, R, max_range = 6, seed = k)
      fe <- ekf_update(fe, z)
      fu <- ukf_update(fu, z)
    }
    e <- c(e, sqrt(sum((fu$mean[1:2] - truth[1:2])^2)))
  }
  expect_lt(max(abs(fu$mean - fe$mean)), 0.02)
  expect_lt(mean(e), 0.15)
  expect_true(min(eigen(fu$cov, only.values = TRUE)$values) > 0)
  # With linear dynamics (zero turn rate, no pose uncertainty in heading) the
  # unscented prediction equals the EKF prediction.
  fe <- robot_ekf(c(0, 0, 0), diag(c(0.1, 0.2, 0)), L, diag(c(0.01, 0)), R)
  fu <- robot_ukf(c(0, 0, 0), diag(c(0.1, 0.2, 0)), L, diag(c(0.01, 0)), R)
  expect_equal(ukf_predict(fu, c(1, 0), 1)$cov, ekf_predict(fe, c(1, 0), 1)$cov,
               tolerance = 1e-10, ignore_attr = TRUE)
  expect_error(ukf_update(fu, data.frame(id = "zz", range = 1, bearing = 0)), "Unknown")
})

test_that("EKF-SLAM builds a consistent map", {
  L <- rbind(a = c(5, 0), b = c(0, 5), c = c(5, 5), d = c(-2, 3))
  Q <- diag(c(0.02, 0.01))
  R <- diag(c(0.05, 0.002))
  f <- robot_ekf_slam(c(0, 0, 0), diag(1e-9, 3), Q, R)
  # A noiseless first sighting initializes the landmark exactly.
  f1 <- ekf_slam_update(f, landmark_measurements(c(0, 0, 0), L[c("a", "b"), ]))
  expect_equal(ekf_slam_landmarks(f1), L[c("a", "b"), ], tolerance = 1e-12,
               ignore_attr = TRUE)
  expect_equal(dim(f1$cov), c(7, 7))
  truth <- c(0, 0, 0)
  set.seed(7)
  for (k in 1:200) {
    u <- c(1, 0.25)
    un <- u + as.numeric(rnorm(2) %*% chol(Q))
    truth <- unicycle_step(truth, un[1], un[2], 0.1)
    f <- ekf_slam_predict(f, u, 0.1)
    if (k %% 5 == 0) {
      f <- ekf_slam_update(f, landmark_measurements(truth, L, R, max_range = 6, seed = k))
    }
  }
  M <- ekf_slam_landmarks(f)
  expect_setequal(rownames(M), rownames(L))
  expect_lt(max(sqrt(rowSums((M - L[rownames(M), ])^2))), 0.3)
  expect_lt(sqrt(sum((f$mean[1:2] - truth[1:2])^2)), 0.3)
  expect_true(isSymmetric(f$cov))
  expect_gt(min(eigen(f$cov, only.values = TRUE)$values), 0)
})

test_that("SE(3) pose graphs recover the true poses", {
  set.seed(3)
  n <- 12
  step <- se3_exp(c(1, 0, 0.1, 0, 0.2, 0.5))
  truth <- Reduce(function(X, k) X %*% step, 2:n, accumulate = TRUE, init = diag(4))
  init <- lapply(truth, function(M) M %*% se3_exp(rnorm(6, sd = 0.05)))
  init[[1]] <- diag(4)
  rel <- function(a, b) solve(truth[[a]]) %*% truth[[b]]
  g <- robot_pose_graph(init)
  expect_equal(g$group, "SE3")
  for (k in 1:(n - 1)) g <- pose_graph_add_edge(g, k, k + 1, rel(k, k + 1))
  g <- pose_graph_add_edge(g, 1, n, robot_pose(matrix = rel(1, n)), diag(10, 6))
  res <- pose_graph_optimize(g)
  expect_true(res$converged)
  err <- max(vapply(1:n, function(k) max(abs(as_plain_matrix(res$graph$nodes[[k]]) -
                                                truth[[k]])), 0))
  expect_lt(err, 1e-8)
  expect_error(pose_graph_add_edge(g, 1, 2, transl(1, 0, 0), diag(3)), "information")
  expect_error(pose_graph_add_edge(g, 1, 2, c(1, 0, 0)), "SE\\(3\\)")
  pdf(NULL)
  on.exit(grDevices::dev.off())
  expect_silent(plot(res$graph))
})
