
slider_fixture <- function() {
  r <- robot_model(joints = list(robot_joint("s", "prismatic", "base", "tip",
    axis = c(1,0,0), limits = c(0,1))))
  robot_attach_geometry(r, "tip", geom_sphere(.5))
}

test_that("continuous paths reject final contact", {
  r <- slider_fixture()
  e <- robot_environment(obstacle=geom_sphere(.5,origin=transl(2,0,0)))
  expect_true(robot_check_collision(r,1,e,self=FALSE)$in_collision)
  expect_false(robot_check_path(r,matrix(c(0,1),2,1),e,self=FALSE,method="continuous")$valid)
})

test_that("duplicate obstacle names never hide an obstacle", {
  e <- tryCatch(robot_environment(same=geom_sphere(.25,origin=transl(20,0,0)),
                                 same=geom_sphere(.25)),error=identity)
  if(inherits(e,"error")) {
    expect_match(conditionMessage(e),"duplicat|unique",ignore.case=TRUE)
  } else {
    expect_true(robot_check_collision(slider_fixture(),0,e,self=FALSE)$in_collision)
  }
})

test_that("capsule overlap classification is scale invariant", {
  for(s in c(1,1e-6,1e-8)) {
    g <- geom_capsule(.01*s,2*s)
    d <- geom_distance(g,g,pose_b=transl(0,0,s))
    expect_true(d$collision)
    expect_equal(d$distance/s,-.02,tolerance=1e-8)
  }
})

test_that("supported point hulls have finite distance results", {
  g <- geom_convex(matrix(c(0,0,0),1,3))
  d <- geom_distance(g,g)
  expect_equal(d$distance,0)
  expect_true(d$collision)
  expect_true(all(is.finite(c(d$point_a,d$point_b))))
})

test_that("unconverged EPA must not claim an accurate known depth", {
  g <- geom_cylinder(.5,10)
  d <- geom_distance(g,g)
  expect_true(d$collision)
  expect_true(!isTRUE(d$depth_known) || abs(d$distance+1) < 1e-8)
})

test_that("constraint vectors are scalar or one value per joint", {
  expect_error(robot_traj_trapezoidal(c(0,0,0),c(1,1,1),c(1,2),1))
  expect_error(robot_traj_trapezoidal(c(0,0,0),c(1,1,1),1,c(1,2)))
})

test_that("nonfinite trajectory samples cannot be certified", {
  r <- robot_example("planar1")
  for(field in c("q","qd","qdd")) {
    tr <- robot_traj_polynomial(0,1,1)
    tr[[field]][20,1] <- NA_real_
    chk <- tryCatch(robot_traj_check(r,tr),error=identity)
    expect_true(inherits(chk,"error") || !isTRUE(chk$ok))
  }
})

test_that("an expired planning budget cannot produce unqualified success", {
  p <- robot_planner(validity=function(q){Sys.sleep(.002);TRUE},
    bounds=matrix(c(0,1),1),algorithm="rrt",step_size=1,goal_bias=1,
    resolution=.01,max_iterations=1,time_limit=.03)
  res <- robot_plan(p,0,1,seed=1)
  expect_false(res$success)
  expect_identical(res$termination, "time_limit")
  expect_match(res$message,"time limit")
})

test_that("Cartesian half-turn velocity uses the actual pose branch", {
  a <- robot_pose(diag(c(1,-1,-1)))
  b <- robot_pose(diag(c(-1,-1,1)))
  ct <- robot_traj_cartesian(a,b,1,n=3)
  R0 <- as.matrix(pose_rotation(ct$poses[[1]]))
  Rh <- as.matrix(pose_rotation(ct$poses[[2]]))
  actual_axis <- 2*so3_log(Rh%*%t(R0))
  expect_equal(unname(ct$velocity[2,4:6]),1.875*actual_axis,tolerance=1e-8)
})

test_that("disabled position-limit checking cannot invalidate collision bounds", {
  r <- robot_model(joints=list(
    robot_joint("r","revolute","base","arm",axis=c(0,0,1)),
    robot_joint("s","prismatic","arm","tip",axis=c(1,0,0),limits=c(0,.1))))
  r <- robot_attach_geometry(r,"tip",geom_sphere(.05))
  e <- robot_environment(post=geom_sphere(.1,origin=transl(10,0,0)))
  P <- rbind(c(-.2,10),c(.2,10))
  expect_true(robot_check_collision(r,c(0,10),e,self=FALSE)$in_collision)
  expect_false(robot_check_path(r,P,e,self=FALSE,check_limits=FALSE,method="continuous")$valid)
})

test_that("continuous checks treat contact like static checks at both ends", {
  r <- slider_fixture()
  touch <- robot_environment(o = geom_sphere(.5, origin = transl(2, 0, 0)))
  check <- function(P, env, ...) robot_check_path(r, matrix(P, ncol = 1), env,
                                                  self = FALSE,
                                                  method = "continuous", ...)
  expect_false(check(c(1, 0), touch)$valid)
  expect_equal(check(c(1, 0), touch)$q, 1)
  near <- robot_environment(o = geom_sphere(.5, origin = transl(2.25, 0, 0)))
  expect_true(check(c(0, 1), near)$valid)
  expect_false(check(c(0, 1), near, margin = 0.25)$valid)
  expect_true(check(c(0, 1), near, margin = 0.2)$valid)
  for (P in list(c(0, 1), c(1, 0), c(0.5, 1))) {
    static <- robot_check_collision(r, P[length(P)], touch, self = FALSE)$in_collision ||
      robot_check_collision(r, P[1], touch, self = FALSE)$in_collision
    expect_identical(check(P, touch)$valid, !static)
  }
})

test_that("environments name every obstacle uniquely and queries agree", {
  e <- robot_environment(obstacle2 = geom_sphere(.25, origin = transl(20, 0, 0)),
                         geom_sphere(.25))
  expect_equal(names(e$obstacles), c("obstacle2", "obstacle2_1"))
  r <- slider_fixture()
  expect_true(robot_check_collision(r, 0, e, self = FALSE)$in_collision)
  expect_false(robot_check_path(r, matrix(0, 1, 1), e, self = FALSE)$valid)
  expect_false(robot_check_path(r, matrix(c(0, 0.1), 2), e, self = FALSE,
                                method = "continuous")$valid)
  expect_error(robot_environment(list(a = geom_sphere(1), a = geom_sphere(1))),
               "unique")
})

test_that("point and segment hulls give finite results in both orders", {
  p0 <- geom_convex(matrix(0, 1, 3))
  p1 <- geom_convex(matrix(c(1, 0, 0), 1))
  expect_equal(geom_distance(p0, p1)$distance, 1)
  expect_equal(geom_distance(p1, p0)$distance, 1)
  d <- geom_distance(p0, p0, transl(3, 4, 5), transl(3, 4, 5))
  expect_true(d$collision)
  expect_true(all(is.finite(c(d$point_a, d$point_b))))
  seg <- geom_convex(rbind(c(0, 0, 0), c(1, 0, 0), c(2, 0, 0)))
  expect_true(geom_distance(seg, seg)$collision)
  expect_equal(geom_distance(seg, p0, pose_b = transl(1, 1, 0))$distance, 1)
  tri <- geom_convex(rbind(c(0, 0, 0), c(1, 0, 0), c(0, 1, 0)))
  expect_equal(geom_distance(tri, p0, pose_b = transl(0.2, 0.2, 2))$distance, 2)
})

test_that("EPA reports certified bounds whether or not it converges", {
  g <- geom_cylinder(.5, 10)
  d <- geom_distance(g, g)
  expect_false(d$depth_known)
  expect_lte(d$distance_bounds[1], -1 + 1e-9)
  expect_gte(d$distance_bounds[2], -1 - 1e-9)
  expect_equal(d$distance, d$distance_bounds[2])
  b <- geom_distance(geom_box(c(1, 1, 1)), geom_box(c(1, 1, 1)), pose_b = transl(0.5, 0, 0))
  expect_true(b$depth_known)
  expect_equal(b$distance, -0.5, tolerance = 1e-9)
  expect_equal(b$distance_bounds, c(-0.5, -0.5), tolerance = 1e-9)
  s <- geom_distance(g, g, pose_b = transl(0.3, 0, 0))
  expect_true(s$collision)
  expect_lte(s$distance_bounds[1], s$distance_bounds[2])
})

test_that("per-item constraints reject wrong lengths everywhere", {
  expect_error(robot_traj_trapezoidal(c(0, 0, 0), c(1, 1, 1), c(1, 2, 3, 4), 1), "length 1 or 3")
  expect_error(robot_traj_trapezoidal(c(0, 0, 0), c(1, 1, 1), numeric(0), 1))
  tr <- robot_traj_trapezoidal(c(0, 0), c(1, 2), c(1, 2), 1)
  expect_s3_class(tr, "robot_trajectory")
})

test_that("malformed trajectories are rejected before limits are checked", {
  r <- robot_example("planar1")
  base <- robot_traj_polynomial(0, 1, 1)
  for (v in c(NaN, Inf)) {
    tr <- base
    tr$qd[5, 1] <- v
    expect_error(robot_traj_check(r, tr), "Malformed")
  }
  tr <- base
  tr$time <- rev(tr$time)
  expect_error(robot_traj_check(r, tr), "nondecreasing")
  tr <- base
  tr$qdd <- tr$qdd[-1, , drop = FALSE]
  expect_error(robot_traj_check(r, tr), "one row per time")
  expect_true(robot_traj_check(r, base)$ok)
  slow <- robot_set_limits(r, list(joint1 = c(-0.5, 0.5)))
  expect_false(robot_traj_check(slow, base)$ok)
})

test_that("planner termination is reported for every outcome", {
  p <- robot_planner(validity = function(q) q[1] < 0.5, bounds = matrix(c(0, 1), 1),
                     max_iterations = 20)
  expect_identical(robot_plan(p, 0.9, 0.1)$termination, "invalid_start")
  expect_identical(robot_plan(p, 0.1, 0.9)$termination, "invalid_goal")
  wall <- robot_planner(validity = function(q) abs(q[1] - 0.5) > 0.1,
                        bounds = matrix(c(0, 1), 1), max_iterations = 20)
  expect_identical(robot_plan(wall, 0.1, 0.9, seed = 1)$termination, "iteration_limit")
  free <- robot_planner(validity = function(q) TRUE, bounds = matrix(c(0, 1), 1))
  expect_identical(robot_plan(free, 0.1, 0.9, seed = 1)$termination, "success")
  expect_error(robot_planner(validity = function(q) TRUE, bounds = matrix(c(0, 1), 1),
                             goal_bias = 1.5), "probability")
  calls <- 0
  slow <- robot_planner(validity = function(q) { calls <<- calls + 1; Sys.sleep(0.01); TRUE },
                        bounds = matrix(c(0, 1), 1), algorithm = "rrt",
                        resolution = 0.001, time_limit = 0.05)
  res <- robot_plan(slow, 0, 1, seed = 1)
  expect_identical(res$termination, "time_limit")
  expect_lt(calls, 100)
})

test_that("half turns use one rotation branch for poses and derivatives", {
  set.seed(4)
  for (k in 1:10) {
    u <- rnorm(3)
    u <- u / sqrt(sum(u^2))
    G <- so3_exp(rnorm(3))
    a <- robot_pose(G, rnorm(3))
    b <- robot_pose(so3_exp(pi * u) %*% G, rnorm(3))
    ct <- robot_traj_cartesian(a, b, 1, n = 201)
    R <- lapply(ct$poses, function(p) as.matrix(pose_rotation(p)))
    h <- diff(ct$time)[1]
    fd <- t(vapply(2:200, function(i) so3_log(R[[i + 1]] %*% t(R[[i - 1]])) / (2 * h),
                   numeric(3)))
    expect_lt(max(abs(fd - ct$velocity[2:200, 4:6])), 1e-3)
    expect_equal(as.matrix(pose_rotation(ct$poses[[201]])), as.matrix(pose_rotation(b)),
                 tolerance = 1e-12)
  }
})

test_that("segment bounds follow the actual prismatic extension", {
  r <- robot_model(joints = list(
    robot_joint("r", "revolute", "base", "arm", axis = c(0, 0, 1)),
    robot_joint("s", "prismatic", "arm", "tip", axis = c(1, 0, 0), limits = c(-1, 1),
                offset = -2)))
  r <- robot_attach_geometry(r, "tip", geom_sphere(.05))
  e <- robot_environment(post = geom_sphere(.1, origin = transl(-12, 0, 0)))
  P <- rbind(c(-.2, -10), c(.2, -10))
  expect_false(robot_check_path(r, P, e, self = FALSE, check_limits = FALSE,
                                method = "continuous")$valid)
  expect_false(robot_check_path(r, P, e, self = FALSE, method = "continuous")$valid)
  P2 <- rbind(c(-.2, 0.5), c(.2, 0.5))
  expect_true(robot_check_path(r, P2, e, self = FALSE, method = "continuous")$valid)
})
