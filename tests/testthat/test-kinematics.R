test_that("two-link reference problem: q = (pi/2, pi/2) gives (-1, 1)", {
  robot <- robot_dh(a = c(1, 1), alpha = c(0, 0), d = c(0, 0),
                    theta = c(0, 0), joint_type = c("revolute", "revolute"))
  p <- pose_translation(robot_fk(robot, c(pi / 2, pi / 2)))
  expect_equal(p, c(-1, 1, 0), tolerance = 1e-15)
  expect_equal(pose_translation(robot_fk(robot, c(0, 0))), c(2, 0, 0))
})

test_that("planar FK matches the closed form for 1, 2 and 3 links", {
  set.seed(10)
  for (nm in c("planar1", "planar2", "planar3")) {
    robot <- robot_example(nm)
    l <- robot$dh$a
    for (i in 1:25) {
      q <- runif(length(l), -pi, pi)
      p <- pose_translation(robot_fk(robot, q))
      expect_equal(p[1:2], ref_planar_fk(l, q), tolerance = 1e-13)
      expect_equal(p[3], 0)
      R <- as.matrix(pose_rotation(robot_fk(robot, q)))
      expect_equal(R, rotz(sum(q)), tolerance = 1e-13)
    }
    # Joint limits are evaluated, not rejected.
    lim <- robot_limits(robot)
    expect_equal(pose_translation(robot_fk(robot, lim[, 2]))[1:2],
                 ref_planar_fk(l, lim[, 2]), tolerance = 1e-13)
  }
})

test_that("standard and modified DH chains match direct matrix products", {
  set.seed(11)
  a <- c(0.1, 0.4, 0.02, 0)
  alpha <- c(pi / 2, 0, -pi / 2, pi / 3)
  d <- c(0.3, 0, 0.15, 0.4)
  theta <- c(0.1, -0.2, 0, 0.5)
  types <- c("revolute", "prismatic", "revolute", "revolute")
  for (conv in c("standard", "modified")) {
    robot <- robot_dh(a, alpha, d, theta, types, convention = conv)
    f <- if (conv == "standard") ref_dh_standard else ref_dh_modified
    for (i in 1:10) {
      q <- runif(4, -1, 1)
      M <- diag(4)
      for (k in 1:4) {
        th <- theta[k] + if (types[k] == "revolute") q[k] else 0
        dk <- d[k] + if (types[k] == "prismatic") q[k] else 0
        M <- M %*% f(a[k], alpha[k], dk, th)
        expect_equal(as.matrix(robot_fk(robot, q, paste0("link", k))), M,
                     tolerance = 1e-13)
      }
    }
  }
  expect_equal(as.matrix(dh_transform(1, 0.2, 0.3, 0.4)),
               ref_dh_standard(1, 0.2, 0.3, 0.4), tolerance = 1e-15)
  expect_equal(as.matrix(dh_transform(1, 0.2, 0.3, 0.4, "modified")),
               ref_dh_modified(1, 0.2, 0.3, 0.4), tolerance = 1e-15)
})

test_that("PUMA 560 zero pose matches the published value", {
  robot <- robot_example("puma560")
  T <- as.matrix(robot_fk(robot, rep(0, 6)))
  expect_equal(T[1:3, 4], c(0.4521, -0.15005, 0.4318), tolerance = 1e-12)
  expect_equal(T[1:3, 1:3], diag(3), tolerance = 1e-14)
})

test_that("base and tool frames compose correctly", {
  base <- robot_pose(rotz(pi / 2), c(1, 0, 0.5))
  tool <- transl(0.2, 0, 0)
  robot <- robot_dh(a = c(1, 1), alpha = c(0, 0), d = c(0, 0),
                    theta = c(0, 0), base = base, tool = tool)
  q <- c(0.3, -0.7)
  inner <- ref_dh_standard(1, 0, 0, q[1]) %*% ref_dh_standard(1, 0, 0, q[2])
  expect_equal(as.matrix(robot_fk(robot, q)),
               as.matrix(base) %*% inner %*% as.matrix(tool), tolerance = 1e-14)
  expect_equal(robot$end_effector, "tool")
  all <- robot_fk(robot, q, "all")
  expect_named(all, c("base", "link1", "link2", "tool"))
  expect_equal(as.matrix(all$base), as.matrix(base))
})

test_that("fixed joints add frames but not degrees of freedom", {
  links <- list(robot_link("base"), robot_link("a"), robot_link("mid"),
                robot_link("b"))
  joints <- list(
    robot_joint("j1", "revolute", "base", "a", axis = c(0, 0, 1)),
    robot_joint("fix", "fixed", "a", "mid", origin = transl(1, 0, 0)),
    robot_joint("j2", "revolute", "mid", "b", origin = transl(0.5, 0, 0),
                axis = c(0, 0, 1))
  )
  robot <- robot_model(links, joints)
  expect_equal(robot_dof(robot), 2L)
  expect_equal(nrow(robot_joints(robot)), 3L)
  expect_equal(robot_joint_names(robot), c("j1", "j2"))
  q <- c(0.4, 0.9)
  expect_equal(pose_translation(robot_fk(robot, q, "b"))[1:2],
               c(1.5 * cos(q[1]), 1.5 * sin(q[1])), tolerance = 1e-14)
  J <- robot_jacobian(robot, q, "b")
  expect_equal(unname(J[1:2, 1]), c(-1.5 * sin(q[1]), 1.5 * cos(q[1])), tolerance = 1e-14)
  expect_equal(unname(J[, 2]), c(0, 0, 0, 0, 0, 1))
})

test_that("branched trees compute each branch independently", {
  robot <- robot_example("branched")
  expect_equal(robot_dof(robot), 4L)
  q <- c(0.3, -0.5, 0.4, -0.2)
  pl <- pose_translation(robot_fk(robot, q, "finger_left"))
  pr <- pose_translation(robot_fk(robot, q, "finger_right"))
  a <- q[1]
  b <- q[1] + q[2]
  palm <- c(0.5 * cos(a) + 0.1 * cos(b), 0.5 * sin(a) + 0.1 * sin(b))
  expect_equal(pl[1:2], palm + 0.03 * c(-sin(b), cos(b)), tolerance = 1e-14)
  expect_equal(pr[1:2], palm - 0.03 * c(-sin(b), cos(b)), tolerance = 1e-14)
  # Each finger's Jacobian ignores the other finger's joint.
  expect_equal(unname(robot_jacobian(robot, q, "finger_left")[, 4]), rep(0, 6))
  expect_equal(unname(robot_jacobian(robot, q, "finger_right")[, 3]), rep(0, 6))
})

test_that("invalid models are rejected", {
  l <- list(robot_link("a"), robot_link("b"), robot_link("c"))
  expect_error(robot_model(l, list(robot_joint("j", "revolute", "a", "b"),
                                   robot_joint("j", "revolute", "b", "c"))),
               "Duplicate joint")
  expect_error(robot_model(list(robot_link("a"), robot_link("a")),
                           list(robot_joint("j", "revolute", "a", "b"))),
               "Duplicate link")
  expect_error(robot_model(l, list(robot_joint("j1", "revolute", "a", "b"),
                                   robot_joint("j2", "revolute", "c", "b"))),
               "more than one parent")
  expect_error(robot_model(l, list(robot_joint("j1", "revolute", "a", "b"),
                                   robot_joint("j2", "revolute", "b", "a"))),
               "more than one parent|cycle")
  expect_error(robot_model(c(l, list(robot_link("d"))),
                           list(robot_joint("j1", "revolute", "a", "b"),
                                robot_joint("j2", "revolute", "c", "d"))),
               "disconnected")
  expect_error(robot_joint("j", "revolute", "a", "a"), "itself")
  expect_error(robot_joint("j", "revolute", "a", "b", limits = c(1, -1)),
               "lower limit")
  expect_error(robot_joint("j", "continuous", "a", "b", limits = c(-1, 1)),
               "Continuous")
  expect_error(robot_joint("j", "revolute", "a", "b", axis = c(0, 0, 0)),
               "nonzero")
  expect_error(robot_link("x", mass = -1), "nonnegative")
  expect_error(robot_link("x", inertia = diag(c(1, 1, 3))), "triangle")
  expect_error(robot_link("x", inertia = diag(c(-1, 1, 1))), "semidefinite")
  expect_error(robot_fk(robot_example("planar2"), 1), "length 2")
  expect_error(robot_fk(robot_example("planar2"), c(0, 0), "nope"), "Unknown frame")
})

test_that("continuous joints are unbounded", {
  j <- robot_joint("w", "continuous", "a", "b")
  expect_equal(j$limits, c(-Inf, Inf))
  robot <- robot_model(list(), list(j))
  expect_true(robot_within_limits(robot, 100))
})

test_that("limits utilities and random configurations", {
  robot <- robot_example("planar2")
  expect_false(robot_within_limits(robot, c(0, 4)))
  expect_equal(unname(robot_clamp(robot, c(0, 4))), c(0, pi))
  old <- if (exists(".Random.seed", globalenv())) get(".Random.seed", globalenv())
  Q1 <- robot_random_configuration(robot, 5, seed = 42)
  Q2 <- robot_random_configuration(robot, 5, seed = 42)
  expect_identical(Q1, Q2)
  now <- if (exists(".Random.seed", globalenv())) get(".Random.seed", globalenv())
  expect_identical(old, now)
  expect_true(all(Q1 >= -pi & Q1 <= pi))
  P <- robot_fk_positions(robot, Q1)
  expect_equal(dim(P), c(5, 3))
  expect_equal(unname(P[2, 1:2]), ref_planar_fk(c(1, 1), Q1[2, ]), tolerance = 1e-14)
})

test_that("geometric Jacobian of the two-link arm matches the closed form", {
  robot <- robot_example("planar2")
  set.seed(12)
  for (i in 1:20) {
    q <- runif(2, -pi, pi)
    J <- robot_jacobian(robot, q)
    expect_equal(unname(J[1:2, ]), ref_planar2_jacobian(1, 1, q), tolerance = 1e-14)
    expect_equal(unname(J[3:5, ]), matrix(0, 3, 2))
    expect_equal(unname(J[6, ]), c(1, 1))
  }
  q <- c(0.2, 0.4)
  expect_equal(unname(robot_jacobian(robot, q)[1:2, ]),
               ref_planar2_jacobian(1, 1, q), tolerance = 1e-14)
})

test_that("Jacobians agree with Lie-group finite differences", {
  set.seed(13)
  for (nm in c("planar3", "puma560", "branched")) {
    robot <- robot_example(nm)
    for (i in 1:5) {
      q <- robot_random_configuration(robot)
      expect_lt(max(abs(robot_jacobian(robot, q) -
                          robot_jacobian_numeric(robot, q))), 1e-7)
    }
  }
  # Prismatic joints in a DH chain.
  robot <- robot_dh(c(0.1, 0.4, 0), c(pi / 2, 0, -pi / 2), c(0.3, 0, 0.15),
                    c(0, 0, 0.3), c("revolute", "prismatic", "revolute"))
  q <- c(0.3, 0.2, -0.5)
  expect_lt(max(abs(robot_jacobian(robot, q) - robot_jacobian_numeric(robot, q))),
            1e-7)
})

test_that("Jacobian reference frames are related by rotations and shifts", {
  robot <- robot_example("puma560")
  q <- c(0.1, 0.5, -0.3, 0.7, 0.2, -0.4)
  Jw <- robot_jacobian(robot, q, reference = "world")
  Jl <- robot_jacobian(robot, q, reference = "local")
  Js <- robot_jacobian(robot, q, reference = "space")
  T <- as.matrix(robot_fk(robot, q))
  R <- T[1:3, 1:3]
  p <- T[1:3, 4]
  expect_equal(unname(Jl[1:3, ]), unname(t(R) %*% Jw[1:3, ]), tolerance = 1e-13)
  expect_equal(unname(Jl[4:6, ]), unname(t(R) %*% Jw[4:6, ]), tolerance = 1e-13)
  # Space twist: v_O = v_p - w x p.
  vO <- Jw[1:3, ] - apply(Jw[4:6, ], 2, function(w)
    c(w[2] * p[3] - w[3] * p[2], w[3] * p[1] - w[1] * p[3],
      w[1] * p[2] - w[2] * p[1]))
  expect_equal(unname(Js[1:3, ]), unname(vO), tolerance = 1e-13)
  # Body Jacobian equals Ad(T^-1) times the space Jacobian.
  expect_equal(unname(Jl), unname(se3_adjoint(solve(T)) %*% Js), tolerance = 1e-12)
})

test_that("point Jacobians and analytic Jacobians", {
  robot <- robot_example("planar2")
  q <- c(0.3, 0.5)
  Jp <- robot_jacobian(robot, q, point = c(-0.5, 0, 0))
  # Midpoint of link 2.
  ref <- rbind(c(-sin(q[1]) - 0.5 * sin(sum(q)), -0.5 * sin(sum(q))),
               c(cos(q[1]) + 0.5 * cos(sum(q)), 0.5 * cos(sum(q))))
  expect_equal(unname(Jp[1:2, ]), ref, tolerance = 1e-14)

  puma <- robot_example("puma560")
  q <- c(0.1, 0.5, -0.3, 0.7, 0.2, -0.4)
  h <- 1e-6
  for (rep in c("rpy", "rotvec")) {
    Ja <- robot_jacobian_analytic(puma, q, representation = rep)
    f <- function(qq) {
      T <- robot_fk(puma, qq)
      o <- if (rep == "rpy") rev(as_euler(T, "ZYX")) else as_rotvec(T)
      c(pose_translation(T), o)
    }
    Jn <- vapply(1:6, function(i) {
      dq <- replace(numeric(6), i, h)
      (f(q + dq) - f(q - dq)) / (2 * h)
    }, numeric(6))
    expect_lt(max(abs(unname(Ja) - Jn)), 1e-7)
  }
})

test_that("manipulability measures and singularities", {
  robot <- robot_example("planar2")
  q <- c(0.3, 1.1)
  m <- robot_manipulability(robot, q, rows = 1:2)
  # For the two-link arm, det J = l1 l2 sin(q2).
  expect_equal(m$yoshikawa, abs(sin(q[2])), tolerance = 1e-14)
  expect_false(m$singular)
  expect_equal(m$rank, 2L)
  ms <- robot_manipulability(robot, c(0.3, 0), rows = 1:2)
  expect_true(ms$singular)
  expect_equal(ms$condition, Inf)
  expect_lt(ms$yoshikawa, 1e-14)
  expect_true(robot_is_singular(robot, c(0.3, pi), rows = 1:2))
  expect_warning(robot_is_singular(robot, c(0.3, 0), rows = 1:2, warn = TRUE),
                 "singular")
  mt <- robot_manipulability(robot_example("puma560"),
                             c(0, 0.5, -0.3, 0, 0.4, 0))
  expect_equal(length(mt$singular_values), 3L)
  # Wrist singularity of the PUMA (q5 = 0) shows up in the full Jacobian.
  expect_true(robot_manipulability(robot_example("puma560"),
                                   c(0, 0.5, -0.3, 0, 0, 0),
                                   measure = "full")$singular)
})
