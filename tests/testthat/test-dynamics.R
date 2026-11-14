ref_planar2_dynamics <- function(q, qd, g) {
  # Closed-form Lagrangian dynamics of two uniform 1 kg, 1 m rods with
  # gravity g along -y (Spong, Hutchinson and Vidyasagar, ch. 7).
  m <- c(1, 1); l1 <- 1; lc <- c(0.5, 0.5); I <- c(1, 1) / 12
  c2 <- cos(q[2]); s2 <- sin(q[2])
  M <- matrix(c(I[1] + I[2] + m[1] * lc[1]^2 + m[2] * (l1^2 + lc[2]^2 + 2 * l1 * lc[2] * c2),
                I[2] + m[2] * (lc[2]^2 + l1 * lc[2] * c2),
                I[2] + m[2] * (lc[2]^2 + l1 * lc[2] * c2),
                I[2] + m[2] * lc[2]^2), 2)
  h <- -m[2] * l1 * lc[2] * s2
  C <- c(h * qd[2] * (2 * qd[1] + qd[2]), -h * qd[1]^2)
  G <- g * c((m[1] * lc[1] + m[2] * l1) * cos(q[1]) + m[2] * lc[2] * cos(q[1] + q[2]),
             m[2] * lc[2] * cos(q[1] + q[2]))
  list(M = M, C = C, G = G)
}

test_that("RNEA, CRBA and ABA match the closed-form two-link dynamics", {
  robot <- robot_example("planar2")
  grav <- c(0, -9.81, 0)
  set.seed(40)
  for (i in 1:15) {
    q <- runif(2, -pi, pi); qd <- rnorm(2); qdd <- rnorm(2)
    ref <- ref_planar2_dynamics(q, qd, 9.81)
    expect_equal(unname(robot_mass_matrix(robot, q)), ref$M, tolerance = 1e-13)
    tau_ref <- as.numeric(ref$M %*% qdd) + ref$C + ref$G
    expect_equal(unname(robot_inverse_dynamics(robot, q, qd, qdd, grav)), tau_ref,
                 tolerance = 1e-12)
    expect_equal(unname(robot_gravity_torque(robot, q, grav)), ref$G, tolerance = 1e-12)
    expect_equal(unname(robot_coriolis(robot, q, qd)), ref$C, tolerance = 1e-12)
    expect_equal(unname(robot_forward_dynamics(robot, q, qd, tau_ref, grav)), qdd,
                 tolerance = 1e-11)
  }
})

test_that("single pendulum dynamics", {
  robot <- robot_example("planar1")
  # Rod about its end: I = m l^2 / 3; torque to hold at angle q is m g l/2 cos q.
  for (q in c(0, 0.5, -1.2)) {
    expect_equal(unname(robot_gravity_torque(robot, q, c(0, -9.81, 0))),
                 9.81 * 0.5 * cos(q), tolerance = 1e-14)
    expect_equal(unname(robot_mass_matrix(robot, q)), matrix(1 / 3), tolerance = 1e-14)
  }
  expect_equal(unname(robot_forward_dynamics(robot, 0, 0, 0, c(0, -9.81, 0))),
               -9.81 * 0.5 / (1 / 3), tolerance = 1e-13)
  # Zero gravity, zero velocity: no torque.
  expect_equal(unname(robot_inverse_dynamics(robot, 0.3, 0, 0, c(0, 0, 0))), 0)
})

test_that("mass matrices are symmetric positive definite and consistent", {
  set.seed(41)
  for (nm in c("planar3", "branched")) {
    robot <- robot_example(nm)
    for (i in 1:5) {
      q <- robot_random_configuration(robot)
      qd <- rnorm(robot_dof(robot))
      M <- robot_mass_matrix(robot, q)
      expect_equal(M, t(M), tolerance = 1e-14)
      expect_gt(min(eigen(M, symmetric = TRUE)$values), 0)
      # Columns of M equal RNEA with unit accelerations and no gravity.
      for (k in seq_len(robot_dof(robot))) {
        e <- replace(numeric(robot_dof(robot)), k, 1)
        expect_equal(unname(robot_inverse_dynamics(robot, q, NULL, e, c(0, 0, 0))),
                     unname(M[, k]), tolerance = 1e-12)
      }
      tau <- rnorm(robot_dof(robot))
      qdd <- robot_forward_dynamics(robot, q, qd, tau)
      expect_equal(unname(robot_inverse_dynamics(robot, q, qd, qdd)), tau,
                   tolerance = 1e-10)
    }
  }
})

test_that("dynamics handle prismatic joints, base poses and external forces", {
  robot <- robot_dh(c(0, 0), c(0, 0), c(0, 0), c(0, 0), c("prismatic", "prismatic"),
                    mass = c(2, 3), com = rbind(c(0, 0, 0), c(0, 0, 0)),
                    inertia = list(diag(0.01, 3), diag(0.01, 3)))
  # Both joints move along z: gravity torque is total supported weight.
  expect_equal(unname(robot_gravity_torque(robot, c(0.1, 0.2))), c(5, 3) * 9.81,
               tolerance = 1e-13)
  # External upward force of 3 g on the last link cancels its weight.
  tau <- robot_inverse_dynamics(robot, c(0.1, 0.2),
                                f_ext = list(link2 = c(0, 0, 3 * 9.81, 0, 0, 0)))
  expect_equal(unname(tau), c(2 * 9.81, 0), tolerance = 1e-13)
  # Rotating the base so that its z axis points along world -z flips gravity.
  flipped <- robot_set_base(robot, trotx(pi))
  expect_equal(unname(robot_gravity_torque(flipped, c(0, 0))), -c(5, 3) * 9.81,
               tolerance = 1e-12)
  massless <- robot_dh(1, 0, 0, 0)
  expect_error(robot_forward_dynamics(massless, 0), "drives no inertia")
})

test_that("unforced conservative simulation conserves energy", {
  robot <- robot_example("planar2")
  g <- c(0, -9.81, 0)
  sim <- robot_simulate(robot, c(0.5, 0.2), c(0, 0), seq(0, 2, by = 0.1), gravity = g)
  E <- apply(cbind(sim$q, sim$qd), 1, function(x) robot_energy(robot, x[1:2], x[3:4], g)[["total"]])
  expect_lt(diff(range(E)), 1e-5)
  expect_s3_class(sim, "robot_trajectory")
  # Gravity compensation holds the arm still.
  hold <- robot_simulate(robot, c(0.4, -0.3), c(0, 0), c(0, 0.5, 1), gravity = g,
                         torque = function(t, q, qd) robot_gravity_torque(robot, q, g))
  expect_equal(unname(hold$q[3, ]), c(0.4, -0.3), tolerance = 1e-6)
  com <- robot_com(robot, c(0, 0))
  expect_equal(com$position, c(1, 0, 0))
  expect_equal(com$mass, 2)
})
