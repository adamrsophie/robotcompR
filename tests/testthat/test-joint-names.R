# Every public entry point that takes joint arrays gives a fully named,
# permuted input the same meaning as the unnamed input in joint order.

perm <- function(x, nms) {
  names(x) <- nms
  x[rev(seq_along(x))]
}

test_that("named joint vectors mean the same joints in every API", {
  r <- robot_capsule_links(robot_example("planar2"), 0.05)
  jn <- robot_joint_names(r)
  q <- c(0.3, -0.8)
  qd <- c(0.5, 0.2)
  qdd <- c(-1, 0.4)
  tau <- c(2, -1)
  g <- c(0, -9.81, 0)
  same <- function(f) expect_equal(f(perm(q, jn), perm(qd, jn), perm(qdd, jn), perm(tau, jn)),
                                   f(q, qd, qdd, tau))
  same(function(q, qd, qdd, tau) robot_fk(r, q))
  same(function(q, qd, qdd, tau) robot_jacobian(r, q))
  same(function(q, qd, qdd, tau) robot_jacobian_analytic(r, q))
  same(function(q, qd, qdd, tau) robot_jacobian_numeric(r, q))
  same(function(q, qd, qdd, tau) robot_manipulability(r, q))
  same(function(q, qd, qdd, tau) robot_is_singular(r, q))
  same(function(q, qd, qdd, tau) robot_mass_matrix(r, q))
  same(function(q, qd, qdd, tau) robot_coriolis(r, q, qd))
  same(function(q, qd, qdd, tau) robot_bias_forces(r, q, qd, g))
  same(function(q, qd, qdd, tau) robot_gravity_torque(r, q, g))
  same(function(q, qd, qdd, tau) robot_energy(r, q, qd, g))
  same(function(q, qd, qdd, tau) robot_com(r, q))
  same(function(q, qd, qdd, tau) robot_inverse_dynamics(r, q, qd, qdd, g))
  same(function(q, qd, qdd, tau) robot_forward_dynamics(r, q, qd, tau, g))
  same(function(q, qd, qdd, tau) robot_within_limits(r, q))
  same(function(q, qd, qdd, tau) robot_clamp(r, q))
  same(function(q, qd, qdd, tau) robot_check_collision(r, q))
  same(function(q, qd, qdd, tau) robot_ik(r, robot_fk(r, c(0.4, 0.4)), initial = q)$q)
  same(function(q, qd, qdd, tau)
    robot_simulate(r, q, qd, c(0, 0.05), torque = tau, gravity = g)$q)
  same(function(q, qd, qdd, tau)
    robot_simulate(r, q, qd, c(0, 0.05), gravity = g,
                   torque = function(t, x, v) tau)$q)
})

test_that("named joint matrices mean the same joints in every API", {
  r <- robot_capsule_links(robot_example("planar2"), 0.05)
  jn <- robot_joint_names(r)
  Q <- rbind(c(0.3, -0.8), c(1, 0.2))
  Qn <- Q[, 2:1]
  colnames(Qn) <- rev(jn)
  expect_equal(robot_fk_positions(r, Qn), robot_fk_positions(r, Q))
  env <- robot_environment(post = geom_sphere(0.1, origin = transl(0, 1.5, 0)))
  for (m in c("discrete", "continuous")) {
    expect_equal(robot_check_path(r, Qn, env, method = m),
                 robot_check_path(r, Q, env, method = m))
  }
  tr <- robot_traj_polynomial(Q[1, ], Q[2, ], 1, n = 5)
  trn <- tr
  for (f in c("q", "qd", "qdd")) {
    trn[[f]] <- tr[[f]][, 2:1]
    colnames(trn[[f]]) <- rev(jn)
  }
  expect_equal(robot_traj_check(r, trn), robot_traj_check(r, tr))
  p <- robot_planner(r, environment = env)
  expect_equal(robot_plan(p, perm(Q[1, ], jn), perm(Q[2, ], jn), seed = 3)$path,
               robot_plan(p, Q[1, ], Q[2, ], seed = 3)$path)
})

test_that("partial, duplicated or unknown joint names are rejected", {
  r <- robot_example("planar2")
  expect_error(robot_fk(r, c(joint1 = 0, nope = 1)), "joint names")
  expect_error(robot_fk(r, c(joint1 = 0, joint1 = 1)), "joint names")
  expect_error(robot_fk(r, c(a = 0, b = 1)), "joint names")
  expect_error(robot_fk_positions(r, matrix(0, 1, 2, dimnames = list(NULL, c("joint1", "x")))),
               "joint names")
  expect_error(robot_simulate(r, c(0, 0), times = c(0, 0.1), torque = c(joint1 = 1, x = 0)),
               "joint names")
  expect_error(robot_traj_polynomial(c(a = 0, b = 0), c(a = 1, c = 1), 1), "names of `q0`")
  tr <- robot_traj_polynomial(c(a = 0, b = 1), c(b = 3, a = 2), 1, n = 3)
  expect_equal(unname(tr$q[3, ]), c(2, 3))
})

test_that("a simulation that stops early is an error that keeps the solver's reason", {
  r <- robot_model(robot_link("tip", 1), robot_joint("j", "prismatic", "base", "tip"))
  e <- expect_error(robot_simulate(r, 0, 1, times = c(0, 1), gravity = numeric(3),
                                   torque = function(t, q, qd) qd, maxsteps = 1),
                    "stopped at t = ")
  expect_match(conditionMessage(e), "Solver: .*maxsteps")
  ok <- robot_simulate(r, 0, 1, times = c(0, 1), gravity = numeric(3))
  expect_equal(unname(ok$q[2, ]), 1)
})
