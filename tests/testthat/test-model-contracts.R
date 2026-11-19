test_that('analytic planar IK reaches near-origin nonzero targets', {
  sol <- ik_planar2(1e-6,0,1,1)
  expect_equal(nrow(sol),2L)
  for(i in seq_len(nrow(sol))) {
    p <- c(sum(cos(cumsum(sol[i,]))),sum(sin(cumsum(sol[i,]))))
    expect_lt(sqrt(sum((p-c(1e-6,0))^2)),1e-10)
  }
})
test_that('positive scalar inertia is not rejected because it is small', {
  r <- robot_dh(1e-3,0,0,0,mass=1e-6,com=matrix(c(-5e-4,0,0),1),
                 inertia=list(inertia_rod(1e-6,1e-3)))
  tau <- robot_inverse_dynamics(r,0,0,1,gravity=c(0,0,0))
  expect_equal(unname(robot_forward_dynamics(r,0,0,tau,gravity=c(0,0,0))),1)
})
test_that('small negative principal inertia is not silently accepted', {
  expect_error(robot_link('bad',1,inertia=diag(c(-1e-10,1e-10,1e-10))),
                        'semidefinite|inertia|negative',ignore.case=TRUE)
})
test_that('literal zero URDF limits remain zero', {
  txt <- '<robot name="r"><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1" effort="0" velocity="0"/></joint></robot>'
  r <- robot_read_urdf(txt)
  expect_identical(r$joints$j$velocity,0)
  expect_identical(r$joints$j$effort,0)
  expect_false(robot_traj_check(r,robot_traj_polynomial(0,1,1))$ok)
})
test_that('URDF damping and friction survive export', {
  txt <- '<robot name="r"><link name="a"/><link name="b"/><joint name="j" type="continuous"><parent link="a"/><child link="b"/><dynamics damping="0.2" friction="0.1"/></joint></robot>'
  r <- suppressWarnings(robot_read_urdf(txt))
  b <- suppressWarnings(robot_read_urdf(robot_write_urdf(r)))
  expect_equal(b$joints$j$damping,.2)
  expect_equal(b$joints$j$friction,.1)
})
test_that('SE2 matrix origins are rejected before model construction', {
  expect_error(robot_joint('j','revolute','a','b',origin=diag(3)),
                        'SE\\(3\\)|dimension|4 by 4')
})
test_that('continuous-joint finite limit updates are rejected', {
  r <- robot_model(joints=list(robot_joint('j','continuous','a','b')))
  expect_error(robot_set_limits(r,list(j=c(-.1,.1))),
                        'continuous|Continuous|finite')
})

test_that("planar IK stays exact near the origin at every scale", {
  for (s in c(1e-6, 1, 1e3)) {
    for (tg in list(c(1e-6, 0), c(0, -1e-6), c(1e-6, 1e-6), c(3e-7, -2e-7))) {
      p <- tg * s
      sol <- ik_planar2(p[1], p[2], s, s)
      expect_equal(nrow(sol), 2L)
      for (i in seq_len(nrow(sol))) {
        e <- s * c(sum(cos(cumsum(sol[i, ]))), sum(sin(cumsum(sol[i, ]))))
        expect_lt(sqrt(sum((e - p)^2)), 1e-12 * s)
      }
    }
  }
  expect_equal(unname(ik_planar2(0, 0, 1, 1)[1, ]), c(0, pi))
  expect_equal(nrow(ik_planar2(1e-6, 0, 1, 1.1)), 0L)
  near <- ik_planar2(0.1 + 1e-9, 0, 1, 1.1)
  expect_equal(nrow(near), 2L)
})

test_that("forward dynamics is invariant to physical scale", {
  for (m in c(1e-9, 1e-6, 1, 1e4)) for (l in c(1e-4, 1e-3, 1, 10)) {
    r <- robot_dh(c(l, l), c(0, 0), c(0, 0), c(0, 0), mass = c(m, m),
                  com = rbind(c(-l / 2, 0, 0), c(-l / 2, 0, 0)),
                  inertia = list(inertia_rod(m, l), inertia_rod(m, l)))
    qdd <- c(0.7, -1.3)
    tau <- robot_inverse_dynamics(r, c(0.2, 0.4), c(0.1, -0.3), qdd,
                                  gravity = c(0, -9.81, 0))
    expect_equal(unname(robot_forward_dynamics(r, c(0.2, 0.4), c(0.1, -0.3), tau,
                                        gravity = c(0, -9.81, 0))),
                 qdd, tolerance = 1e-8)
  }
  massless <- robot_dh(c(1, 1), c(0, 0), c(0, 0), c(0, 0), mass = c(1, 0),
                       com = rbind(c(-0.5, 0, 0), c(0, 0, 0)),
                       inertia = list(inertia_rod(1, 1), matrix(0, 3, 3)))
  expect_error(robot_forward_dynamics(massless, c(0, 0), c(0, 0), c(0, 0)),
               "drives no inertia")
  # Two massless sliders along one axis carrying one mass: the first joint
  # moves nothing the second cannot absorb, so the system is singular.
  j1 <- robot_joint("a", "prismatic", "base", "l1", axis = c(1, 0, 0))
  j2 <- robot_joint("b", "prismatic", "l1", "l2", axis = c(1, 0, 0))
  r <- robot_model(links = list(robot_link("l2", 1, inertia = diag(1e-3, 3))),
                   joints = list(j1, j2))
  expect_error(robot_forward_dynamics(r, c(0, 0), c(0, 0), c(0, 0)),
               "drives no inertia")
})

test_that("inertia checks are relative to the tensor at every magnitude", {
  for (s in c(1e-12, 1e-6, 1, 1e6)) {
    expect_error(robot_link("a", 1, inertia = diag(c(-1, 1, 1)) * s),
                 "semidefinite")
    expect_error(robot_link("a", 1, inertia = diag(c(3, 1, 1)) * s),
                 "triangle")
    A <- diag(3) * s
    A[1, 2] <- 0.1 * s
    expect_error(robot_link("a", 1, inertia = A), "symmetric")
    expect_silent(robot_link("a", 1, inertia = diag(c(1, 1, 1.5)) * s))
  }
  expect_silent(robot_link("point", 1, inertia = matrix(0, 3, 3)))
})

test_that("URDF limits distinguish absent, zero and positive values", {
  urdf <- function(lim) sprintf(paste0(
    '<robot name="r"><link name="a"/><link name="b"/><joint name="j" ',
    'type="revolute"><parent link="a"/><child link="b"/>%s</joint></robot>'),
    lim)
  absent <- robot_read_urdf(urdf('<limit lower="-1" upper="1"/>'))
  expect_null(absent$joints$j$velocity)
  expect_null(absent$joints$j$effort)
  pos <- robot_read_urdf(urdf('<limit lower="-1" upper="1" effort="3" velocity="2"/>'))
  expect_identical(pos$joints$j$velocity, 2)
  expect_error(robot_read_urdf(urdf('<limit lower="-1" upper="1" velocity="x" effort="1"/>')))
  expect_error(robot_write_urdf(absent), "fill_limits")
  expect_error(robot_write_urdf(absent, fill_limits = c(1, 2)), "fill_limits")
  back <- robot_read_urdf(robot_write_urdf(absent,
                                           fill_limits = c(effort = 4, velocity = 0.5)))
  expect_identical(back$joints$j$velocity, 0.5)
  expect_identical(back$joints$j$effort, 4)
  zero <- robot_read_urdf(robot_write_urdf(robot_read_urdf(urdf(
    '<limit lower="-1" upper="1" effort="0" velocity="0"/>'))))
  expect_identical(zero$joints$j$velocity, 0)
  cont <- robot_model(joints = list(robot_joint("c", "continuous", "a", "b")))
  expect_false(grepl("<limit", robot_write_urdf(cont)))
})

test_that("damping and friction survive limit edits and export", {
  txt <- paste0('<robot name="r"><link name="a"/><link name="b"/><joint ',
                'name="j" type="revolute"><parent link="a"/><child link="b"/>',
                '<limit lower="-1" upper="1" effort="1" velocity="1"/>',
                '<dynamics damping="0.2" friction="0.1"/></joint></robot>')
  r <- suppressWarnings(robot_read_urdf(txt))
  r2 <- robot_set_limits(r, list(j = c(-0.5, 0.5)))
  expect_equal(r2$joints$j$limits, c(-0.5, 0.5))
  expect_equal(r2$joints$j$damping, 0.2)
  xml <- robot_write_urdf(r2)
  expect_warning(robot_read_urdf(xml), "not modeled")
  b <- suppressWarnings(robot_read_urdf(xml))
  expect_equal(b$joints$j$friction, 0.1)
  expect_equal(b$joints$j$limits, c(-0.5, 0.5))
})

test_that("every pose input route requires SE(3)", {
  expect_error(robot_joint("j", "revolute", "a", "b", post = diag(3)), "4 by 4")
  expect_error(robot_joint("j", "revolute", "a", "b", origin = matrix(0, 4, 3)),
               "4 by 4")
  expect_error(geom_sphere(1, origin = diag(3)))
  expect_error(robot_model(joints = list(robot_joint("j", "revolute", "a", "b")),
                           base = diag(3)))
})
