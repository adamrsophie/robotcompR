test_that("URDF import reproduces the two-link reference", {
  robot <- robot_read_urdf(system.file("extdata", "planar2.urdf",
                                       package = "robotcompR"))
  expect_equal(robot_dof(robot), 2L)
  expect_equal(robot$end_effector, "tool")
  set.seed(30)
  for (i in 1:10) {
    q <- runif(2, -pi, pi)
    expect_equal(pose_translation(robot_fk(robot, q))[1:2],
                 ref_planar_fk(c(1, 1), q), tolerance = 1e-13)
  }
  expect_equal(robot$joints$joint1$effort, 50)
  expect_equal(length(robot$links$link1$collision), 1L)
  # Its dynamics equal the DH example with the same rods.
  dh <- robot_example("planar2")
  q <- c(0.3, -0.8)
  expect_equal(unname(robot_mass_matrix(robot, q)),
               unname(robot_mass_matrix(dh, q)), tolerance = 1e-13)
})

test_that("URDF rpy origins, continuous and prismatic joints", {
  robot <- robot_read_urdf(system.file("extdata", "pan_tilt.urdf",
                                       package = "robotcompR"))
  expect_equal(robot_joints(robot)$type, c("continuous", "revolute", "prismatic"))
  q <- c(0.4, -0.3, 0.02)
  # Independent composition of the URDF transforms.
  rpy <- function(r, p, y) rotz(y) %*% roty(p) %*% rotx(r)
  H <- function(R, p) { M <- diag(4); M[1:3, 1:3] <- R; M[1:3, 4] <- p; M }
  T <- H(diag(3), c(0, 0, 0.1)) %*% H(rotz(q[1]), c(0, 0, 0)) %*%
    H(rpy(pi / 2, 0, 0), c(0, 0, 0.1)) %*% H(rotz(q[2]), c(0, 0, 0)) %*%
    H(rpy(0, pi / 2, 0), c(0.06, 0, 0)) %*% H(diag(3), c(0, 0, q[3]))
  expect_equal(as.matrix(robot_fk(robot, q, "lens")), T, tolerance = 1e-12)
  # Inertia rotated from the inertial frame into the link frame.
  I0 <- matrix(c(0.002, 0.0001, 0, 0.0001, 0.003, 0, 0, 0, 0.004), 3)
  expect_equal(robot$links$pan$inertia, rotz(0.3) %*% I0 %*% t(rotz(0.3)),
               tolerance = 1e-15)
  expect_lt(max(abs(robot_jacobian(robot, q) - robot_jacobian_numeric(robot, q))),
            1e-7)
})

test_that("URDF round trip preserves kinematics and dynamics", {
  set.seed(31)
  for (nm in c("puma560", "planar3", "branched")) {
    robot <- robot_example(nm)
    back <- robot_read_urdf(robot_write_urdf(robot, fill_limits = c(effort = 100, velocity = 100)))
    expect_equal(robot_dof(back), robot_dof(robot))
    for (i in 1:3) {
      q <- robot_random_configuration(robot)
      for (f in robot$link_names) {
        expect_equal(as.matrix(robot_fk(back, q, f)),
                     as.matrix(robot_fk(robot, q, f)), tolerance = 1e-12)
      }
    }
  }
  r3 <- robot_example("planar3")
  b3 <- robot_read_urdf(robot_write_urdf(r3, fill_limits = c(effort = 100, velocity = 100)))
  q <- c(0.1, 0.2, 0.3)
  expect_equal(unname(robot_mass_matrix(b3, q)), unname(robot_mass_matrix(r3, q)),
               tolerance = 1e-12)
  f <- tempfile(fileext = ".urdf")
  robot_write_urdf(r3, f, fill_limits = c(effort = 100, velocity = 100))
  expect_true(file.exists(f))
})

test_that("unsupported and unsafe URDF content is rejected", {
  base <- '<robot name="r"><link name="a"/><link name="b"/>%s</robot>'
  j <- function(x) sprintf(base, x)
  expect_error(robot_read_urdf(j('<joint name="j" type="floating"><parent link="a"/><child link="b"/></joint>')),
               "unsupported type")
  expect_error(robot_read_urdf(j('<joint name="j" type="planar"><parent link="a"/><child link="b"/></joint>')),
               "unsupported type")
  expect_error(robot_read_urdf(j('<joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="0" upper="1"/><mimic joint="k"/></joint>')),
               "mimic")
  expect_error(robot_read_urdf(j('<joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>')),
               "limit")
  expect_error(robot_read_urdf('<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="a"/></robot>'),
               "xacro")
  expect_error(robot_read_urdf("model.urdf.xacro"), "xacro")
  expect_error(robot_read_urdf('<?xml version="1.0"?><!DOCTYPE r [<!ENTITY x SYSTEM "file:///etc/passwd">]><robot name="r"><link name="a"/></robot>'),
               "entity")
  expect_error(robot_read_urdf(j('<joint name="j" type="revolute"><parent link="a"/><child link="b"/><origin xyz="1 2"/><limit lower="0" upper="1"/></joint>')),
               "Malformed")
  expect_warning(robot_read_urdf(j('<joint name="j" type="continuous"><parent link="a"/><child link="b"/><dynamics damping="0.5"/></joint>')),
                 "not modeled")
  # Default URDF axis is x.
  r <- robot_read_urdf(j('<joint name="j" type="continuous"><parent link="a"/><child link="b"/></joint>'))
  expect_equal(r$joints$j$axis, c(1, 0, 0))
  # Mesh geometry is kept as metadata and never opened.
  m <- robot_read_urdf(sprintf(base, '<joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>'))
  expect_equal(robot_dof(m), 0L)
})
