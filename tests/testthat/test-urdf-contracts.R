test_that("URDF numbers round-trip exactly in any locale", {
  r <- robot_model(list(robot_link("tip", 1e-9, inertia = diag(1e-18, 3))),
                   robot_joint("j", "continuous", "base", "tip"))
  b <- robot_read_urdf(robot_write_urdf(r))
  expect_identical(robot_mass_matrix(b, 0)[1, 1], 1e-18)
  f <- robot_model(joints = robot_joint("j", "fixed", "a", "b", origin = transl(0.1, 1 / 3, 0)))
  old <- options(OutDec = ",")
  on.exit(options(old))
  x <- robot_write_urdf(f)
  options(old)
  expect_identical(pose_translation_vector(robot_fk(robot_read_urdf(x), numeric(0), "b")),
                   c(0.1, 1 / 3, 0))
  rotor <- robot_model(list(robot_link("rotor", mass = 0, inertia = diag(0.1, 3))),
                       robot_joint("j", "continuous", "base", "rotor"))
  expect_equal(robot_read_urdf(robot_write_urdf(rotor))$links$rotor$inertia, diag(0.1, 3))
})

test_that("URDF versions, quaternion origins and extended limits are handled", {
  x <- paste0('<robot name="r" version="1.2"><link name="base"/><link name="tip"/>',
              '<joint name="j" type="revolute"><parent link="base"/><child link="tip"/>',
              '<origin quat_xyzw="0 0 1 0"/><limit lower="-1" upper="1" acceleration="0"/>',
              '</joint></robot>')
  expect_warning(r <- robot_read_urdf(x), "not modeled")
  expect_equal(unname(diag(as.matrix(robot_fk(r, 0)))), c(-1, -1, 1, 1))
  expect_error(robot_read_urdf(sub("1.2", "2.0", x, fixed = TRUE)), "version")
  expect_error(robot_read_urdf(sub('quat_xyzw', 'rpy="0 0 0" quat_xyzw', x)), "both")
  r <- robot_attach_geometry(r, "tip", geom_capsule(0.05, 0.5))
  txt <- robot_write_urdf(r, fill_limits = c(effort = 1, velocity = 1))
  expect_match(txt, '<robot name="r" version="1.1">', fixed = TRUE)
  plain <- robot_write_urdf(robot_example("planar2"), fill_limits = c(effort = 1, velocity = 1))
  expect_false(grepl("version=\"1.1\"", plain, fixed = TRUE))
})

test_that("a known continuous-joint limit is never dropped", {
  m <- robot_model(joints = robot_joint("j", "continuous", "base", "tip", velocity = 0))
  expect_error(robot_write_urdf(m), "fill_limits")
  b <- robot_read_urdf(robot_write_urdf(m, fill_limits = c(effort = 2, velocity = 9)))
  expect_identical(b$joints[[1]]$velocity, 0)
  expect_identical(b$joints[[1]]$effort, 2)
  expect_null(names(robot_capsule_links(robot_example("planar2"), 0.05)$link_names))
})
