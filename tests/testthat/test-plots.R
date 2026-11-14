test_that("plots run on a headless device", {
  pdf(NULL)
  on.exit(grDevices::dev.off())
  robot <- robot_example("planar2")
  expect_silent(robot_plot(robot, c(pi / 4, pi / 4)))
  expect_silent(robot_plot(robot, c(0, 1), add = TRUE, col = "grey"))
  puma <- robot_example("puma560")
  O <- robot_plot(puma, rep(0.2, 6), projection = "3d", frames = TRUE)
  expect_equal(ncol(O), 2L)
  robot_plot(puma, rep(0.2, 6), projection = "xz")
  P <- robot_plot_workspace(robot, n = 50, seed = 1,
                            color_by = function(q) abs(sin(q[2])))
  expect_equal(dim(P), c(50, 3))
})
