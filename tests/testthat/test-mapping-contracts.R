test_that("A* validates its inputs and does not leave the map", {
  g <- robot_occupancy_grid(c(0, 3), c(0, 1), 1)
  mask <- rbind(c(FALSE, TRUE, FALSE), rep(FALSE, 3))
  expect_error(grid_astar(g, c(0.5, 0.5), c(2.5, 0.5), occupied = mask), "1 by 3")
  expect_error(grid_probability("x"), "robot_occupancy_grid")
  expect_error(grid_world_to_cell(list(), c(0, 0)), "robot_occupancy_grid")
  n <- 60
  occ <- matrix(FALSE, n, n)
  occ[5:55, 30] <- TRUE
  gg <- robot_occupancy_grid(c(0, n), c(0, n), 1, occupied = occ)
  p <- grid_astar(gg, c(0.5, 0.5), c(n - 0.5, n - 0.5))
  expect_true(all(p >= 0 & p <= n))
  expect_false(any(grid_occupied(gg)[grid_world_to_cell(gg, p)]))
})

test_that("scan updates mark the measured cell, whatever the beam direction", {
  g <- robot_occupancy_grid(c(0, 2), c(0, 1), 1)
  expect_equal(grid_update_scan(g, c(0.5, 0.5, 0), 0, 0.4999995, 2)$logodds[1, ],
               c(0.85, 0))
  expect_equal(grid_update_scan(g, c(1.5, 0.5, pi), 0, 0.5, 2)$logodds[1, ],
               c(0.85, -0.4))
  expect_equal(grid_update_scan(g, c(0.5, 0.5, 0), 0, 0.5, 2)$logodds[1, ],
               c(-0.4, 0.85))
})

test_that("rays are clipped to the map", {
  g <- robot_occupancy_grid(c(0, 1), c(0, 1), 0.01)
  expect_identical(nrow(grid_traverse(g, c(0.505, 0.505), c(100.505, 0.505))), 50L)
  expect_identical(nrow(grid_traverse(g, c(-5, 0.505), c(-4, 0.505))), 0L)
  expect_identical(grid_raycast(g, c(0.505, 0.505, 0), 0, 100), 100)
})

test_that("pure pursuit turns toward a target behind the robot", {
  u <- pure_pursuit(c(0, 0, 0), rbind(c(0, 0), c(-2, 0)), 0.5, 0.5)
  expect_identical(u$v, 0)
  expect_gt(u$omega, 0)
  w <- pure_pursuit(c(0, 0, 0), rbind(c(0, 0), c(-2, -0.1)), 0.5, 0.5)
  expect_lt(w$omega, 0)
})
