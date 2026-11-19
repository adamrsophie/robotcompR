test_that("a precise coordinate cannot hide error in another", {
  g <- robot_pose_graph(rbind(c(0, 0, 0), c(1, 0, 1)))
  g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0), diag(c(1e26, 1, 1)))
  r <- pose_graph_optimize(g)
  expect_identical(r$status, "converged")
  expect_lt(abs(r$graph$nodes[2, 3]), 1e-9)
  h <- robot_pose_graph(matrix(0, 2, 3))
  h <- pose_graph_add_edge(h, 1, 2, c(0, 1, 0), diag(c(1e14, 1, 1)))
  expect_equal(pose_graph_optimize(h)$graph$nodes[2, 2], 1, tolerance = 1e-9)
  # Tiny information must not underflow the stationarity test.
  k <- robot_pose_graph(matrix(0, 2, 3))
  k <- pose_graph_add_edge(k, 1, 2, c(0, 1, 0), 1e-200 * diag(3))
  expect_equal(pose_graph_optimize(k)$graph$nodes[2, 2], 1, tolerance = 1e-9)
})

test_that("solved graphs far from the origin converge", {
  for (off in c(5e5, 1e7)) {
    g <- robot_pose_graph(rbind(c(off, off, 0), c(off + 1.05, off, 0.02),
                                c(off + 2, off + 0.1, 0)))
    g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0))
    g <- pose_graph_add_edge(g, 2, 3, c(1, 0, 0))
    expect_identical(pose_graph_optimize(g)$status, "converged")
  }
  g <- robot_pose_graph(list(robot_pose(), transl(1e11, 0, 0)))
  g <- pose_graph_add_edge(g, 1, 2, robot_pose())
  r <- pose_graph_optimize(g)
  expect_identical(r$status, "converged")
  expect_lt(abs(as_plain_matrix(r$graph$nodes[[2]])[1, 4]), 1e-6)
})

test_that("degenerate graphs and arguments are handled", {
  r <- pose_graph_optimize(robot_pose_graph(matrix(0, 1, 3)))
  expect_identical(r$status, "converged")
  expect_identical(r$iterations, 0L)
  g <- robot_pose_graph(rbind(c(0, 0, 0), c(1.1, 0, 0)))
  g <- pose_graph_add_edge(g, 1, 2, c(1, 0, 0))
  r0 <- pose_graph_optimize(g, max_iterations = 0)
  expect_identical(r0$iterations, 0L)
  expect_identical(r0$status, "max_iterations")
  expect_error(pose_graph_optimize(g, max_iterations = -1), "max_iterations")
  expect_error(pose_graph_optimize(g, tol = "a"), "tol")
})

test_that("long chains are checked for connectivity in linear time", {
  n <- 2000
  g <- robot_pose_graph(cbind(seq_len(n) - 1, 0, 0))
  for (i in seq_len(n - 1)) g <- pose_graph_add_edge(g, i, i + 1, c(1, 0, 0))
  expect_lt(system.time(r <- pose_graph_optimize(g))[["elapsed"]], 10)
  expect_identical(r$status, "converged")
})
