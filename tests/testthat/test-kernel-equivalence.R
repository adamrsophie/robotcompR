# Faster kernels against the straightforward reference formulations.

test_that("batch particle prediction matches unicycle_step exactly", {
  L <- rbind(a = c(1, 0))
  cases <- list(c(1, 0), c(1, 1e-9), c(0.5, 2e-4 / 0.1), c(0.5, -2.1e-3), c(-1, 3),
                c(0, 0))
  for (n in c(1, 7, 50)) for (u in cases) for (dt in c(0, 1e-6, 0.1)) {
    for (Q in list(diag(c(0.01, 0.02)), matrix(c(0.01, 0.01, 0.01, 0.01), 2),
                   matrix(0, 2, 2))) {
      pf <- robot_particle_filter(n, c(0.1, -0.2, pi - 0.01), diag(c(0.1, 0.1, 0.2)),
                                  L, Q, diag(2), seed = 3)
      set.seed(11)
      out <- pf_predict(pf, u, dt, seed = 5)
      after <- .Random.seed
      noise <- with_seed_internal(5, matrix(stats::rnorm(2 * n), n) %*% psd_factor(Q, "Q"))
      ref <- pf$particles
      for (i in seq_len(n)) {
        ref[i, ] <- unicycle_step(ref[i, ], u[1] + noise[i, 1], u[2] + noise[i, 2], dt)
      }
      expect_identical(unname(out$particles), unname(ref))
      set.seed(11)
      expect_identical(.Random.seed, after)
    }
  }
})

test_that("rank-two EKF-SLAM update matches the dense Joseph form", {
  dense <- function(filter, z) {
    for (k in seq_len(nrow(z))) {
      j <- match(as.character(z$id[k]), filter$ids)
      n <- length(filter$mean)
      li <- 3 + 2 * (j - 1) + 1:2
      x <- filter$mean[1:3]
      d <- filter$mean[li] - x[1:2]
      q <- sum(d^2)
      r <- sqrt(q)
      Hx <- rbind(c(-d[1] / r, -d[2] / r, 0), c(d[2] / q, -d[1] / q, -1))
      H <- matrix(0, 2, n)
      H[, 1:3] <- Hx
      H[, li] <- -Hx[, 1:2]
      innov <- c(z$range[k] - r, wrap_angle_internal(z$bearing[k] -
                   wrap_angle_internal(atan2(d[2], d[1]) - x[3])))
      P <- filter$cov
      S <- H %*% P %*% t(H) + filter$R
      K <- P %*% t(H) %*% solve(S)
      filter$mean <- filter$mean + as.numeric(K %*% innov)
      filter$mean[3] <- wrap_angle_internal(filter$mean[3])
      IKH <- diag(n) - K %*% H
      P <- IKH %*% P %*% t(IKH) + K %*% filter$R %*% t(K)
      filter$cov <- (P + t(P)) / 2
    }
    filter
  }
  set.seed(21)
  for (nl in c(1, 3, 8)) {
    L <- matrix(runif(2 * nl, -5, 5), nl, dimnames = list(letters[seq_len(nl)], NULL))
    f <- robot_ekf_slam(c(0, 0, 3), diag(c(0.01, 0.02, 0.01)), diag(c(0.01, 0.01)),
                        matrix(c(0.02, 0.005, 0.005, 0.01), 2))
    f <- ekf_slam_update(f, landmark_measurements(c(0, 0, 3), L))
    for (step in 1:4) {
      f <- ekf_slam_predict(f, c(0.5, 0.2), 0.5)
      z <- landmark_measurements(c(0.2 * step, 0.1, 3 + 0.1 * step), L)
      z$range <- z$range + rnorm(nl, 0, 0.05)
      a <- ekf_slam_update(f, z)
      b <- dense(f, z)
      expect_equal(a$mean, b$mean, tolerance = 1e-12)
      expect_equal(a$cov, b$cov, tolerance = 1e-12)
      expect_gte(min(eigen(a$cov, symmetric = TRUE, only.values = TRUE)$values), 0)
      f <- a
    }
  }
  # Empty scans leave nis untouched; coincident landmarks are skipped.
  g <- f
  g$nis <- 7
  expect_identical(ekf_slam_update(g, data.frame(id = character(0), range = numeric(0),
                                                 bearing = numeric(0)))$nis, 7)
})

test_that("cached dynamics constants match fresh computation and follow edits", {
  for (nm in c("planar2", "planar3", "puma560", "branched")) {
    r <- robot_example(nm)
    bare <- r
    bare$spatial_inertia <- NULL
    bare$motion_subspace <- NULL
    q <- robot_random_configuration(r, seed = 2)
    qd <- seq_along(q) / 10
    expect_identical(robot_mass_matrix(r, q), robot_mass_matrix(bare, q))
    expect_identical(robot_inverse_dynamics(r, q, qd, -qd),
                     robot_inverse_dynamics(bare, q, qd, -qd))
    # The puma560 example carries no wrist inertia, so it has no forward
    # dynamics; the others exercise the articulated-body pass.
    if (nm != "puma560") {
      tau <- robot_inverse_dynamics(r, q, qd, -qd)
      expect_identical(robot_forward_dynamics(r, q, qd, tau),
                       robot_forward_dynamics(bare, q, qd, tau))
    }
  }
  r <- robot_example("planar2")
  heavy <- robot_set_inertia(r, "link2", mass = 5, com = c(-0.5, 0, 0),
                              inertia = inertia_rod(5, 1))
  expect_false(isTRUE(all.equal(robot_mass_matrix(heavy, c(0.1, 0.2)),
                                robot_mass_matrix(r, c(0.1, 0.2)))))
  fresh <- heavy
  fresh$spatial_inertia <- NULL
  expect_identical(robot_mass_matrix(heavy, c(0.1, 0.2)),
                   robot_mass_matrix(fresh, c(0.1, 0.2)))
})
