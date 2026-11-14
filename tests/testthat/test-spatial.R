test_that("elementary rotations match the Rodrigues formula", {
  for (th in c(-2, -0.3, 0, 0.7, 3)) {
    expect_equal(rotx(th), ref_rot_axis_angle(c(1, 0, 0), th), tolerance = 1e-14)
    expect_equal(roty(th), ref_rot_axis_angle(c(0, 1, 0), th), tolerance = 1e-14)
    expect_equal(rotz(th), ref_rot_axis_angle(c(0, 0, 1), th), tolerance = 1e-14)
  }
  expect_equal(rotz(pi / 2) %*% c(1, 0, 0), matrix(c(0, 1, 0)), tolerance = 1e-15)
})

test_that("rotations are orthonormal with unit determinant", {
  set.seed(1)
  for (i in 1:20) {
    R <- robot_rotation(axis = rnorm(3), angle = runif(1, -pi, pi))
    M <- as.matrix(R)
    expect_equal(crossprod(M), diag(3), tolerance = 1e-14)
    expect_equal(det(M), 1, tolerance = 1e-14)
  }
  expect_error(robot_rotation(diag(c(1, 1, -1))), "determinant")
  expect_error(robot_rotation(matrix(1, 3, 3)), "orthonormal")
  expect_true(is_rotation_matrix(rotx(1)))
  expect_false(is_rotation_matrix(diag(c(1, -1, 1))))
})

test_that("orthonormalize repairs drift", {
  set.seed(2)
  R <- random_rotation() + matrix(rnorm(9, sd = 1e-5), 3)
  expect_false(is_rotation_matrix(R, tol = 1e-9))
  expect_true(is_rotation_matrix(orthonormalize(R), tol = 1e-12))
})

test_that("Euler angle round trips hold for all 12 sequences", {
  set.seed(3)
  seqs <- c("ZYX", "XYZ", "ZYZ", "ZXZ", "XYX", "YZX", "XZY", "YXZ", "ZXY",
            "XZX", "YXY", "YZY")
  for (s in seqs) {
    ax <- strsplit(s, "")[[1]]
    el <- list(X = rotx, Y = roty, Z = rotz)
    for (i in 1:30) {
      e <- runif(3, -pi, pi)
      Rref <- el[[ax[1]]](e[1]) %*% el[[ax[2]]](e[2]) %*% el[[ax[3]]](e[3])
      R <- robot_rotation(euler = e, sequence = s)
      expect_equal(as.matrix(R), Rref, tolerance = 1e-13)
      e2 <- as_euler(R, s)
      R2 <- el[[ax[1]]](e2[1]) %*% el[[ax[2]]](e2[2]) %*% el[[ax[3]]](e2[3])
      expect_equal(R2, Rref, tolerance = 1e-12)
    }
    # Gimbal lock: middle angle at its singular value.
    mid <- if (ax[1] == ax[3]) 0 else pi / 2
    Rref <- el[[ax[1]]](0.4) %*% el[[ax[2]]](mid) %*% el[[ax[3]]](-0.3)
    e2 <- as_euler(Rref, s)
    R2 <- el[[ax[1]]](e2[1]) %*% el[[ax[2]]](e2[2]) %*% el[[ax[3]]](e2[3])
    expect_equal(R2, Rref, tolerance = 1e-9)
  }
  expect_equal(as_euler(rotz(0.3) %*% roty(-0.2) %*% rotx(0.1), "ZYX"),
               c(0.3, -0.2, 0.1), tolerance = 1e-14)
})

test_that("axis-angle and rotation vectors are consistent", {
  R <- robot_rotation(axis = c(1, 1, 0), angle = 2)
  aa <- as_axis_angle(R)
  expect_equal(aa$angle, 2, tolerance = 1e-14)
  expect_equal(aa$axis, c(1, 1, 0) / sqrt(2), tolerance = 1e-14)
  expect_equal(as_rotvec(R), 2 * c(1, 1, 0) / sqrt(2), tolerance = 1e-14)
  expect_equal(as_axis_angle(robot_rotation(diag(3)))$angle, 0)
})

test_that("SO(3) exp and log are inverse, including near 0 and pi", {
  u <- c(1, 2, -2) / 3
  for (th in c(0, 1e-12, 1e-8, 1e-5, 1e-3, 0.5, 2, pi - 1e-2, pi - 1e-6,
               pi - 1e-10)) {
    w <- th * u
    R <- so3_exp(w)
    # The textbook formula loses digits for tiny angles; compare absolutely.
    expect_lt(max(abs(R - ref_rot_axis_angle(u, th))), 1e-14)
    expect_equal(so3_log(R), w, tolerance = 1e-9)
  }
  # Exactly pi: either sign of the axis is valid.
  w <- so3_log(ref_rot_axis_angle(u, pi))
  expect_equal(sqrt(sum(w^2)), pi, tolerance = 1e-12)
  expect_equal(abs(sum(w * u)), pi, tolerance = 1e-12)
  expect_equal(so3_exp(w), ref_rot_axis_angle(u, pi), tolerance = 1e-12)
  set.seed(4)
  for (i in 1:50) {
    R <- random_rotation()
    expect_equal(so3_exp(so3_log(R)), R, tolerance = 1e-12)
  }
})

test_that("SE(3) exp and log are inverse and match the closed form", {
  set.seed(5)
  for (th in c(0, 1e-9, 1e-4, 1, 3, pi - 1e-8)) {
    w <- th * c(0, 0.6, 0.8)
    xi <- c(0.3, -1, 2, w)
    T <- se3_exp(xi)
    expect_equal(se3_log(T), xi, tolerance = 1e-8)
  }
  # Pure translation and a screw about z with known closed form.
  expect_equal(se3_exp(c(1, 2, 3, 0, 0, 0))[1:3, 4], c(1, 2, 3))
  T <- se3_exp(c(0, 0, 1, 0, 0, pi / 2))
  expect_equal(T[1:3, 4], c(0, 0, 1), tolerance = 1e-15)
  T <- se3_exp(c(1, 0, 0, 0, 0, pi / 2))
  expect_equal(T[1:3, 4], c(2 / pi, 2 / pi, 0), tolerance = 1e-14)
})

test_that("SE(2) maps are consistent", {
  for (th in c(0, 1e-9, 0.5, -2, pi - 1e-9)) {
    xi <- c(1, -0.5, th)
    expect_equal(se2_log(se2_exp(xi)), xi, tolerance = 1e-10)
  }
  expect_equal(se2_exp(c(1, 0, pi / 2))[1:2, 3], c(2 / pi, 2 / pi),
               tolerance = 1e-14)
  expect_equal(so2_log(so2_exp(3)), 3)
})

test_that("hat and vee operators are inverse", {
  w <- c(0.1, -2, 3)
  v <- c(4, 5, 6)
  expect_equal(so3_hat(w) %*% v, matrix(c(w[2] * v[3] - w[3] * v[2],
                                          w[3] * v[1] - w[1] * v[3],
                                          w[1] * v[2] - w[2] * v[1])))
  expect_equal(so3_vee(so3_hat(w)), w)
  expect_equal(se3_vee(se3_hat(c(v, w))), c(v, w))
  expect_equal(se2_vee(se2_hat(c(1, 2, 3))), c(1, 2, 3))
  expect_error(so3_vee(diag(3)), "skew")
})

test_that("SO(3) Jacobians satisfy their defining properties", {
  set.seed(6)
  for (th in c(0, 1e-7, 0.3, 2.5)) {
    w <- th * c(1, -1, 1) / sqrt(3)
    expect_equal(so3_jacobian_left(w) %*% so3_jacobian_left_inverse(w),
                 diag(3), tolerance = 1e-12)
    expect_equal(so3_jacobian_right(w) %*% so3_jacobian_right_inverse(w),
                 diag(3), tolerance = 1e-12)
    # exp(w + d) ~ exp(Jl d) exp(w) to first order.
    d <- rnorm(3) * 1e-6
    lhs <- so3_exp(w + d)
    rhs <- so3_exp(so3_jacobian_left(w) %*% d) %*% so3_exp(w)
    expect_lt(max(abs(lhs - rhs)), 1e-11)
    rhs_r <- so3_exp(w) %*% so3_exp(so3_jacobian_right(w) %*% d)
    expect_lt(max(abs(lhs - rhs_r)), 1e-11)
  }
})

test_that("adjoint maps twists between frames", {
  set.seed(7)
  T <- se3_exp(rnorm(6))
  xi <- rnorm(6)
  lhs <- T %*% se3_hat(xi) %*% solve(T)
  expect_equal(se3_vee(lhs), as.numeric(se3_adjoint(T) %*% xi),
               tolerance = 1e-12)
  T2 <- se2_exp(c(0.4, -1, 0.7))
  xi2 <- c(1, 2, 3)
  lhs2 <- T2 %*% se2_hat(xi2) %*% solve(T2)
  expect_equal(se2_vee(lhs2), as.numeric(se2_adjoint(T2) %*% xi2),
               tolerance = 1e-12)
})

test_that("quaternions follow the documented conventions", {
  q <- robot_quaternion(c(cos(pi / 4), 0, 0, sin(pi / 4)))
  expect_equal(as.matrix(robot_rotation(q)), rotz(pi / 2), tolerance = 1e-15)
  expect_equal(quaternion_rotate(q, c(1, 0, 0)), c(0, 1, 0), tolerance = 1e-15)
  set.seed(8)
  for (i in 1:30) {
    A <- random_rotation()
    B <- random_rotation()
    qa <- robot_quaternion(A)
    qb <- robot_quaternion(B)
    expect_equal(sqrt(sum(as.numeric(qa)^2)), 1, tolerance = 1e-15)
    expect_gte(as.numeric(qa)[1], 0)
    expect_equal(as.matrix(robot_rotation(qa)), A, tolerance = 1e-13)
    expect_equal(as.matrix(robot_rotation(qa * qb)), A %*% B, tolerance = 1e-13)
  }
  # Double cover: q and -q give the same rotation and zero distance.
  v <- c(0.2, -0.5, 0.1, 0.8)
  qp <- robot_quaternion(v, canonical = FALSE)
  qm <- robot_quaternion(-v, canonical = FALSE)
  expect_equal(as.matrix(robot_rotation(qp)), as.matrix(robot_rotation(qm)))
  expect_equal(quaternion_distance(qp, qm), 0, tolerance = 1e-7)
  expect_equal(as.numeric(qp * quaternion_conjugate(qp)), c(1, 0, 0, 0),
               tolerance = 1e-15)
  expect_error(robot_quaternion(c(0, 0, 0, 0)), "nonzero")
})

test_that("Shepperd conversion is accurate at rotations by pi", {
  for (u in list(c(1, 0, 0), c(0, 1, 0), c(0, 0, 1), c(1, 1, 1) / sqrt(3))) {
    R <- ref_rot_axis_angle(u, pi)
    q <- robot_quaternion(R)
    expect_equal(as.matrix(robot_rotation(q)), R, tolerance = 1e-14)
  }
})

test_that("slerp follows the shortest great circle", {
  qa <- robot_quaternion(c(1, 0, 0, 0))
  qb <- robot_quaternion(robot_rotation(axis = c(0, 0, 1), angle = 2))
  for (s in c(0, 0.25, 0.5, 1)) {
    expect_equal(as.matrix(robot_rotation(quaternion_slerp(qa, qb, s))),
                 rotz(2 * s), tolerance = 1e-14)
  }
  # Sign flip: -qb is the same rotation; slerp must still take the short arc.
  qbn <- robot_quaternion(-as.numeric(qb), canonical = FALSE)
  expect_equal(as.matrix(robot_rotation(quaternion_slerp(qa, qbn, 0.5))),
               rotz(1), tolerance = 1e-14)
  expect_length(quaternion_slerp(qa, qb, c(0, 0.5, 1)), 3)
})

test_that("pose composition, inversion and point transforms", {
  set.seed(9)
  for (i in 1:20) {
    A <- robot_pose(random_rotation(), rnorm(3))
    B <- robot_pose(random_rotation(), rnorm(3))
    expect_equal(as.matrix(A * B), as.matrix(A) %*% as.matrix(B),
                 tolerance = 1e-14)
    expect_equal(as.matrix(robot_inverse(A)), solve(as.matrix(A)),
                 tolerance = 1e-12)
    expect_equal(as.matrix(robot_inverse(A) * A), diag(4), tolerance = 1e-14)
    p <- rnorm(3)
    expect_equal(robot_transform_point(A, p),
                 as.numeric(as.matrix(A) %*% c(p, 1))[1:3], tolerance = 1e-14)
    expect_equal(robot_transform_direction(A, p),
                 as.numeric(as.matrix(A)[1:3, 1:3] %*% p), tolerance = 1e-14)
  }
  T <- robot_pose(robot_rotation(axis = c(0, 0, 1), angle = pi / 2), c(1, 2, 3))
  expect_equal(robot_transform_point(T, c(1, 0, 0)), c(1, 3, 3),
               tolerance = 1e-15)
  P <- robot_transform_point(T, rbind(c(1, 0, 0), c(0, 1, 0)))
  expect_equal(dim(P), c(2, 3))
  expect_equal(robot_compose(T, robot_inverse(T), T)$matrix, T$matrix,
               tolerance = 1e-14)
})

test_that("frame names are checked during composition", {
  A <- robot_pose(translation = c(1, 0, 0), parent = "world", child = "a")
  B <- robot_pose(translation = c(0, 1, 0), parent = "a", child = "b")
  C <- robot_pose(translation = c(0, 0, 1), parent = "x", child = "c")
  AB <- A * B
  expect_equal(AB$parent, "world")
  expect_equal(AB$child, "b")
  expect_error(A * C, "Frame mismatch")
  Ai <- robot_inverse(A)
  expect_equal(c(Ai$parent, Ai$child), c("a", "world"))
  expect_equal(pose_translation(robot_relative_pose(A, A * B)), c(0, 1, 0))
})

test_that("SE(2) poses work", {
  T <- robot_pose(robot_rotation(angle = pi / 2), c(1, 0))
  expect_equal(T$dim, 2L)
  expect_equal(robot_transform_point(T, c(1, 0)), c(1, 1), tolerance = 1e-15)
  expect_equal(as.matrix(robot_inverse(T) * T), diag(3), tolerance = 1e-15)
  expect_error(robot_pose(diag(2), c(1, 2, 3)), "length 2")
})

test_that("pose interpolation hits its endpoints", {
  a <- robot_pose(rotx(0.3), c(0, 0, 0))
  b <- robot_pose(rotz(1.2) %*% roty(0.4), c(1, 2, 3))
  for (m in c("decoupled", "screw")) {
    expect_equal(robot_pose_interp(a, b, 0, m)$matrix, a$matrix, tolerance = 1e-12)
    expect_equal(robot_pose_interp(a, b, 1, m)$matrix, b$matrix, tolerance = 1e-12)
  }
  mid <- robot_pose_interp(robot_pose(), robot_pose(rotz(1), c(2, 0, 0)), 0.5)
  expect_equal(pose_translation(mid), c(1, 0, 0))
  expect_equal(as.matrix(pose_rotation(mid)), rotz(0.5), tolerance = 1e-14)
  d <- robot_pose_distance(robot_pose(), robot_pose(rotz(0.5), c(3, 4, 0)))
  expect_equal(unname(d), c(5, 0.5), tolerance = 1e-14)
})

test_that("angle helpers", {
  expect_equal(wrap_angle(c(0, 3 * pi / 2, -3 * pi / 2, pi)),
               c(0, -pi / 2, pi / 2, -pi))
  expect_equal(deg2rad(180), pi)
  expect_equal(rad2deg(pi / 2), 90)
})
