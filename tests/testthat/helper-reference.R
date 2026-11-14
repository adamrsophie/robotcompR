# Independent reference formulas used by the tests. They are written from
# textbook expressions and never call the package's kinematics code.

ref_rot_axis_angle <- function(u, th) {
  u <- u / sqrt(sum(u^2))
  K <- matrix(c(0, u[3], -u[2], -u[3], 0, u[1], u[2], -u[1], 0), 3, 3)
  diag(3) + sin(th) * K + (1 - cos(th)) * K %*% K
}

ref_dh_standard <- function(a, alpha, d, theta) {
  ct <- cos(theta); st <- sin(theta); ca <- cos(alpha); sa <- sin(alpha)
  matrix(c(ct, st, 0, 0,
           -st * ca, ct * ca, sa, 0,
           st * sa, -ct * sa, ca, 0,
           a * ct, a * st, d, 1), 4, 4)
}

ref_dh_modified <- function(a, alpha, d, theta) {
  ct <- cos(theta); st <- sin(theta); ca <- cos(alpha); sa <- sin(alpha)
  matrix(c(ct, st * ca, st * sa, 0,
           -st, ct * ca, ct * sa, 0,
           0, -sa, ca, 0,
           a, -sa * d, ca * d, 1), 4, 4)
}

ref_planar_fk <- function(l, q) {
  cq <- cumsum(q)
  c(sum(l * cos(cq)), sum(l * sin(cq)))
}

ref_planar2_jacobian <- function(l1, l2, q) {
  rbind(c(-l1 * sin(q[1]) - l2 * sin(q[1] + q[2]), -l2 * sin(q[1] + q[2])),
        c(l1 * cos(q[1]) + l2 * cos(q[1] + q[2]), l2 * cos(q[1] + q[2])))
}

random_rotation <- function() {
  q <- stats::rnorm(4)
  q <- q / sqrt(sum(q^2))
  w <- q[1]; x <- q[2]; y <- q[3]; z <- q[4]
  matrix(c(1 - 2 * (y^2 + z^2), 2 * (x * y + w * z), 2 * (x * z - w * y),
           2 * (x * y - w * z), 1 - 2 * (x^2 + z^2), 2 * (y * z + w * x),
           2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x^2 + y^2)), 3, 3)
}
