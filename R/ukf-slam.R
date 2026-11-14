# Unscented Kalman filter localization and EKF-SLAM with known data
# association. Measurement conventions match ekf_update(): range and
# bearing (relative to the heading) with wrapped bearing innovations.

range_bearing <- function(x, lm) {
  d <- lm - x[1:2]
  c(sqrt(sum(d^2)), wrap_angle_internal(atan2(d[2], d[1]) - x[3]))
}

# Sigma points of the scaled unscented transform (Julier, 2002; Wan and
# van der Merwe, 2000) as the columns of a matrix, with mean and
# covariance weights.
sigma_points <- function(m, P, alpha, beta, kappa) {
  n <- length(m)
  lam <- alpha^2 * (n + kappa) - n
  L <- t(psd_factor(P, "cov")) * sqrt(n + lam)
  X <- cbind(m, m + L, m - L)
  wm <- c(lam / (n + lam), rep(1 / (2 * (n + lam)), 2 * n))
  wc <- wm
  wc[1] <- wc[1] + 1 - alpha^2 + beta
  list(X = X, wm = wm, wc = wc)
}

# Weighted mean of sigma points whose rows `angular` are angles (circular
# mean), and the deviations from that mean with those rows wrapped.
sigma_mean <- function(Y, wm, angular) {
  mu <- as.numeric(Y %*% wm)
  for (r in angular) {
    mu[r] <- atan2(sum(wm * sin(Y[r, ])), sum(wm * cos(Y[r, ])))
  }
  D <- Y - mu
  for (r in angular) D[r, ] <- wrap_angle_internal(D[r, ])
  list(mean = mu, dev = D)
}

#' Unscented Kalman filter localization
#'
#' A drop-in alternative to [robot_ekf()] that propagates the planar pose
#' with the unscented transform instead of Jacobians. `ukf_predict()`
#' augments the state with the control noise, draws \eqn{2(3 + 2) + 1}
#' sigma points, moves each through [unicycle_step()] with its noisy
#' control, and recombines them with the heading averaged on the circle.
#' `ukf_update()` fuses range-bearing measurements of known landmarks one
#' at a time with sigma points of the current estimate, a circular mean
#' of the predicted bearings, and wrapped bearing innovations. The default
#' scaling `alpha = 1, beta = 2, kappa = 0` makes every weight
#' nonnegative, so the predicted covariances stay positive semidefinite.
#'
#' @inheritParams robot_ekf
#' @param alpha,beta,kappa Unscented transform parameters.
#' @param filter A `robot_ukf`.
#' @return A `robot_ukf`. `ukf_update()` also records the normalized
#'   innovation squared in `nis`.
#' @examples
#' L <- rbind(a = c(2, 0), b = c(0, 2))
#' f <- robot_ukf(c(0, 0, 0), diag(0.1, 3), L, Q = diag(c(0.01, 0.01)),
#'                R = diag(c(0.01, 0.001)))
#' f <- ukf_predict(f, c(1, 0), 0.1)
#' f <- ukf_update(f, landmark_measurements(c(0.1, 0, 0), L))
#' f$mean
#' @export
robot_ukf <- function(mean, cov, landmarks, Q, R, alpha = 1, beta = 2,
                      kappa = 0) {
  structure(list(mean = check_pose2(mean, "mean"),
                 cov = check_matrix(cov, 3, 3, "cov"),
                 landmarks = check_landmarks(landmarks),
                 Q = check_matrix(Q, 2, 2, "Q"), R = check_matrix(R, 2, 2, "R"),
                 alpha = check_scalar(alpha, "alpha", positive = TRUE),
                 beta = check_scalar(beta, "beta"),
                 kappa = check_scalar(kappa, "kappa"),
                 nis = NA_real_),
            class = "robot_ukf")
}

#' @rdname robot_ukf
#' @export
ukf_predict <- function(filter, control, dt) {
  stopifnot(inherits(filter, "robot_ukf"))
  u <- check_numeric_vector(control, 2, "control")
  Pa <- matrix(0, 5, 5)
  Pa[1:3, 1:3] <- filter$cov
  Pa[4:5, 4:5] <- filter$Q
  sp <- sigma_points(c(filter$mean, 0, 0), Pa, filter$alpha, filter$beta,
                     filter$kappa)
  Y <- apply(sp$X, 2, function(z) unicycle_step(z[1:3], u[1] + z[4],
                                               u[2] + z[5], dt))
  st <- sigma_mean(Y, sp$wm, 3)
  P <- st$dev %*% (sp$wc * t(st$dev))
  filter$mean <- stats::setNames(st$mean, c("x", "y", "theta"))
  filter$cov <- (P + t(P)) / 2
  filter
}

#' @rdname robot_ukf
#' @export
ukf_update <- function(filter, measurements) {
  stopifnot(inherits(filter, "robot_ukf"))
  if (!nrow(measurements)) return(filter)
  check_landmark_ids(measurements$id, filter$landmarks)
  nis <- 0
  for (k in seq_len(nrow(measurements))) {
    lm <- filter$landmarks[as.character(measurements$id[k]), ]
    sp <- sigma_points(filter$mean, filter$cov, filter$alpha, filter$beta,
                       filter$kappa)
    Z <- apply(sp$X, 2, function(x) range_bearing(x, lm))
    zs <- sigma_mean(Z, sp$wm, 2)
    xs <- sigma_mean(sp$X, sp$wm, 3)
    S <- zs$dev %*% (sp$wc * t(zs$dev)) + filter$R
    Pxz <- xs$dev %*% (sp$wc * t(zs$dev))
    K <- Pxz %*% solve(S)
    innov <- c(measurements$range[k] - zs$mean[1],
               wrap_angle_internal(measurements$bearing[k] - zs$mean[2]))
    m <- as.numeric(filter$mean + K %*% innov)
    m[3] <- wrap_angle_internal(m[3])
    P <- filter$cov - K %*% S %*% t(K)
    filter$mean <- stats::setNames(m, c("x", "y", "theta"))
    filter$cov <- (P + t(P)) / 2
    nis <- nis + sum(innov * solve(S, innov))
  }
  filter$nis <- nis
  filter
}

#' @export
print.robot_ukf <- function(x, ...) {
  cat("<robot_ukf> mean:", paste(signif(x$mean, 5), collapse = ", "), "\n")
  cat("std devs:", paste(signif(sqrt(diag(x$cov)), 4), collapse = ", "), "\n")
  invisible(x)
}

#' EKF-SLAM with range-bearing landmarks
#'
#' Simultaneous localization and mapping with an extended Kalman filter
#' over the joint state \eqn{(x, y, \theta, m_{1x}, m_{1y}, \dots)}, with
#' known data association: each measurement carries a landmark id.
#' `robot_ekf_slam()` starts with the pose only. `ekf_slam_predict()` moves
#' the pose with the exact unicycle model and its analytic Jacobians,
#' updating the pose block and the pose-landmark cross covariances.
#' `ekf_slam_update()` initializes a landmark the first time its id is seen,
#' from the inverse measurement model with the covariance
#' \eqn{G_x P_{xx} G_x^\top + G_z R G_z^\top} and cross terms
#' \eqn{G_x P_{x\cdot}}, and fuses later sightings with a Joseph-form EKF
#' update (Thrun, Burgard and Fox, 2005, chapter 10). `ekf_slam_landmarks()`
#' returns the current map.
#'
#' @param mean Initial pose estimate.
#' @param cov Initial 3 by 3 pose covariance.
#' @param Q Control noise covariance (2 by 2) on `c(v, omega)`.
#' @param R Measurement noise covariance (2 by 2, range then bearing).
#' @param filter A `robot_ekf_slam`.
#' @param control `c(v, omega)`.
#' @param dt Duration (s).
#' @param measurements A data frame with columns `id`, `range`, `bearing`,
#'   as returned by [landmark_measurements()].
#' @return A `robot_ekf_slam` with `mean`, `cov`, `ids` (landmark ids in
#'   state order) and `nis`; `ekf_slam_landmarks()` returns a matrix of
#'   landmark estimates with ids as row names.
#' @examples
#' L <- rbind(a = c(2, 0), b = c(0, 2))
#' f <- robot_ekf_slam(c(0, 0, 0), diag(1e-6, 3), Q = diag(c(0.01, 0.01)),
#'                     R = diag(c(0.01, 0.001)))
#' f <- ekf_slam_update(f, landmark_measurements(c(0, 0, 0), L))
#' f <- ekf_slam_predict(f, c(1, 0), 0.5)
#' f <- ekf_slam_update(f, landmark_measurements(c(0.5, 0, 0), L))
#' ekf_slam_landmarks(f)
#' @export
robot_ekf_slam <- function(mean, cov, Q, R) {
  structure(list(mean = stats::setNames(check_pose2(mean, "mean"),
                                        c("x", "y", "theta")),
                 cov = check_matrix(cov, 3, 3, "cov"),
                 ids = character(0),
                 Q = check_matrix(Q, 2, 2, "Q"), R = check_matrix(R, 2, 2, "R"),
                 nis = NA_real_),
            class = "robot_ekf_slam")
}

#' @rdname robot_ekf_slam
#' @export
ekf_slam_predict <- function(filter, control, dt) {
  stopifnot(inherits(filter, "robot_ekf_slam"))
  u <- check_numeric_vector(control, 2, "control")
  x <- filter$mean[1:3]
  m <- unicycle_motion(x, u[1], u[2], dt)
  P <- filter$cov
  n <- length(filter$mean)
  P[1:3, 1:3] <- m$F %*% P[1:3, 1:3] %*% t(m$F) + m$G %*% filter$Q %*% t(m$G)
  if (n > 3) {
    P[1:3, 4:n] <- m$F %*% P[1:3, 4:n, drop = FALSE]
    P[4:n, 1:3] <- t(P[1:3, 4:n, drop = FALSE])
  }
  filter$mean[1:3] <- unicycle_step(x, u[1], u[2], dt)
  filter$cov <- (P + t(P)) / 2
  filter
}

#' @rdname robot_ekf_slam
#' @export
ekf_slam_update <- function(filter, measurements) {
  stopifnot(inherits(filter, "robot_ekf_slam"))
  if (!nrow(measurements)) return(filter)
  nis <- 0
  for (k in seq_len(nrow(measurements))) {
    id <- as.character(measurements$id[k])
    z <- c(measurements$range[k], measurements$bearing[k])
    x <- filter$mean[1:3]
    n <- length(filter$mean)
    j <- match(id, filter$ids)
    if (is.na(j)) {
      a <- x[3] + z[2]
      lm <- x[1:2] + z[1] * c(cos(a), sin(a))
      Gx <- rbind(c(1, 0, -z[1] * sin(a)), c(0, 1, z[1] * cos(a)))
      Gz <- rbind(c(cos(a), -z[1] * sin(a)), c(sin(a), z[1] * cos(a)))
      P <- filter$cov
      cross <- Gx %*% P[1:3, , drop = FALSE]
      Pll <- Gx %*% P[1:3, 1:3] %*% t(Gx) + Gz %*% filter$R %*% t(Gz)
      filter$cov <- rbind(cbind(P, t(cross)), cbind(cross, Pll))
      filter$mean <- c(filter$mean, stats::setNames(lm, paste0(id, c(".x", ".y"))))
      filter$ids <- c(filter$ids, id)
      next
    }
    li <- 3 + 2 * (j - 1) + 1:2
    d <- filter$mean[li] - x[1:2]
    q <- sum(d^2)
    r <- sqrt(q)
    if (r < 1e-9) {
      warning(sprintf("Skipped landmark '%s': the estimate coincides with it.", id),
              call. = FALSE)
      next
    }
    zhat <- c(r, wrap_angle_internal(atan2(d[2], d[1]) - x[3]))
    Hx <- rbind(c(-d[1] / r, -d[2] / r, 0), c(d[2] / q, -d[1] / q, -1))
    H <- matrix(0, 2, n)
    H[, 1:3] <- Hx
    H[, li] <- -Hx[, 1:2]
    innov <- c(z[1] - zhat[1], wrap_angle_internal(z[2] - zhat[2]))
    P <- filter$cov
    S <- H %*% P %*% t(H) + filter$R
    K <- P %*% t(H) %*% solve(S)
    filter$mean <- filter$mean + as.numeric(K %*% innov)
    filter$mean[3] <- wrap_angle_internal(filter$mean[3])
    IKH <- diag(n) - K %*% H
    P <- IKH %*% P %*% t(IKH) + K %*% filter$R %*% t(K)
    filter$cov <- (P + t(P)) / 2
    nis <- nis + sum(innov * solve(S, innov))
  }
  filter$nis <- nis
  filter
}

#' @rdname robot_ekf_slam
#' @export
ekf_slam_landmarks <- function(filter) {
  stopifnot(inherits(filter, "robot_ekf_slam"))
  if (!length(filter$ids)) return(matrix(numeric(0), 0, 2,
                                         dimnames = list(NULL, c("x", "y"))))
  matrix(filter$mean[-(1:3)], ncol = 2, byrow = TRUE,
         dimnames = list(filter$ids, c("x", "y")))
}

#' @export
print.robot_ekf_slam <- function(x, ...) {
  cat("<robot_ekf_slam> pose:", paste(signif(x$mean[1:3], 5), collapse = ", "),
      sprintf("; %d landmark%s\n", length(x$ids), if (length(x$ids) == 1) "" else "s"))
  invisible(x)
}
