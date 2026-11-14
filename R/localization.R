# Landmark-based localization with an extended Kalman filter and a
# particle filter. Measurements are range and bearing to landmarks with
# known identities. Bearing innovations are wrapped to [-pi, pi).

check_landmarks <- function(L) {
  L <- check_matrix(L, NULL, 2, "landmarks")
  if (is.null(rownames(L))) rownames(L) <- as.character(seq_len(nrow(L)))
  L
}

#' Range-bearing landmark measurements
#'
#' Returns the range and bearing (relative to the robot heading, in
#' \eqn{[-\pi, \pi)}) from `pose` to each landmark within `max_range`,
#' optionally corrupted by Gaussian noise with covariance `R`
#' (2 by 2, range then bearing).
#'
#' @param pose Robot pose `c(x, y, theta)`.
#' @param landmarks Matrix of landmark positions (one per row).
#' @param R Optional measurement noise covariance.
#' @param max_range Sensor range.
#' @param seed Optional seed (the global random state is restored).
#' @return A data frame with columns `id`, `range`, `bearing`.
#' @examples
#' landmark_measurements(c(0, 0, 0), rbind(c(1, 0), c(0, 2)))
#' @export
landmark_measurements <- function(pose, landmarks, R = NULL, max_range = Inf,
                                  seed = NULL) {
  p <- check_pose2(pose)
  L <- check_landmarks(landmarks)
  d <- sweep(L, 2, p[1:2])
  rng <- sqrt(rowSums(d^2))
  brg <- wrap_angle_internal(atan2(d[, 2], d[, 1]) - p[3])
  keep <- rng <= max_range
  out <- data.frame(id = rownames(L)[keep], range = rng[keep],
                    bearing = brg[keep], stringsAsFactors = FALSE)
  if (!is.null(R) && nrow(out)) {
    R <- check_matrix(R, 2, 2, "R")
    with_seed_internal(seed, {
      Z <- matrix(stats::rnorm(2 * nrow(out)), ncol = 2) %*% chol(R)
      out$range <- out$range + Z[, 1]
      out$bearing <- wrap_angle_internal(out$bearing + Z[, 2])
    })
  }
  out
}

#' Extended Kalman filter localization
#'
#' `robot_ekf()` creates a filter for the planar pose with mean `mean` and
#' covariance `cov`. `ekf_predict()` propagates through the exact unicycle
#' motion model with control noise covariance `Q` on `c(v, omega)`, using
#' the analytic Jacobians of [unicycle_step()]. `ekf_update()` fuses
#' range-bearing measurements of known landmarks with covariance `R`,
#' wrapping the bearing innovation and using the Joseph form so the
#' covariance stays symmetric positive semidefinite. The heading estimate
#' is wrapped after each step. Filters are immutable; each step returns a
#' new filter.
#'
#' @param mean Initial pose estimate.
#' @param cov Initial 3 by 3 covariance.
#' @param landmarks Matrix of landmark positions with row names as ids.
#' @param Q Control noise covariance (2 by 2).
#' @param R Measurement noise covariance (2 by 2).
#' @param filter A `robot_ekf`.
#' @param control `c(v, omega)`.
#' @param dt Duration (s).
#' @param measurements A data frame as returned by [landmark_measurements()].
#' @return A `robot_ekf`. `ekf_update()` also records the normalized
#'   innovation squared in `nis`.
#' @examples
#' L <- rbind(a = c(2, 0), b = c(0, 2))
#' f <- robot_ekf(c(0, 0, 0), diag(0.1, 3), L, Q = diag(c(0.01, 0.01)),
#'                R = diag(c(0.01, 0.001)))
#' f <- ekf_predict(f, c(1, 0), 0.1)
#' f <- ekf_update(f, landmark_measurements(c(0.1, 0, 0), L))
#' f$mean
#' @export
robot_ekf <- function(mean, cov, landmarks, Q, R) {
  structure(list(mean = check_pose2(mean, "mean"),
                 cov = check_matrix(cov, 3, 3, "cov"),
                 landmarks = check_landmarks(landmarks),
                 Q = check_matrix(Q, 2, 2, "Q"), R = check_matrix(R, 2, 2, "R"),
                 nis = NA_real_),
            class = "robot_ekf")
}

#' @rdname robot_ekf
#' @export
ekf_predict <- function(filter, control, dt) {
  stopifnot(inherits(filter, "robot_ekf"))
  u <- check_numeric_vector(control, 2, "control")
  x <- filter$mean
  v <- u[1]
  w <- u[2]
  th <- x[3]
  if (abs(w) < 1e-9) {
    F <- matrix(c(1, 0, 0, 0, 1, 0, -v * sin(th) * dt, v * cos(th) * dt, 1), 3)
    G <- matrix(c(cos(th) * dt, sin(th) * dt, 0,
                  -v * sin(th) * dt^2 / 2, v * cos(th) * dt^2 / 2, dt), 3)
  } else {
    th2 <- th + w * dt
    F <- matrix(c(1, 0, 0, 0, 1, 0,
                  v / w * (cos(th2) - cos(th)), v / w * (sin(th2) - sin(th)), 1), 3)
    G <- matrix(c((sin(th2) - sin(th)) / w, -(cos(th2) - cos(th)) / w, 0,
                  v * (sin(th) - sin(th2)) / w^2 + v * cos(th2) * dt / w,
                  -v * (cos(th) - cos(th2)) / w^2 + v * sin(th2) * dt / w, dt), 3)
  }
  filter$mean <- unicycle_step(x, v, w, dt)
  P <- F %*% filter$cov %*% t(F) + G %*% filter$Q %*% t(G)
  filter$cov <- (P + t(P)) / 2
  filter
}

#' @rdname robot_ekf
#' @export
ekf_update <- function(filter, measurements) {
  stopifnot(inherits(filter, "robot_ekf"))
  if (!nrow(measurements)) return(filter)
  nis <- 0
  for (k in seq_len(nrow(measurements))) {
    id <- as.character(measurements$id[k])
    if (!id %in% rownames(filter$landmarks)) {
      stop(sprintf("Unknown landmark id '%s'.", id), call. = FALSE)
    }
    lm <- filter$landmarks[id, ]
    x <- filter$mean
    d <- lm - x[1:2]
    q <- sum(d^2)
    r <- sqrt(q)
    zhat <- c(r, wrap_angle_internal(atan2(d[2], d[1]) - x[3]))
    H <- rbind(c(-d[1] / r, -d[2] / r, 0),
               c(d[2] / q, -d[1] / q, -1))
    innov <- c(measurements$range[k] - zhat[1],
               wrap_angle_internal(measurements$bearing[k] - zhat[2]))
    S <- H %*% filter$cov %*% t(H) + filter$R
    K <- filter$cov %*% t(H) %*% solve(S)
    filter$mean <- as.numeric(x + K %*% innov)
    filter$mean[3] <- wrap_angle_internal(filter$mean[3])
    IKH <- diag(3) - K %*% H
    P <- IKH %*% filter$cov %*% t(IKH) + K %*% filter$R %*% t(K)
    filter$cov <- (P + t(P)) / 2
    nis <- nis + sum(innov * solve(S, innov))
  }
  names(filter$mean) <- c("x", "y", "theta")
  filter$nis <- nis
  filter
}

#' @export
print.robot_ekf <- function(x, ...) {
  cat("<robot_ekf> mean:", paste(signif(x$mean, 5), collapse = ", "), "\n")
  cat("std devs:", paste(signif(sqrt(diag(x$cov)), 4), collapse = ", "), "\n")
  invisible(x)
}

#' Particle filter localization
#'
#' A sequential importance resampling filter for the planar pose.
#' `robot_particle_filter()` draws `n` particles from a Gaussian around
#' `mean`. `pf_predict()` moves each particle with the exact unicycle model
#' under control noise `Q`. `pf_update()` weights particles by the Gaussian
#' likelihood of range-bearing measurements (with wrapped bearing
#' residuals) and resamples with low-variance resampling when the
#' effective sample size falls below `resample_threshold * n`. The
#' estimate uses the circular mean for the heading. Seeds make each step
#' reproducible without changing the global random state.
#'
#' @param n Number of particles.
#' @param mean,cov Initial Gaussian.
#' @param landmarks Landmark matrix with row names as ids.
#' @param Q,R Control and measurement noise covariances.
#' @param filter A `robot_particle_filter`.
#' @param control `c(v, omega)`.
#' @param dt Duration.
#' @param measurements Data frame from [landmark_measurements()].
#' @param resample_threshold Fraction of `n` below which to resample.
#' @param seed Optional seed.
#' @return A `robot_particle_filter`; `pf_estimate()` returns the weighted
#'   mean pose and covariance.
#' @examples
#' L <- rbind(a = c(2, 0), b = c(0, 2))
#' pf <- robot_particle_filter(200, c(0, 0, 0), diag(0.05, 3), L,
#'                             Q = diag(0.01, 2), R = diag(c(0.01, 0.001)),
#'                             seed = 1)
#' pf <- pf_update(pf, landmark_measurements(c(0, 0, 0), L), seed = 2)
#' pf_estimate(pf)$mean
#' @export
robot_particle_filter <- function(n, mean, cov, landmarks, Q, R, seed = NULL) {
  n <- check_count(n, "n")
  m <- check_pose2(mean, "mean")
  C <- check_matrix(cov, 3, 3, "cov")
  X <- with_seed_internal(seed,
    matrix(stats::rnorm(3 * n), n) %*% chol(C) + matrix(m, n, 3, byrow = TRUE))
  X[, 3] <- wrap_angle_internal(X[, 3])
  structure(list(particles = X, weights = rep(1 / n, n),
                 landmarks = check_landmarks(landmarks),
                 Q = check_matrix(Q, 2, 2, "Q"), R = check_matrix(R, 2, 2, "R")),
            class = "robot_particle_filter")
}

#' @rdname robot_particle_filter
#' @export
pf_predict <- function(filter, control, dt, seed = NULL) {
  stopifnot(inherits(filter, "robot_particle_filter"))
  u <- check_numeric_vector(control, 2, "control")
  X <- filter$particles
  n <- nrow(X)
  noise <- with_seed_internal(seed,
    matrix(stats::rnorm(2 * n), n) %*% chol(filter$Q))
  for (i in seq_len(n)) {
    X[i, ] <- unicycle_step(X[i, ], u[1] + noise[i, 1], u[2] + noise[i, 2], dt)
  }
  filter$particles <- X
  filter
}

#' @rdname robot_particle_filter
#' @export
pf_update <- function(filter, measurements, resample_threshold = 0.5,
                      seed = NULL) {
  stopifnot(inherits(filter, "robot_particle_filter"))
  if (!nrow(measurements)) return(filter)
  X <- filter$particles
  Rinv <- solve(filter$R)
  logw <- log(filter$weights)
  for (k in seq_len(nrow(measurements))) {
    lm <- filter$landmarks[as.character(measurements$id[k]), ]
    dx <- lm[1] - X[, 1]
    dy <- lm[2] - X[, 2]
    er <- measurements$range[k] - sqrt(dx^2 + dy^2)
    eb <- wrap_angle_internal(measurements$bearing[k] -
                                (atan2(dy, dx) - X[, 3]))
    logw <- logw - 0.5 * (Rinv[1, 1] * er^2 + 2 * Rinv[1, 2] * er * eb +
                            Rinv[2, 2] * eb^2)
  }
  w <- exp(logw - max(logw))
  w <- w / sum(w)
  filter$weights <- w
  n <- nrow(X)
  if (1 / sum(w^2) < resample_threshold * n) {
    idx <- with_seed_internal(seed, {
      u0 <- stats::runif(1) / n
      findInterval(u0 + (seq_len(n) - 1) / n, c(0, cumsum(w)),
                   rightmost.closed = TRUE)
    })
    idx <- pmin(pmax(idx, 1), n)
    filter$particles <- X[idx, , drop = FALSE]
    filter$weights <- rep(1 / n, n)
  }
  filter
}

#' @rdname robot_particle_filter
#' @export
pf_estimate <- function(filter) {
  stopifnot(inherits(filter, "robot_particle_filter"))
  X <- filter$particles
  w <- filter$weights
  th <- atan2(sum(w * sin(X[, 3])), sum(w * cos(X[, 3])))
  m <- c(x = sum(w * X[, 1]), y = sum(w * X[, 2]), theta = th)
  D <- cbind(X[, 1] - m[1], X[, 2] - m[2], wrap_angle_internal(X[, 3] - th))
  list(mean = m, cov = crossprod(D * sqrt(w)), ess = 1 / sum(w^2))
}
