# Numerical and analytical inverse kinematics.

ik_error <- function(Tc, target_p, target_R) {
  ep <- target_p - Tc[1:3, 4]
  er <- if (is.null(target_R)) c(0, 0, 0)
        else so3_log(orthonormalize(target_R %*% t(Tc[1:3, 1:3])), tol = 1e-4)
  c(ep, er)
}

pinv_solve <- function(A, b, tol = 1e-10) {
  s <- svd(A)
  keep <- s$d > tol * max(s$d, .Machine$double.eps)
  if (!any(keep)) return(numeric(ncol(A)))
  as.numeric(s$v[, keep, drop = FALSE] %*%
               ((crossprod(s$u[, keep, drop = FALSE], b)) / s$d[keep]))
}

chain_reach_bound <- function(robot, k) {
  r <- 0
  kk <- k
  while (kk > 1) {
    j <- robot$joints[[kk - 1L]]
    r <- r + vnorm(j$origin[1:3, 4]) +
      (if (is.null(j$post)) 0 else vnorm(j$post[1:3, 4]))
    if (j$type == "prismatic") {
      r <- r + max(abs(j$limits + j$offset))
    }
    kk <- robot$parent_index[kk]
  }
  r
}

ik_single <- function(robot, k, q0, target_p, target_R, W, method, tol,
                      max_iterations, lambda, limits, step_tol, max_step) {
  L <- robot_limits(robot)
  q <- if (limits) pmin(pmax(q0, L[, 1]), L[, 2]) else q0
  sw <- sqrt(W)
  fk <- fk_internal(robot, q)
  e <- ik_error(fk$link[[k]], target_p, target_R)
  err <- sqrt(sum(W * e * e))
  lam <- lambda
  it <- 0L
  while (it < max_iterations && err > tol) {
    it <- it + 1L
    J <- jacobian_internal(robot, q, k, fk = fk)
    Jw <- sw * J
    ew <- sw * e
    step_for <- function(Jw) {
      if (method == "pseudoinverse") {
        pinv_solve(Jw, ew)
      } else if (method == "damped_least_squares") {
        as.numeric(t(Jw) %*% solve(Jw %*% t(Jw) + lam^2 * diag(6), ew))
      } else {
        JtJ <- crossprod(Jw)
        H <- JtJ + lam * max(diag(JtJ), 1e-12) * diag(robot$dof)
        as.numeric(solve(H, crossprod(Jw, ew)))
      }
    }
    dq <- step_for(Jw)
    if (limits) {
      # Active set: joints sitting on a limit and pushed outward are frozen
      # and the step is recomputed with the remaining joints, so one
      # saturated joint does not stall the others.
      eps <- 1e-12 * pmax(1, abs(q))
      frozen <- rep(FALSE, length(q))
      for (pass in seq_along(q)) {
        push <- (q <= L[, 1] + eps & dq < 0) | (q >= L[, 2] - eps & dq > 0)
        new <- push & !frozen
        if (!any(new)) break
        frozen <- frozen | new
        Jf <- Jw
        Jf[, frozen] <- 0
        dq <- step_for(Jf)
        dq[frozen] <- 0
      }
    }
    big <- max(abs(dq))
    if (big > max_step) dq <- dq * (max_step / big)
    qn <- q + dq
    if (limits) qn <- pmin(pmax(qn, L[, 1]), L[, 2])
    fkn <- fk_internal(robot, qn)
    en <- ik_error(fkn$link[[k]], target_p, target_R)
    errn <- sqrt(sum(W * en * en))
    if (method == "levenberg_marquardt") {
      if (errn < err) {
        q <- qn; fk <- fkn; e <- en
        step <- abs(err - errn)
        err <- errn
        lam <- max(lam / 10, 1e-12)
        if (vnorm(dq) < step_tol && step < step_tol * tol) break
      } else {
        lam <- min(lam * 10, 1e12)
        if (lam >= 1e12) break
      }
    } else {
      # Backtracking keeps undamped Newton steps from diverging.
      alpha <- 1
      while (errn >= err && alpha > 1 / 64) {
        alpha <- alpha / 2
        qn <- q + alpha * dq
        if (limits) qn <- pmin(pmax(qn, L[, 1]), L[, 2])
        fkn <- fk_internal(robot, qn)
        en <- ik_error(fkn$link[[k]], target_p, target_R)
        errn <- sqrt(sum(W * en * en))
      }
      if (errn >= err) break
      moved <- vnorm(qn - q)
      q <- qn; fk <- fkn; e <- en; err <- errn
      if (moved < step_tol) break
    }
  }
  list(q = q, error = e, residual = err, iterations = it, fk = fk)
}

#' Numerical inverse kinematics
#'
#' Solves \eqn{FK(q) \approx T^*} for a target pose (or position) of
#' `frame`. The error is \eqn{e = (p^* - p, \log(R^* R^\top))} in world
#' coordinates, a proper SO(3) orientation error rather than a difference
#' of Euler angles. With \eqn{W = \mathrm{diag}(weights)} the solver
#' minimizes \eqn{e^\top W e = \|W^{1/2} e\|^2}; `residual` is its square
#' root. Joints that sit on a limit and would be pushed outward are frozen
#' for that step (an active set), so the other joints keep moving. Methods:
#'
#' * `"levenberg_marquardt"` (default): \eqn{\Delta q = (J^\top W J +
#'   \lambda \mu I)^{-1} J^\top W e}, where \eqn{\mu} is the largest
#'   diagonal entry of \eqn{J^\top W J}, with \eqn{\lambda} decreased
#'   after successful steps and increased after rejected steps.
#' * `"damped_least_squares"`: \eqn{\Delta q = J^\top (J J^\top +
#'   \lambda^2 I)^{-1} e} with fixed \eqn{\lambda}, plus backtracking.
#' * `"pseudoinverse"`: \eqn{\Delta q = J^{+} e} from a truncated SVD,
#'   plus backtracking.
#'
#' With `respect_limits = TRUE` each iterate is projected onto the joint
#' limits. When the solve from `initial` fails, up to `restarts` further
#' attempts start from random configurations within the limits; the best
#' result is returned. Numerical IK can fail even for reachable targets
#' and never proves that a target is unreachable.
#'
#' @param robot A `robot_model`.
#' @param target A `robot_pose` (SE(3)) or a length-3 position. A position
#'   target sets the orientation weights to zero.
#' @param initial Initial configuration (defaults to [robot_home()]).
#' @param frame Frame to place (`NULL` for the end effector).
#' @param method `"levenberg_marquardt"`, `"damped_least_squares"`, or
#'   `"pseudoinverse"`.
#' @param weights Length-6 nonnegative weights for
#'   \eqn{(x, y, z, \omega_x, \omega_y, \omega_z)} errors. Use zeros for
#'   unconstrained directions, for example `c(1, 1, 0, 0, 0, 1)` for a
#'   planar arm with a pose target.
#' @param tol Convergence tolerance on the weighted error norm.
#' @param max_iterations Maximum iterations per attempt.
#' @param lambda Initial damping.
#' @param max_step Largest change of any joint variable in one iteration
#'   (radians or meters). Steps are scaled down to this size, which keeps
#'   iterations started near singular configurations from jumping far.
#' @param restarts Maximum number of extra attempts from random initial
#'   configurations, used only while the solver has not converged.
#' @param respect_limits Project iterates onto the joint limits.
#' @param seed Seed for the restart configurations (default 1, so results
#'   are reproducible); the global random state is never changed.
#' @return A `robot_ik_solution` list with `q`, `converged`, `residual`,
#'   `position_error`, `orientation_error` (radians), `iterations`,
#'   `attempts`, `active_limits` (names of joints at a limit),
#'   `singular` (rank-deficient weighted Jacobian at the solution),
#'   `reach_bound` (an upper bound on the reachable distance from the base)
#'   and `target_distance`.
#' @examples
#' robot <- robot_example("planar2")
#' sol <- robot_ik(robot, c(1, 1, 0), initial = c(0.3, 0.3))
#' sol$q
#' sol$converged
#' @export
robot_ik <- function(robot, target, initial = NULL, frame = NULL,
                     method = c("levenberg_marquardt",
                                "damped_least_squares", "pseudoinverse"),
                     weights = NULL, tol = 1e-10, max_iterations = 200,
                     lambda = 0.01, max_step = 0.5, restarts = 5,
                     respect_limits = TRUE, seed = 1L) {
  check_robot(robot)
  method <- match.arg(method)
  k <- frame_index(robot, frame)
  if (inherits(target, "robot_pose")) {
    if (target$dim != 3) stop("`target` must be an SE(3) pose.", call. = FALSE)
    target_p <- pose_translation_vector(target)
    target_R <- pose_rotation_matrix(target)
    W <- weights %||% rep(1, 6)
  } else {
    target_p <- check_numeric_vector(target, 3, "target")
    target_R <- NULL
    W <- weights %||% c(1, 1, 1, 0, 0, 0)
    W[4:6] <- 0
  }
  W <- check_numeric_vector(W, 6, "weights")
  if (any(W < 0)) stop("`weights` must be nonnegative.", call. = FALSE)
  q0 <- if (is.null(initial)) robot_home(robot) else check_q(robot, initial,
                                                             "initial")
  tol <- check_scalar(tol, "tol", positive = TRUE)
  max_iterations <- check_count(max_iterations, "max_iterations")
  restarts <- check_count(restarts, "restarts", min = 0)
  lambda <- check_scalar(lambda, "lambda", positive = TRUE)
  max_step <- check_scalar(max_step, "max_step", positive = TRUE)
  if (robot$dof == 0) {
    e <- ik_error(fk_internal(robot, numeric(0))$link[[k]], target_p, target_R)
    res <- sqrt(sum(W * e * e))
    return(structure(list(
      q = stats::setNames(numeric(0), character(0)), converged = res <= tol,
      residual = res, position_error = vnorm(e[1:3] * (W[1:3] > 0)),
      orientation_error = if (is.null(target_R) || all(W[4:6] == 0)) NA_real_
                          else vnorm(e[4:6] * (W[4:6] > 0)),
      iterations = 0L, attempts = 1L, method = method,
      active_limits = character(0), singular = FALSE,
      reach_bound = chain_reach_bound(robot, k),
      target_distance = vnorm(target_p - robot$base[1:3, 4])
    ), class = "robot_ik_solution"))
  }
  run <- function(qs) ik_single(robot, k, qs, target_p, target_R, W, method,
                                tol, max_iterations, lambda, respect_limits,
                                step_tol = 1e-14, max_step = max_step)
  best <- run(q0)
  attempts <- 1L
  if (best$residual > tol && restarts > 0) {
    starts <- robot_random_configuration(robot, restarts, seed = seed %||% 1L)
    if (!is.matrix(starts)) starts <- matrix(starts, 1)
    for (i in seq_len(nrow(starts))) {
      attempts <- attempts + 1L
      r <- run(starts[i, ])
      if (r$residual < best$residual) best <- r
      if (best$residual <= tol) break
    }
  }
  q <- best$q
  types <- vapply(robot$joints[robot$movable], `[[`, "", "type")
  q[types == "continuous"] <- wrap_angle_internal(q[types == "continuous"])
  names(q) <- robot_joint_names(robot)
  L <- robot_limits(robot)
  at_limit <- (abs(q - L[, 1]) < 1e-9 | abs(q - L[, 2]) < 1e-9)
  J <- jacobian_internal(robot, q, k)
  Jw <- sqrt(W) * J
  wrank <- function(Jw) {
    sv <- svd(Jw)$d
    sum(sv > 1e-8 * max(sv, .Machine$double.eps))
  }
  rank <- wrank(Jw)
  # The rank the weighted task can reach at all (for example 2 for the
  # position of a planar arm, whose z row is identically zero), taken as
  # the largest rank over a few fixed sample configurations.
  probes <- with_seed_internal(1L, robot_random_configuration(robot, 4))
  if (!is.matrix(probes)) probes <- matrix(probes, ncol = robot$dof)
  needed <- max(rank, apply(probes, 1, function(qp)
    wrank(sqrt(W) * jacobian_internal(robot, qp, k))))
  base_p <- robot$base[1:3, 4]
  structure(list(
    q = q,
    converged = best$residual <= tol,
    residual = best$residual,
    position_error = vnorm(best$error[1:3] * (W[1:3] > 0)),
    orientation_error = if (is.null(target_R) || all(W[4:6] == 0)) NA_real_
                        else vnorm(best$error[4:6] * (W[4:6] > 0)),
    iterations = best$iterations,
    attempts = attempts,
    method = method,
    active_limits = names(q)[at_limit],
    singular = rank < needed,
    reach_bound = chain_reach_bound(robot, k),
    target_distance = vnorm(target_p - base_p)
  ), class = "robot_ik_solution")
}

#' @export
print.robot_ik_solution <- function(x, ...) {
  cat(sprintf("<robot_ik_solution> %s after %d iterations (%d attempt%s, %s)\n",
              if (x$converged) "converged" else "NOT converged",
              x$iterations, x$attempts, if (x$attempts == 1) "" else "s",
              x$method))
  cat("q =", paste(signif(x$q, 6), collapse = ", "), "\n")
  cat(sprintf("residual %.3g, position error %.3g m%s\n", x$residual,
              x$position_error,
              if (is.na(x$orientation_error)) ""
              else sprintf(", orientation error %.3g rad",
                           x$orientation_error)))
  if (length(x$active_limits)) {
    cat("joints at limits:", paste(x$active_limits, collapse = ", "), "\n")
  }
  if (x$singular) cat("warning: the solution is at a singular configuration\n")
  if (x$target_distance > x$reach_bound) {
    cat(sprintf("note: target distance %.4g exceeds the reach bound %.4g\n",
                x$target_distance, x$reach_bound))
  }
  invisible(x)
}

#' Closed-form inverse kinematics of a planar two-link arm
#'
#' For link lengths \eqn{l_1, l_2} and target \eqn{(x, y)},
#' \eqn{c_2 = (x^2 + y^2 - l_1^2 - l_2^2) / (2 l_1 l_2)}. When
#' \eqn{|c_2| \le 1} the solutions are \eqn{q_2 = \pm\arccos c_2} and
#' \eqn{q_1 = \mathrm{atan2}(y, x) - \mathrm{atan2}(l_2 \sin q_2,
#' l_1 + l_2 \cos q_2)}. At the workspace boundary (\eqn{|c_2| = 1}) the
#' two solutions coincide and one row is returned. Unreachable targets
#' return a matrix with zero rows. At the origin with \eqn{l_1 = l_2}
#' every \eqn{q_1} is a solution; the function returns \eqn{q_1 = 0}.
#'
#' @param x,y Target position.
#' @param l1,l2 Link lengths.
#' @param tol Tolerance for the reachability test.
#' @return A matrix with columns `q1`, `q2` and rows `q2_positive` and
#'   `q2_negative`.
#' @examples
#' ik_planar2(-1, 1, 1, 1)
#' ik_planar2(3, 0, 1, 1)
#' @export
ik_planar2 <- function(x, y, l1, l2, tol = 1e-12) {
  x <- check_scalar(x, "x")
  y <- check_scalar(y, "y")
  l1 <- check_scalar(l1, "l1", positive = TRUE)
  l2 <- check_scalar(l2, "l2", positive = TRUE)
  c2 <- (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)
  if (abs(c2) > 1 + tol) {
    return(matrix(numeric(0), 0, 2, dimnames = list(NULL, c("q1", "q2"))))
  }
  r <- sqrt(x^2 + y^2)
  # Positional degeneracy is a distance test, so it scales linearly with
  # the arm, unlike the dimensionless reachability slack above.
  if (r <= tol * (l1 + l2) && abs(l1 - l2) <= tol * (l1 + l2)) {
    return(matrix(c(0, pi), 1, dimnames = list("q2_positive", c("q1", "q2"))))
  }
  # Half-angle form tan(q2 / 2) = sqrt(((l1 + l2)^2 - r^2) / (r^2 - (l1 - l2)^2))
  # stays accurate near the stretched and folded configurations, where
  # acos() loses half its digits.
  num <- max(0, (l1 + l2 - r) * (l1 + l2 + r))
  den <- max(0, (r - abs(l1 - l2)) * (r + abs(l1 - l2)))
  q2 <- 2 * atan2(sqrt(num), sqrt(den))
  q2s <- if (num == 0 || den == 0) q2 else c(q2, -q2)
  out <- t(vapply(q2s, function(b) {
    # l1 + l2 cos(b) written without cancellation near b = pi.
    a <- atan2(y, x) - atan2(l2 * sin(b), (l1 - l2) + 2 * l2 * cos(b / 2)^2)
    c(wrap_angle_internal(a), b)
  }, numeric(2)))
  dimnames(out) <- list(c("q2_positive", "q2_negative")[seq_len(nrow(out))],
                        c("q1", "q2"))
  out
}
