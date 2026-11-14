# Base-graphics visualization. Plotting is kept separate from computation
# and works on headless devices such as pdf().

projection_axes <- function(projection) {
  switch(projection, xy = 1:2, xz = c(1, 3), yz = 2:3)
}

robot_segments <- function(robot, q) {
  fk <- fk_internal(robot, q)
  segs <- list()
  for (k in seq_along(fk$link)[-1]) {
    a <- fk$link[[robot$parent_index[k]]][1:3, 4]
    b <- fk$joint[[k]][1:3, 4]
    c_ <- fk$link[[k]][1:3, 4]
    segs[[length(segs) + 1]] <- rbind(a, b)
    segs[[length(segs) + 1]] <- rbind(b, c_)
  }
  list(segments = segs, fk = fk)
}

view_matrix <- function(theta, phi) {
  graphics::persp(0:1, 0:1, matrix(0, 2, 2), theta = theta, phi = phi,
                  plot = FALSE)
}

project3d <- function(P, pmat) {
  P <- if (is.matrix(P)) P else matrix(P, 1)
  tr <- cbind(P, 1) %*% pmat
  cbind(tr[, 1] / tr[, 4], tr[, 2] / tr[, 4])
}

#' Plot a robot configuration
#'
#' Draws the kinematic skeleton of a robot with base graphics: links as
#' segments between consecutive frame origins, joints as points and the
#' end effector as a filled marker. Planar projections (`"xy"`, `"xz"`,
#' `"yz"`) are exact orthographic views; `"3d"` uses a perspective view
#' set by `theta` and `phi` as in [graphics::persp()]. Several
#' configurations can be overlaid with `add = TRUE`. Works on any graphics
#' device, including headless ones.
#'
#' @param robot A `robot_model`.
#' @param q Configuration.
#' @param projection `"xy"`, `"xz"`, `"yz"`, or `"3d"`.
#' @param frames Draw coordinate axes of each link frame (red x, green y,
#'   blue z).
#' @param frame_scale Length of the frame axes.
#' @param add Add to the current plot.
#' @param col Link color.
#' @param lwd Link line width.
#' @param xlim,ylim Optional axis limits for planar projections.
#' @param theta,phi Viewing angles for `"3d"`.
#' @param main Plot title.
#' @param ... Further arguments passed to [graphics::plot()].
#' @return Invisibly, the matrix of projected link origins.
#' @examples
#' robot <- robot_example("planar2")
#' robot_plot(robot, c(pi / 4, pi / 4))
#' robot_plot(robot, c(-pi / 4, pi / 2), add = TRUE, col = "grey50")
#' @export
robot_plot <- function(robot, q, projection = c("xy", "xz", "yz", "3d"),
                       frames = FALSE, frame_scale = NULL, add = FALSE,
                       col = "steelblue4", lwd = 3, xlim = NULL, ylim = NULL,
                       theta = 30, phi = 20, main = NULL, ...) {
  check_robot(robot)
  projection <- match.arg(projection)
  q <- check_q(robot, q)
  sg <- robot_segments(robot, q)
  origins <- t(vapply(sg$fk$link, function(T) T[1:3, 4], numeric(3)))
  reach <- max(1e-3, chain_reach_bound(robot, length(robot$link_names)),
               max(abs(origins)))
  frame_scale <- frame_scale %||% (0.12 * reach)
  if (projection == "3d") {
    pmat <- if (add && !is.null(.robot_plot_state$pmat)) .robot_plot_state$pmat
            else NULL
    if (is.null(pmat)) {
      lim <- c(-reach, reach)
      pmat <- graphics::persp(lim, lim, matrix(min(0, min(origins[, 3])), 2, 2),
                              zlim = c(min(0, min(origins[, 3])), reach),
                              theta = theta, phi = phi, border = "grey85",
                              col = NA, xlab = "x", ylab = "y", zlab = "z",
                              main = main %||% robot$name, box = TRUE)
      .robot_plot_state$pmat <- pmat
    }
    proj <- function(P) project3d(P, pmat)
  } else {
    ax <- projection_axes(projection)
    proj <- function(P) {
      P <- if (is.matrix(P)) P else matrix(P, 1)
      P[, ax, drop = FALSE]
    }
    if (!add) {
      lim <- c(-reach, reach) * 1.05
      graphics::plot(NA, xlim = xlim %||% lim, ylim = ylim %||% lim, asp = 1,
                     xlab = c("x", "y", "z")[ax[1]],
                     ylab = c("x", "y", "z")[ax[2]],
                     main = main %||% robot$name, ...)
      graphics::abline(h = 0, v = 0, col = "grey90")
    }
  }
  for (s in sg$segments) {
    P <- proj(s)
    graphics::lines(P[, 1], P[, 2], col = col, lwd = lwd)
  }
  O <- proj(origins)
  graphics::points(O[-1, 1], O[-1, 2], pch = 21, bg = "white", col = col,
                   cex = 1.1)
  graphics::points(O[1, 1], O[1, 2], pch = 15, col = "grey20", cex = 1.4)
  ke <- frame_index(robot, NULL)
  graphics::points(O[ke, 1], O[ke, 2], pch = 19, col = "firebrick", cex = 1.3)
  if (frames) {
    cols <- c("red3", "green4", "blue3")
    for (T in sg$fk$link) {
      for (a in 1:3) {
        P <- proj(rbind(T[1:3, 4], T[1:3, 4] + frame_scale * T[1:3, a]))
        graphics::lines(P[, 1], P[, 2], col = cols[a], lwd = 1.5)
      }
    }
  }
  invisible(O)
}

.robot_plot_state <- new.env(parent = emptyenv())

#' Plot the reachable workspace
#'
#' Samples `n` random configurations within the joint limits and plots
#' the positions of `frame`. This is a Monte Carlo picture of the
#' workspace, not an exact boundary.
#'
#' @param robot A `robot_model`.
#' @param n Number of samples.
#' @param frame Frame (`NULL` for the end effector).
#' @param projection `"xy"`, `"xz"` or `"yz"`.
#' @param seed Optional seed (the global random state is restored).
#' @param color_by Optional function of a configuration returning a number
#'   used to color points, for example a manipulability measure.
#' @param col Point color when `color_by` is `NULL`.
#' @param main Plot title.
#' @param ... Further arguments to [graphics::plot()].
#' @return Invisibly, the sampled positions.
#' @examples
#' robot <- robot_example("planar2")
#' robot_plot_workspace(robot, n = 500, seed = 1)
#' @export
robot_plot_workspace <- function(robot, n = 2000, frame = NULL,
                                 projection = c("xy", "xz", "yz"),
                                 seed = NULL, color_by = NULL,
                                 col = grDevices::adjustcolor("steelblue", 0.4),
                                 main = "Workspace samples", ...) {
  projection <- match.arg(projection)
  Q <- robot_random_configuration(robot, n, seed = seed)
  if (!is.matrix(Q)) Q <- matrix(Q, 1)
  P <- robot_fk_positions(robot, Q, frame)
  ax <- projection_axes(projection)
  if (!is.null(color_by)) {
    v <- apply(Q, 1, color_by)
    pal <- grDevices::hcl.colors(64, "viridis")
    rng <- range(v, finite = TRUE)
    idx <- 1 + floor(63 * (v - rng[1]) / max(diff(rng), 1e-12))
    col <- pal[pmin(64, pmax(1, idx))]
  }
  graphics::plot(P[, ax], pch = 16, cex = 0.5, col = col, asp = 1,
                 xlab = c("x", "y", "z")[ax[1]], ylab = c("x", "y", "z")[ax[2]],
                 main = main, ...)
  invisible(P)
}
