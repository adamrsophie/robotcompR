# Two-dimensional occupancy grids.

#' Occupancy grid maps
#'
#' Creates a 2D occupancy grid covering `xlim` by `ylim` with square cells
#' of side `resolution`. Cells store log-odds of occupancy, initialized
#' from the prior probability. Cell `[i, j]` covers
#' `x` in `xlim[1] + (j - 1, j) * resolution` and `y` in
#' `ylim[1] + (i - 1, i) * resolution` (rows are y, columns are x).
#'
#' @param xlim,ylim World extents (m).
#' @param resolution Cell size (m).
#' @param prior Prior occupancy probability.
#' @param occupied Optional logical matrix to initialize a known map
#'   (`TRUE` is occupied) with probabilities 0.99 and 0.01.
#' @return A `robot_occupancy_grid`.
#' @examples
#' g <- robot_occupancy_grid(c(0, 5), c(0, 3), resolution = 0.1)
#' g
#' @export
robot_occupancy_grid <- function(xlim, ylim, resolution, prior = 0.5,
                                 occupied = NULL) {
  xlim <- check_numeric_vector(xlim, 2, "xlim")
  ylim <- check_numeric_vector(ylim, 2, "ylim")
  res <- check_scalar(resolution, "resolution", positive = TRUE)
  if (xlim[2] <= xlim[1] || ylim[2] <= ylim[1]) {
    stop("Extents must be increasing.", call. = FALSE)
  }
  nc <- ceiling((xlim[2] - xlim[1]) / res - 1e-9)
  nr <- ceiling((ylim[2] - ylim[1]) / res - 1e-9)
  logit <- function(p) log(p / (1 - p))
  L <- matrix(logit(check_scalar(prior, "prior")), nr, nc)
  if (!is.null(occupied)) {
    if (!is.matrix(occupied) || !all(dim(occupied) == c(nr, nc))) {
      stop(sprintf("`occupied` must be a %d by %d logical matrix.", nr, nc),
           call. = FALSE)
    }
    L[] <- ifelse(occupied, logit(0.99), logit(0.01))
  }
  structure(list(logodds = L, xlim = c(xlim[1], xlim[1] + nc * res),
                 ylim = c(ylim[1], ylim[1] + nr * res), resolution = res),
            class = "robot_occupancy_grid")
}

#' @export
print.robot_occupancy_grid <- function(x, ...) {
  cat(sprintf("<robot_occupancy_grid> %d by %d cells of %g m covering [%g, %g] x [%g, %g]\n",
              nrow(x$logodds), ncol(x$logodds), x$resolution, x$xlim[1],
              x$xlim[2], x$ylim[1], x$ylim[2]))
  invisible(x)
}

#' Grid coordinate conversion and probabilities
#'
#' `grid_world_to_cell()` maps world points to `c(row, col)` indices
#' (`NA` outside the map); `grid_cell_to_world()` returns cell centers.
#' `grid_probability()` returns the occupancy probability matrix.
#' `grid_occupied()` thresholds it.
#'
#' @param grid A `robot_occupancy_grid`.
#' @param points Matrix of world points (one per row) or one point.
#' @param cells Matrix of `c(row, col)` indices.
#' @param threshold Probability threshold.
#' @return Matrices.
#' @examples
#' g <- robot_occupancy_grid(c(0, 1), c(0, 1), 0.25)
#' grid_world_to_cell(g, c(0.3, 0.9))
#' grid_cell_to_world(g, cbind(1, 1))
#' @name grid_coordinates
NULL

#' @rdname grid_coordinates
#' @export
grid_world_to_cell <- function(grid, points) {
  P <- if (is.matrix(points)) points else matrix(points, 1)
  col <- floor((P[, 1] - grid$xlim[1]) / grid$resolution) + 1
  row <- floor((P[, 2] - grid$ylim[1]) / grid$resolution) + 1
  out <- cbind(row = row, col = col)
  bad <- row < 1 | row > nrow(grid$logodds) | col < 1 | col > ncol(grid$logodds)
  out[bad, ] <- NA
  out
}

#' @rdname grid_coordinates
#' @export
grid_cell_to_world <- function(grid, cells) {
  C <- if (is.matrix(cells)) cells else matrix(cells, 1)
  cbind(x = grid$xlim[1] + (C[, 2] - 0.5) * grid$resolution,
        y = grid$ylim[1] + (C[, 1] - 0.5) * grid$resolution)
}

#' @rdname grid_coordinates
#' @export
grid_probability <- function(grid) 1 / (1 + exp(-grid$logodds))

#' @rdname grid_coordinates
#' @export
grid_occupied <- function(grid, threshold = 0.65) {
  grid_probability(grid) > threshold
}

# Exact grid traversal (Amanatides and Woo, 1987). Returns the cells
# crossed by the segment from a to b, in order, excluding cells outside.
grid_traverse <- function(grid, a, b) {
  res <- grid$resolution
  x0 <- (a[1] - grid$xlim[1]) / res
  y0 <- (a[2] - grid$ylim[1]) / res
  x1 <- (b[1] - grid$xlim[1]) / res
  y1 <- (b[2] - grid$ylim[1]) / res
  cx <- floor(x0)
  cy <- floor(y0)
  ex <- floor(x1)
  ey <- floor(y1)
  dx <- x1 - x0
  dy <- y1 - y0
  sx <- if (dx > 0) 1 else -1
  sy <- if (dy > 0) 1 else -1
  tdx <- if (dx != 0) abs(1 / dx) else Inf
  tdy <- if (dy != 0) abs(1 / dy) else Inf
  tmx <- if (dx > 0) (cx + 1 - x0) / dx else if (dx < 0) (x0 - cx) / -dx else Inf
  tmy <- if (dy > 0) (cy + 1 - y0) / dy else if (dy < 0) (y0 - cy) / -dy else Inf
  n <- abs(ex - cx) + abs(ey - cy) + 1
  cells <- matrix(0L, n, 2)
  for (i in seq_len(n)) {
    cells[i, ] <- c(cy + 1, cx + 1)
    if (tmx < tmy) {
      tmx <- tmx + tdx
      cx <- cx + sx
    } else {
      tmy <- tmy + tdy
      cy <- cy + sy
    }
  }
  ok <- cells[, 1] >= 1 & cells[, 1] <= nrow(grid$logodds) &
    cells[, 2] >= 1 & cells[, 2] <= ncol(grid$logodds)
  cells[ok, , drop = FALSE]
}

#' Simulate and integrate range scans
#'
#' `grid_raycast()` returns, for each beam angle (relative to the robot
#' heading), the range to the first cell whose occupancy probability
#' exceeds `threshold`, or `max_range`. `grid_update_scan()` performs the
#' standard inverse sensor model update in log-odds: cells traversed by a
#' beam before the hit receive `l_free`, the hit cell receives `l_occ`;
#' beams at `max_range` only clear space. Log-odds are clamped to
#' `clamp`.
#'
#' @param grid A `robot_occupancy_grid`.
#' @param pose Sensor pose `c(x, y, theta)`.
#' @param angles Beam angles (rad) relative to the heading.
#' @param ranges Measured ranges.
#' @param max_range Maximum sensor range.
#' @param threshold Occupancy threshold for ray casting.
#' @param l_occ,l_free Log-odds increments.
#' @param clamp Log-odds bounds.
#' @return Ranges (`grid_raycast`) or an updated grid.
#' @examples
#' occ <- matrix(FALSE, 20, 20)
#' occ[, 15] <- TRUE
#' truth <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1, occupied = occ)
#' grid_raycast(truth, c(0.5, 1, 0), 0, max_range = 3)
#' @name grid_scans
NULL

#' @rdname grid_scans
#' @export
grid_raycast <- function(grid, pose, angles, max_range, threshold = 0.65) {
  p <- check_pose2(pose)
  occ <- grid_occupied(grid, threshold)
  vapply(angles, function(a) {
    dir <- c(cos(p[3] + a), sin(p[3] + a))
    cells <- grid_traverse(grid, p[1:2], p[1:2] + max_range * dir)
    for (i in seq_len(nrow(cells))) {
      if (occ[cells[i, 1], cells[i, 2]]) {
        # Distance to the entry point of the hit cell along the beam.
        lo <- c(grid$xlim[1] + (cells[i, 2] - 1) * grid$resolution,
                grid$ylim[1] + (cells[i, 1] - 1) * grid$resolution)
        hi <- lo + grid$resolution
        t1 <- (lo - p[1:2]) / dir
        t2 <- (hi - p[1:2]) / dir
        tn <- max(pmin(t1, t2)[is.finite(pmin(t1, t2))], 0)
        return(min(tn, max_range))
      }
    }
    max_range
  }, numeric(1))
}

#' @rdname grid_scans
#' @export
grid_update_scan <- function(grid, pose, angles, ranges, max_range,
                             l_occ = 0.85, l_free = -0.4, clamp = c(-5, 5)) {
  p <- check_pose2(pose)
  ranges <- check_numeric_vector(ranges, length(angles), "ranges")
  L <- grid$logodds
  for (k in seq_along(angles)) {
    dir <- c(cos(p[3] + angles[k]), sin(p[3] + angles[k]))
    hit <- ranges[k] < max_range
    # A hit point usually lies on the face of the occupied cell; nudging it
    # a hair along the beam assigns it to that cell for either direction.
    reach <- min(ranges[k], max_range) + if (hit) 1e-6 * grid$resolution else 0
    end <- p[1:2] + reach * dir
    cells <- grid_traverse(grid, p[1:2], end)
    if (!nrow(cells)) next
    hit_cell <- if (hit) grid_world_to_cell(grid, end) else matrix(NA, 1, 2)
    if (hit && !anyNA(hit_cell)) {
      free <- cells[!(cells[, 1] == hit_cell[1] & cells[, 2] == hit_cell[2]), ,
                    drop = FALSE]
      L[hit_cell] <- L[hit_cell] + l_occ
    } else {
      # No hit, or a hit outside the map: every traversed cell is free.
      free <- cells
    }
    if (nrow(free)) L[free] <- L[free] + l_free
  }
  grid$logodds <- pmin(pmax(L, clamp[1]), clamp[2])
  grid
}

#' Inflate obstacles
#'
#' Marks every cell whose center lies closer than `radius` to an occupied
#' cell (treated as a solid square, not a point) as occupied, so a
#' disc-shaped robot of that radius placed at any remaining free cell
#' center does not overlap an occupied cell. Plan the robot as a point on
#' the result.
#'
#' @param grid A `robot_occupancy_grid`.
#' @param radius Inflation radius (m).
#' @param threshold Occupancy threshold.
#' @return A logical matrix of inflated occupancy.
#' @examples
#' occ <- matrix(FALSE, 10, 10); occ[5, 5] <- TRUE
#' g <- robot_occupancy_grid(c(0, 1), c(0, 1), 0.1, occupied = occ)
#' sum(grid_inflate(g, 0.15))
#' @export
grid_inflate <- function(grid, radius, threshold = 0.65) {
  occ <- grid_occupied(grid, threshold)
  r <- ceiling(radius / grid$resolution + 0.5)
  out <- occ
  idx <- which(occ, arr.ind = TRUE)
  offs <- expand.grid(di = -r:r, dj = -r:r)
  # Distance from a cell center to the nearest point of an occupied cell
  # (a square), so a disc of `radius` at a free cell center never overlaps
  # an occupied cell.
  gap <- grid$resolution * sqrt(pmax(abs(offs$di) - 0.5, 0)^2 +
                                  pmax(abs(offs$dj) - 0.5, 0)^2)
  offs <- offs[gap < radius - 1e-12 * grid$resolution, ]
  for (k in seq_len(nrow(idx))) {
    ii <- idx[k, 1] + offs$di
    jj <- idx[k, 2] + offs$dj
    ok <- ii >= 1 & ii <= nrow(occ) & jj >= 1 & jj <= ncol(occ)
    out[cbind(ii[ok], jj[ok])] <- TRUE
  }
  out
}

#' A* search on an occupancy grid
#'
#' Finds a shortest 8-connected path between two world points through
#' free cells, with Euclidean step costs and the Euclidean distance as
#' an admissible heuristic. Diagonal moves are not allowed to cut corners
#' of occupied cells.
#'
#' @param grid A `robot_occupancy_grid`.
#' @param start,goal World points.
#' @param occupied Optional logical occupancy matrix (for example from
#'   [grid_inflate()]); defaults to [grid_occupied()].
#' @return A matrix of world waypoints (cell centers), or `NULL` when the
#'   goal is unreachable on the grid.
#' @examples
#' occ <- matrix(FALSE, 20, 20); occ[3:18, 10] <- TRUE
#' g <- robot_occupancy_grid(c(0, 2), c(0, 2), 0.1, occupied = occ)
#' path <- grid_astar(g, c(0.25, 1), c(1.75, 1))
#' nrow(path)
#' @export
grid_astar <- function(grid, start, goal, occupied = NULL) {
  occ <- occupied %||% grid_occupied(grid)
  s <- grid_world_to_cell(grid, start)
  g <- grid_world_to_cell(grid, goal)
  if (anyNA(s) || anyNA(g)) stop("Start or goal is outside the map.", call. = FALSE)
  if (occ[s] || occ[g]) return(NULL)
  nr <- nrow(occ)
  nc <- ncol(occ)
  id <- function(r, c) (c - 1) * nr + r
  gscore <- rep(Inf, nr * nc)
  came <- integer(nr * nc)
  closed <- logical(nr * nc)
  open <- logical(nr * nc)
  h <- function(r, c) sqrt((r - g[1])^2 + (c - g[2])^2)
  si <- id(s[1], s[2])
  gi <- id(g[1], g[2])
  gscore[si] <- 0
  fscore <- rep(Inf, nr * nc)
  fscore[si] <- h(s[1], s[2])
  open[si] <- TRUE
  moves <- rbind(c(1, 0), c(-1, 0), c(0, 1), c(0, -1), c(1, 1), c(1, -1),
                 c(-1, 1), c(-1, -1))
  while (any(open)) {
    cand <- which(open)
    cur <- cand[which.min(fscore[cand])]
    if (cur == gi) break
    open[cur] <- FALSE
    closed[cur] <- TRUE
    r <- (cur - 1) %% nr + 1
    c <- (cur - 1) %/% nr + 1
    for (m in seq_len(nrow(moves))) {
      rr <- r + moves[m, 1]
      cc <- c + moves[m, 2]
      if (rr < 1 || rr > nr || cc < 1 || cc > nc || occ[rr, cc]) next
      if (m > 4 && (occ[r, cc] || occ[rr, c])) next
      ni <- id(rr, cc)
      if (closed[ni]) next
      tentative <- gscore[cur] + sqrt(sum(moves[m, ]^2))
      if (tentative < gscore[ni]) {
        came[ni] <- cur
        gscore[ni] <- tentative
        fscore[ni] <- tentative + h(rr, cc)
        open[ni] <- TRUE
      }
    }
  }
  if (!is.finite(gscore[gi])) return(NULL)
  path <- gi
  while (path[1] != si) path <- c(came[path[1]], path)
  cells <- cbind((path - 1) %% nr + 1, (path - 1) %/% nr + 1)
  grid_cell_to_world(grid, cells)
}

#' @export
plot.robot_occupancy_grid <- function(x, ...) {
  P <- grid_probability(x)
  xs <- seq(x$xlim[1] + x$resolution / 2, by = x$resolution, length.out = ncol(P))
  ys <- seq(x$ylim[1] + x$resolution / 2, by = x$resolution, length.out = nrow(P))
  graphics::image(xs, ys, t(P), col = grDevices::gray.colors(64, 1, 0),
                  zlim = c(0, 1), asp = 1, xlab = "x (m)", ylab = "y (m)",
                  useRaster = TRUE, ...)
  invisible(x)
}
