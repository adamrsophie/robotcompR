# Internal helpers shared by all modules.

`%||%` <- function(a, b) if (is.null(a)) b else a

check_numeric_vector <- function(x, n = NULL, name = deparse(substitute(x))) {
  if (!is.numeric(x) || anyNA(x) || any(!is.finite(x))) {
    stop(sprintf("`%s` must be a finite numeric vector.", name), call. = FALSE)
  }
  if (!is.null(n) && length(x) != n) {
    stop(sprintf("`%s` must have length %d, not %d.", name, n, length(x)),
         call. = FALSE)
  }
  as.numeric(x)
}

check_scalar <- function(x, name = deparse(substitute(x)), positive = FALSE,
                         nonnegative = FALSE) {
  if (!is.numeric(x) || length(x) != 1L || !is.finite(x)) {
    stop(sprintf("`%s` must be a single finite number.", name), call. = FALSE)
  }
  if (positive && x <= 0) {
    stop(sprintf("`%s` must be positive.", name), call. = FALSE)
  }
  if (nonnegative && x < 0) {
    stop(sprintf("`%s` must be nonnegative.", name), call. = FALSE)
  }
  as.numeric(x)
}

check_count <- function(x, name = deparse(substitute(x)), min = 1) {
  if (!is.numeric(x) || length(x) != 1L || !is.finite(x) || x != round(x) ||
      x < min) {
    stop(sprintf("`%s` must be a whole number of at least %d.", name, min),
         call. = FALSE)
  }
  as.integer(x)
}

check_matrix <- function(x, nrow = NULL, ncol = NULL,
                         name = deparse(substitute(x))) {
  if (!is.matrix(x) || !is.numeric(x) || anyNA(x) || any(!is.finite(x))) {
    stop(sprintf("`%s` must be a finite numeric matrix.", name), call. = FALSE)
  }
  if (!is.null(nrow) && nrow(x) != nrow) {
    stop(sprintf("`%s` must have %d rows.", name, nrow), call. = FALSE)
  }
  if (!is.null(ncol) && ncol(x) != ncol) {
    stop(sprintf("`%s` must have %d columns.", name, ncol), call. = FALSE)
  }
  storage.mode(x) <- "double"
  x
}

check_string <- function(x, name = deparse(substitute(x))) {
  if (!is.character(x) || length(x) != 1L || is.na(x) || !nzchar(x)) {
    stop(sprintf("`%s` must be a single non-empty string.", name),
         call. = FALSE)
  }
  x
}

# Scaling by the largest component first keeps the norm from overflowing
# or underflowing; only an exactly zero vector is rejected.
unit_vector <- function(v, name = "axis") {
  v <- check_numeric_vector(v, name = name)
  m <- max(abs(v))
  if (!(m > 0)) {
    stop(sprintf("`%s` must be a nonzero vector.", name), call. = FALSE)
  }
  u <- v / m
  u / sqrt(sum(u * u))
}

vnorm <- function(v) sqrt(sum(v * v))

cross3 <- function(a, b) {
  c(a[2] * b[3] - a[3] * b[2],
    a[3] * b[1] - a[1] * b[3],
    a[1] * b[2] - a[2] * b[1])
}

# Wrap angles to the half-open interval [-pi, pi). Values already in the
# interval are returned unchanged; very large ones are reduced through
# sin and cos, whose argument reduction is exact, instead of subtracting a
# huge multiple of 2 pi.
wrap_angle_internal <- function(theta) {
  out <- theta - 2 * pi * floor((theta + pi) / (2 * pi))
  keep <- which(theta >= -pi & theta < pi)
  out[keep] <- theta[keep]
  big <- which(abs(theta) > 1e6)
  out[big] <- atan2(sin(theta[big]), cos(theta[big]))
  out[which(out >= pi)] <- -pi
  out[which(out < -pi)] <- -pi
  out
}

# Run `code` with a temporary RNG seed while restoring the caller's state.
with_seed_internal <- function(seed, code) {
  if (is.null(seed)) {
    return(code)
  }
  seed <- check_count(seed, "seed", min = 0)
  has_seed <- exists(".Random.seed", envir = globalenv(), inherits = FALSE)
  if (has_seed) {
    old <- get(".Random.seed", envir = globalenv(), inherits = FALSE)
    on.exit(assign(".Random.seed", old, envir = globalenv()), add = TRUE)
  } else {
    on.exit(rm(".Random.seed", envir = globalenv()), add = TRUE)
  }
  set.seed(seed)
  code
}

#' Wrap angles to the interval from minus pi to pi
#'
#' Maps angles in radians to the half-open interval \eqn{[-\pi, \pi)}.
#'
#' @param theta Numeric vector of angles in radians.
#' @return Numeric vector of wrapped angles.
#' @examples
#' wrap_angle(c(0, 3 * pi / 2, -3 * pi / 2, 7))
#' @export
wrap_angle <- function(theta) {
  if (!is.numeric(theta)) stop("`theta` must be numeric.", call. = FALSE)
  wrap_angle_internal(theta)
}

#' Convert between degrees and radians
#'
#' All angles in robotcompR are radians. These helpers make conversions
#' explicit; no function in the package interprets degrees silently.
#'
#' @param x Numeric vector.
#' @return Numeric vector.
#' @examples
#' deg2rad(180)
#' rad2deg(pi / 2)
#' @name angle_units
NULL

#' @rdname angle_units
#' @export
deg2rad <- function(x) x * pi / 180

#' @rdname angle_units
#' @export
rad2deg <- function(x) x * 180 / pi

# A physical constraint given as one value for all items or exactly one
# value per item; other lengths are an error, never recycled.
check_per_item <- function(x, n, name) {
  x <- check_numeric_vector(x, name = name)
  if (length(x) == 1L) return(rep(x, n))
  if (length(x) != n) {
    stop(sprintf("`%s` must have length 1 or %d, not %d.", name, n, length(x)),
         call. = FALSE)
  }
  x
}

check_probability <- function(x, name) {
  x <- check_scalar(x, name)
  if (x < 0 || x > 1) {
    stop(sprintf("`%s` must be a probability in [0, 1].", name), call. = FALSE)
  }
  x
}
