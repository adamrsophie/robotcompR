# Manipulability and singularity analysis.

jacobian_rows <- function(J, measure) {
  switch(measure,
         translational = J[1:3, , drop = FALSE],
         rotational = J[4:6, , drop = FALSE],
         full = J)
}

#' Manipulability and singularity analysis
#'
#' Computes SVD-based diagnostics of the world-frame Jacobian (or a block
#' of it). Translational and rotational blocks have consistent units;
#' the `"full"` 6-row Jacobian mixes meters and radians, so measures
#' computed from it depend on the length unit and should be interpreted
#' only with an explicit scaling convention. The Yoshikawa measure is
#' \eqn{w = \sqrt{\det(J J^\top)} = \prod_i \sigma_i}, using the
#' \eqn{\min(m, n)} singular values of the selected block, and is zero at
#' singular configurations. The condition number is
#' \eqn{\sigma_{max} / \sigma_{min}} (`Inf` when the rank is deficient). The rank is
#' the number of singular values above `tol * sigma_max`.
#'
#' Rows that are identically zero for the robot (for example out-of-plane
#' rows of a planar arm) can be removed with `rows`.
#'
#' @inheritParams robot_jacobian
#' @param measure `"translational"`, `"rotational"`, or `"full"`.
#' @param rows Optional integer vector of Jacobian rows to keep instead of
#'   `measure`.
#' @param tol Relative tolerance for rank and singularity decisions.
#' @return A `robot_manipulability` list with `singular_values`, `rank`,
#'   `yoshikawa`, `condition`, `inverse_condition`, `singular` (logical),
#'   `ellipsoid` (a list with `axes`, the left singular vectors, and
#'   `radii`, the singular values) and `rows`.
#' @examples
#' robot <- robot_example("planar2")
#' robot_manipulability(robot, c(0.3, 0), rows = 1:2)
#' robot_manipulability(robot, c(0.3, 1), rows = 1:2)$yoshikawa
#' @export
robot_manipulability <- function(robot, q, frame = NULL,
                                 measure = c("translational", "rotational",
                                             "full"),
                                 rows = NULL, tol = 1e-8) {
  measure <- match.arg(measure)
  J <- robot_jacobian(robot, q, frame)
  Js <- if (is.null(rows)) jacobian_rows(J, measure) else J[rows, , drop = FALSE]
  s <- svd(Js)
  sv <- s$d
  smax <- max(sv)
  rank <- sum(sv > tol * max(smax, .Machine$double.eps))
  full_rank <- min(dim(Js))
  structure(list(
    singular_values = sv,
    rank = rank,
    yoshikawa = prod(sv),
    condition = if (rank < full_rank) Inf else smax / min(sv),
    inverse_condition = if (smax > 0) min(sv) / smax else 0,
    singular = rank < full_rank,
    ellipsoid = list(axes = s$u, radii = sv),
    rows = rownames(Js), measure = if (is.null(rows)) measure else "custom"
  ), class = "robot_manipulability")
}

#' @export
print.robot_manipulability <- function(x, ...) {
  cat(sprintf("<robot_manipulability> %s rows (%s)\n", x$measure,
              paste(x$rows, collapse = ", ")))
  cat(sprintf("singular values: %s\n",
              paste(signif(x$singular_values, 5), collapse = ", ")))
  cat(sprintf("rank %d, Yoshikawa %.6g, condition %.6g%s\n", x$rank,
              x$yoshikawa, x$condition,
              if (x$singular) ", SINGULAR" else ""))
  invisible(x)
}

#' Detect singular configurations
#'
#' Returns `TRUE` when the selected Jacobian block loses rank within
#' `tol`, with the manipulability diagnostics attached as an attribute.
#' A warning is issued when `warn = TRUE`.
#'
#' @inheritParams robot_manipulability
#' @param warn Issue a warning at singular configurations.
#' @return Logical scalar.
#' @examples
#' robot <- robot_example("planar2")
#' robot_is_singular(robot, c(0.4, 0), rows = 1:2)
#' @export
robot_is_singular <- function(robot, q, frame = NULL,
                              measure = c("translational", "rotational",
                                          "full"),
                              rows = NULL, tol = 1e-8, warn = FALSE) {
  m <- robot_manipulability(robot, q, frame, match.arg(measure), rows, tol)
  if (warn && m$singular) {
    warning(sprintf("Configuration is singular: rank %d of %d.", m$rank,
                    length(m$singular_values)), call. = FALSE)
  }
  structure(m$singular, manipulability = m)
}
