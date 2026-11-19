# Geometric primitives for collision checking and visualization.
#
# Every convex primitive is represented as a convex core plus a radius
# (a "swept sphere"): spheres are a point core, capsules a segment core,
# boxes, cylinders and convex hulls have radius zero. Distances are computed
# between cores with GJK and the radii are subtracted.

new_geometry <- function(type, params, origin = robot_pose()) {
  structure(list(type = type, params = params,
                 origin = as_pose3(origin, "origin")$matrix),
            class = "robot_geometry")
}

#' Collision and visual geometry
#'
#' Constructors for geometric primitives. Each is defined in its own frame,
#' placed by `origin` relative to the frame it is attached to (a link or
#' the world):
#'
#' * `geom_sphere(radius)`: centered at the origin.
#' * `geom_box(size)`: an axis-aligned box in its own frame with edge
#'   lengths `size`, centered at the origin. A rotated `origin` gives an
#'   oriented box.
#' * `geom_cylinder(radius, length)`: axis along z, centered at the origin.
#' * `geom_capsule(radius, length)`: a segment of length `length` along z,
#'   centered at the origin, swept by a sphere of `radius`.
#' * `geom_convex(vertices)`: the convex hull of the rows of `vertices`.
#' * `geom_mesh(filename, scale)`: a reference to a mesh file, kept as
#'   metadata only. Mesh files are never opened, and meshes are ignored by
#'   collision checking.
#'
#' @param radius,length Dimensions in meters.
#' @param size Box edge lengths `c(x, y, z)`.
#' @param vertices Matrix with one 3D vertex per row.
#' @param filename Mesh file name (not read).
#' @param scale Mesh scale factors.
#' @param origin Pose of the geometry frame.
#' @return A `robot_geometry` object.
#' @examples
#' geom_box(c(0.2, 0.1, 0.1), origin = transl(0.5, 0, 0))
#' geom_capsule(0.05, 0.4)
#' @name geometry
NULL

#' @rdname geometry
#' @export
geom_sphere <- function(radius, origin = robot_pose()) {
  new_geometry("sphere", list(radius = check_scalar(radius, "radius",
                                                    positive = TRUE)), origin)
}

#' @rdname geometry
#' @export
geom_box <- function(size, origin = robot_pose()) {
  s <- check_numeric_vector(size, 3, "size")
  if (any(s <= 0)) stop("Box edge lengths must be positive.", call. = FALSE)
  new_geometry("box", list(size = s), origin)
}

#' @rdname geometry
#' @export
geom_cylinder <- function(radius, length, origin = robot_pose()) {
  new_geometry("cylinder",
               list(radius = check_scalar(radius, "radius", positive = TRUE),
                    length = check_scalar(length, "length", positive = TRUE)),
               origin)
}

#' @rdname geometry
#' @export
geom_capsule <- function(radius, length, origin = robot_pose()) {
  new_geometry("capsule",
               list(radius = check_scalar(radius, "radius", positive = TRUE),
                    length = check_scalar(length, "length", nonnegative = TRUE)),
               origin)
}

#' @rdname geometry
#' @export
geom_convex <- function(vertices, origin = robot_pose()) {
  V <- check_matrix(vertices, NULL, 3, "vertices")
  if (nrow(V) < 1) stop("`vertices` needs at least one row.", call. = FALSE)
  new_geometry("convex", list(vertices = V), origin)
}

#' @rdname geometry
#' @export
geom_mesh <- function(filename, scale = c(1, 1, 1), origin = robot_pose()) {
  new_geometry("mesh", list(filename = check_string(filename, "filename"),
                            scale = check_numeric_vector(scale, 3, "scale")),
               origin)
}

#' @export
print.robot_geometry <- function(x, ...) {
  desc <- switch(x$type,
    sphere = sprintf("radius %g", x$params$radius),
    box = sprintf("size (%s)", paste(x$params$size, collapse = ", ")),
    cylinder = sprintf("radius %g, length %g", x$params$radius,
                       x$params$length),
    capsule = sprintf("radius %g, length %g", x$params$radius,
                      x$params$length),
    convex = sprintf("%d vertices", nrow(x$params$vertices)),
    mesh = sprintf("file '%s' (not collidable)", x$params$filename))
  cat(sprintf("<robot_geometry> %s: %s, at (%s)\n", x$type, desc,
              paste(signif(x$origin[1:3, 4], 4), collapse = ", ")))
  invisible(x)
}

geometry_radius <- function(g) {
  switch(g$type, sphere = g$params$radius, capsule = g$params$radius, 0)
}

# Support point of the core of `g` posed at world transform `T` in world
# direction `d`.
geometry_support <- function(g, T, d) {
  R <- T[1:3, 1:3]
  dl <- as.numeric(crossprod(R, d))
  pl <- switch(g$type,
    sphere = c(0, 0, 0),
    capsule = c(0, 0, if (dl[3] >= 0) g$params$length / 2
                      else -g$params$length / 2),
    box = sign_nonzero(dl) * g$params$size / 2,
    cylinder = {
      # Scale before the norm so the support does not depend on the
      # magnitude of the search direction.
      m <- max(abs(dl[1:2]))
      xy <- if (m > 0) {
        u <- dl[1:2] / m
        g$params$radius * u / sqrt(sum(u * u))
      } else c(0, 0)
      c(xy, if (dl[3] >= 0) g$params$length / 2 else -g$params$length / 2)
    },
    convex = {
      V <- g$params$vertices
      V[which.max(V %*% dl), ]
    })
  as.numeric(R %*% pl + T[1:3, 4])
}

sign_nonzero <- function(x) ifelse(x >= 0, 1, -1)

collidable <- function(g) g$type != "mesh"
