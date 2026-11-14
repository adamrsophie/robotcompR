# Distance between two geometric primitives

Computes the separation distance and closest points between two convex
primitives placed in a common frame. Each primitive is a convex core
plus a radius: GJK (Gilbert, Johnson and Keerthi, 1988) computes the
distance between the cores, and the radii are subtracted. The result is
exact (up to the GJK tolerance) for separated shapes. When shapes with a
radius overlap but their cores do not, the negative distance is the
exact penetration depth. When the cores themselves overlap, the
expanding polytope algorithm (EPA; van den Bergen, 2001) computes the
penetration depth of the cores, and the negative distance is that depth
plus both radii: the length of the shortest translation that separates
the shapes. For curved cores (cylinders) EPA converges to a relative
tolerance of about \\10^{-9}\\ of the object size. `depth_known` is
`TRUE` for every result and kept for compatibility. Meshes are not
supported.

## Usage

``` r
geom_distance(a, b, pose_a = robot_pose(), pose_b = robot_pose())
```

## Arguments

- a, b:

  `robot_geometry` objects.

- pose_a, pose_b:

  Poses of the frames the geometries are attached to.

## Value

A list with `distance`, `collision`, `depth_known` and the closest
points `point_a` and `point_b`.

## Examples

``` r
geom_distance(geom_sphere(0.5), geom_box(c(1, 1, 1)),
              transl(2, 0, 0), robot_pose())$distance
#> [1] 1
```
