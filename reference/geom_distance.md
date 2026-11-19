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
the shapes. EPA stops when its lower and upper bounds on the depth agree
to a relative tolerance of about \\10^{-9}\\ of the object size, and
`depth_known` is then `TRUE`. Curved cores in symmetric configurations
(for example two identical coaxial cylinders) can exhaust the iteration
budget first: `depth_known` is then `FALSE`, `distance` is the
conservative (shallowest) estimate and `distance_bounds` brackets the
true signed distance. The collision classification is exact either way.
When the cores intersect, `point_a` and `point_b` are the extreme points
of each shape along the separating direction; when only the radii
overlap they are the deepest points of each shape. Meshes are not
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

A list with `distance`, `collision`, `depth_known`, `distance_bounds` (a
length-2 interval containing the exact signed distance; both ends equal
`distance` when it is exact) and the closest points `point_a` and
`point_b`.

## Examples

``` r
geom_distance(geom_sphere(0.5), geom_box(c(1, 1, 1)),
              transl(2, 0, 0), robot_pose())$distance
#> [1] 1
```
