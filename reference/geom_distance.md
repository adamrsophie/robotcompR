# Distance between two geometric primitives

Computes the separation distance and closest points between two convex
primitives placed in a common frame. Each primitive is a convex core
plus a radius: GJK (Gilbert, Johnson and Keerthi, 1988) computes the
distance between the cores, and the radii are subtracted. The result is
accurate to the GJK tolerance for separated shapes. When shapes with a
radius overlap but their cores do not, the negative distance is the
exact penetration depth. When the cores themselves overlap, the
expanding polytope algorithm (EPA; van den Bergen, 2001) computes the
penetration depth of the cores, and the negative distance is that depth
plus both radii: the length of the shortest translation that separates
the shapes. Separation is reported only when it is proved: GJK's
distance is an upper bound, and a support plane gives a lower bound that
must be positive. For extremely thin or high aspect ratio cores the
lower bound can fail to be positive although the candidate distance is;
if EPA then proves overlap its depth is used, and otherwise the pair is
reported as colliding with `depth_known = FALSE` and an interval from
support-function bounds. EPA stops when its lower and upper bounds on
the depth agree to a relative tolerance of about \\10^{-9}\\ of the
object size, and `depth_known` is then `TRUE`. Curved cores in symmetric
configurations (for example two identical coaxial cylinders) can exhaust
the iteration budget first: `depth_known` is then `FALSE`, `distance` is
the conservative (shallowest) estimate and `distance_bounds` brackets
the true signed distance. A pair is never reported as separated unless
separation is proved; unresolved contact counts as a collision. When the
cores intersect, `point_a` and `point_b` are the extreme points of each
shape along the separating direction; when only the radii overlap they
are the deepest points of each shape. Meshes are not supported.

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
length-2 interval containing the exact signed distance up to floating
point roundoff; for separated shapes it runs from the certified lower
bound to `distance`) and the closest points `point_a` and `point_b`.

## Examples

``` r
geom_distance(geom_sphere(0.5), geom_box(c(1, 1, 1)),
              transl(2, 0, 0), robot_pose())$distance
#> [1] 1
```
