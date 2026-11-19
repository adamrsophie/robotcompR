# Collision checking along a path

Checks the straight joint-space segments between consecutive waypoints.

## Usage

``` r
robot_check_path(
  robot,
  path,
  environment = NULL,
  self = TRUE,
  resolution = 0.02,
  margin = 0,
  check_limits = TRUE,
  method = c("discrete", "continuous"),
  tol = 1e-09,
  max_steps = 10000
)
```

## Arguments

- robot:

  A `robot_model` with collision geometry.

- path:

  Matrix of waypoints (one per row) or a `robot_plan`.

- environment:

  Optional `robot_environment`.

- self:

  Check self-collision.

- resolution:

  Maximum joint step between checked samples (discrete method).

- margin:

  Safety margin: pairs closer than `margin` count as colliding.

- check_limits:

  Also require samples to respect joint limits.

- method:

  `"discrete"` or `"continuous"`.

- tol:

  Clearance treated as contact by the continuous method.

- max_steps:

  Maximum advancement steps per segment (continuous method); reaching it
  reports the segment as invalid with a warning.

## Value

A list with `valid`, `first_invalid` (the index of the first invalid
sample for the discrete method, or of the first invalid segment for the
continuous method; `NA` when valid), the invalid configuration `q`, and
`samples`, the number of configurations checked.

## Details

With `method = "discrete"` (the default) each segment is sampled so that
no joint moves more than `resolution` between samples (radians or
meters). This can miss collisions with obstacles thinner than the swept
motion between samples.

With `method = "continuous"` each segment is checked by conservative
advancement: at the current configuration the clearance \\d\\ of every
geometry pair is computed, together with an upper bound \\M\\ on how far
any point of the pair can move per unit of the segment parameter (from
the joint displacements and fixed bounds on the lever arms of the
kinematic chain). Advancing by \\d / M\\ cannot create a contact, so the
motion is collision-free when the steps reach the end of the segment and
in collision as soon as a clearance drops to `margin`. Both ends of
every segment are checked with the same contact rule as
[`robot_check_collision()`](https://adamrsophie.github.io/robotcompR/reference/robot_check_collision.md),
so a path ending in contact is invalid. The lever-arm bounds use the
prismatic extensions actually reached on each segment, so they hold with
`check_limits = FALSE` and for joints without finite limits. The test is
exact for the continuous straight-line motion, up to `tol`.

## Examples

``` r
robot <- robot_capsule_links(robot_example("planar2"), 0.05)
env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4),
                                        origin = transl(1.5, 0, 0)))
robot_check_path(robot, rbind(c(pi / 2, 0), c(-pi / 2, 0)), env)$valid
#> [1] FALSE
```
