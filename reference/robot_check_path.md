# Collision checking along a path

Checks the straight joint-space segments between consecutive waypoints
by sampling them so that no joint moves more than `resolution` between
samples (radians or meters). This discrete check can miss collisions
with obstacles thinner than the swept motion between samples; it is not
continuous swept-volume checking.

## Usage

``` r
robot_check_path(
  robot,
  path,
  environment = NULL,
  self = TRUE,
  resolution = 0.02,
  margin = 0,
  check_limits = TRUE
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

  Maximum joint step between checked samples.

- margin:

  Safety margin: pairs closer than `margin` count as colliding.

- check_limits:

  Also require samples to respect joint limits.

## Value

A list with `valid`, the index of the first invalid sample (`NA` when
valid), the invalid configuration, and the number of samples checked.

## Examples

``` r
robot <- robot_capsule_links(robot_example("planar2"), 0.05)
env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4),
                                        origin = transl(1.5, 0, 0)))
robot_check_path(robot, rbind(c(pi / 2, 0), c(-pi / 2, 0)), env)$valid
#> [1] FALSE
```
