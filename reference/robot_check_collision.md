# Collision checking at a configuration

Checks the robot's collision geometry against the obstacles of an
environment and, with `self = TRUE`, against itself. Self-collision
skips link pairs that no movable joint separates, and pairs separated by
one movable joint when no link between them carries collision geometry
(joint neighbors, possibly through geometry-free fixed frames); a tool
fixed to a link is still checked against the link before that link's
joint. Further link pairs can be excluded with `ignore_pairs`. A cheap
bounding-sphere test prunes pairs that are farther apart than `margin`.
This checks one configuration only: a collision-free start and goal do
not imply that the motion between them is collision-free; use
[`robot_check_path()`](https://adamrsophie.github.io/robotcompR/reference/robot_check_path.md)
for that.

## Usage

``` r
robot_check_collision(
  robot,
  q,
  environment = NULL,
  self = TRUE,
  margin = 0,
  ignore_pairs = NULL,
  distances = FALSE
)
```

## Arguments

- robot:

  A `robot_model` with collision geometry.

- q:

  Configuration.

- environment:

  Optional `robot_environment`.

- self:

  Check self-collision.

- margin:

  Safety margin: pairs closer than `margin` count as colliding.

- ignore_pairs:

  Optional list of `c(link_a, link_b)` name pairs to skip in
  self-collision checking.

- distances:

  Compute exact distances for every pair (slower).

## Value

A `robot_collision_result` with `in_collision`, `min_distance` (over
evaluated pairs, `NA` if none) and a data frame `pairs` of evaluated
pairs.

## Examples

``` r
robot <- robot_capsule_links(robot_example("planar2"), 0.05)
env <- robot_environment(box = geom_box(c(0.4, 0.4, 0.4),
                                        origin = transl(1.5, 0, 0)))
robot_check_collision(robot, c(0, 0), env)$in_collision
#> [1] TRUE
robot_check_collision(robot, c(pi / 2, 0), env)$in_collision
#> [1] FALSE
```
