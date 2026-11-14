# Shortcut smoothing of a planned path

Repeatedly picks two random waypoints and replaces the path between them
with a straight segment when that segment is valid. The path length
never increases. Endpoints are preserved.

## Usage

``` r
robot_shortcut(plan, iterations = 100, seed = NULL)
```

## Arguments

- plan:

  A successful `robot_plan`.

- iterations:

  Number of shortcut attempts.

- seed:

  Optional seed.

## Value

A `robot_plan` with the shortened path.

## Examples

``` r
planner <- robot_planner(validity = function(q) TRUE,
                         bounds = rbind(c(0, 1), c(0, 1)), algorithm = "rrt")
plan <- robot_plan(planner, c(0, 0), c(1, 1), seed = 3)
robot_shortcut(plan, seed = 3)$length <= plan$length
#> [1] TRUE
```
