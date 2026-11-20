# Plan a path between two configurations

Runs the configured planner from `start` to `goal`. Start and goal are
validated first. Randomness is reproducible: with `seed` the global
random number state is restored afterwards and the seed is recorded in
the result. A failure (`success = FALSE`) only means that no path was
found within the iteration budget; it does not prove that no path
exists. A successful path is collision-free only up to the discrete
edge-checking resolution; verify it with
`robot_check_path(..., method = "continuous")` when that matters.

## Usage

``` r
robot_plan(planner, start, goal, seed = NULL, smooth = FALSE)
```

## Arguments

- planner:

  A `robot_planner`.

- start, goal:

  Configurations.

- seed:

  Optional integer seed.

- smooth:

  Apply
  [`robot_shortcut()`](https://adamrsophie.github.io/robotcompR/reference/robot_shortcut.md)
  to a successful path.

## Value

A `robot_plan` with `path` (one waypoint per row, or `NULL`), `success`,
`termination` (one of `"success"`, `"invalid_start"`, `"invalid_goal"`,
`"iteration_limit"` or `"time_limit"`; the reason work stopped, kept
apart from `success`: RRT\* stopped by the deadline returns its best
valid path with `success = TRUE` and `termination = "time_limit"`),
`message`, `algorithm`, `iterations`, `nodes`, `length`, `seed` and
`time` (elapsed seconds, including smoothing). When smoothing is cut
short by the time limit the path found is kept (it is valid), `success`
stays `TRUE` and the message says so.

## Examples

``` r
planner <- robot_planner(validity = function(q) abs(q[1] - 0.5) > 0.1 || q[2] > 0.8,
                         bounds = rbind(c(0, 1), c(0, 1)))
plan <- robot_plan(planner, c(0.1, 0.1), c(0.9, 0.1), seed = 2)
plan
#> <robot_plan> rrt_connect: success
#> path found 
#> 11 waypoints, length 2.315, 37 nodes, 81 iterations, 0.009 s
#> seed: 2 
```
