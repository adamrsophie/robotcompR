# Motion planners

Configures a sampling-based planner in joint space. Validity of a
configuration means: inside `bounds` (the joint limits by default) and,
when a robot is given, free of collisions with `environment` and of
self-collisions. A custom `validity` function `function(q)` returning
`TRUE` for valid states can replace or refine this, which also allows
planning for abstract point robots without a `robot_model`.

## Usage

``` r
robot_planner(
  robot = NULL,
  algorithm = c("rrt_connect", "rrt", "rrt_star", "prm"),
  environment = NULL,
  validity = NULL,
  bounds = NULL,
  self = TRUE,
  margin = 0,
  step_size = 0.3,
  goal_bias = 0.05,
  max_iterations = 5000,
  resolution = 0.02,
  rewire_factor = 2,
  prm_nodes = 200,
  prm_neighbors = 8,
  time_limit = Inf
)
```

## Arguments

- robot:

  Optional `robot_model`.

- algorithm:

  Planner name.

- environment:

  Optional `robot_environment`.

- validity:

  Optional function of a configuration.

- bounds:

  Matrix of `c(lower, upper)` rows; defaults to the robot's sampling
  bounds (see
  [`robot_random_configuration()`](https://adamrsophie.github.io/robotcompR/reference/robot_configurations.md)).

- self:

  Check self-collision.

- margin:

  Collision margin.

- step_size:

  Maximum extension per tree step.

- goal_bias:

  Probability of sampling the goal (RRT and RRT\*).

- max_iterations:

  Maximum number of samples.

- resolution:

  Edge checking resolution.

- rewire_factor:

  Scale \\\gamma\\ of the RRT\* radius \\\min(\gamma (\log n / n)^{1/d},
  \text{step\\size})\\.

- prm_nodes, prm_neighbors:

  Roadmap size and connectivity.

- time_limit:

  Wall-clock budget in seconds for each call to
  [`robot_plan()`](https://adamrsophie.github.io/robotcompR/reference/robot_plan.md),
  covering start and goal validation, planning and optional smoothing.
  The deadline is checked between configuration checks, including inside
  edge checks, so planning stops with `success = FALSE` and
  `termination = "time_limit"` once it is spent. A single call of
  `validity` cannot be interrupted, so the overrun is at most one such
  call (plus one distance matrix for PRM).

## Value

A `robot_planner`.

## Details

Edges between states are straight lines in joint space, checked at
samples no farther apart than `resolution` in any coordinate. This is a
discrete check (see
[`robot_check_path()`](https://adamrsophie.github.io/robotcompR/reference/robot_check_path.md)).
Distances are Euclidean in joint space; continuous joints are treated as
bounded to \\\[-\pi, \pi\]\\ without wrap-around.

Algorithms:

- `"rrt"`: rapidly exploring random tree (LaValle, 1998) with goal bias.

- `"rrt_connect"`: bidirectional RRT with greedy connection (Kuffner and
  LaValle, 2000).

- `"rrt_star"`: RRT\* (Karaman and Frazzoli, 2011) with rewiring in a
  shrinking radius. It is asymptotically optimal only as the number of
  samples grows without bound; with a finite `max_iterations` it returns
  the best path found so far.

- `"prm"`: probabilistic roadmap with `prm_nodes` samples and
  `prm_neighbors` nearest-neighbor connections, queried with Dijkstra's
  algorithm.

## Examples

``` r
planner <- robot_planner(validity = function(q) sum((q - c(0.5, 0.5))^2) > 0.04,
                         bounds = rbind(c(0, 1), c(0, 1)),
                         algorithm = "rrt_connect")
plan <- robot_plan(planner, c(0.1, 0.1), c(0.9, 0.9), seed = 1)
plan$success
#> [1] TRUE
```
