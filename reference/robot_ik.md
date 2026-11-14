# Numerical inverse kinematics

Solves \\FK(q) \approx T^\*\\ for a target pose (or position) of
`frame`. The error is \\e = (p^\* - p, \log(R^\* R^\top))\\ in world
coordinates, a proper SO(3) orientation error rather than a difference
of Euler angles, weighted componentwise by `weights`. Methods:

## Usage

``` r
robot_ik(
  robot,
  target,
  initial = NULL,
  frame = NULL,
  method = c("levenberg_marquardt", "damped_least_squares", "pseudoinverse"),
  weights = NULL,
  tol = 1e-10,
  max_iterations = 200,
  lambda = 0.01,
  max_step = 0.5,
  restarts = 0,
  respect_limits = TRUE,
  seed = NULL
)
```

## Arguments

- robot:

  A `robot_model`.

- target:

  A `robot_pose` (SE(3)) or a length-3 position. A position target sets
  the orientation weights to zero.

- initial:

  Initial configuration (defaults to
  [`robot_home()`](https://adamrsophie.github.io/robotcompR/reference/robot_configurations.md)).

- frame:

  Frame to place (`NULL` for the end effector).

- method:

  `"levenberg_marquardt"`, `"damped_least_squares"`, or
  `"pseudoinverse"`.

- weights:

  Length-6 nonnegative weights for \\(x, y, z, \omega_x, \omega_y,
  \omega_z)\\ errors. Use zeros for unconstrained directions, for
  example `c(1, 1, 0, 0, 0, 1)` for a planar arm with a pose target.

- tol:

  Convergence tolerance on the weighted error norm.

- max_iterations:

  Maximum iterations per attempt.

- lambda:

  Initial damping.

- max_step:

  Largest change of any joint variable in one iteration (radians or
  meters). Steps are scaled down to this size, which keeps iterations
  started near singular configurations from jumping far.

- restarts:

  Number of random restarts.

- respect_limits:

  Project iterates onto the joint limits.

- seed:

  Optional seed for restarts; the global random state is restored
  afterwards.

## Value

A `robot_ik_solution` list with `q`, `converged`, `residual`,
`position_error`, `orientation_error` (radians), `iterations`,
`attempts`, `active_limits` (names of joints at a limit), `singular`
(rank-deficient weighted Jacobian at the solution), `reach_bound` (an
upper bound on the reachable distance from the base) and
`target_distance`.

## Details

- `"levenberg_marquardt"` (default): \\\Delta q = (J^\top W J + \lambda
  \mu I)^{-1} J^\top W e\\, where \\\mu\\ is the largest diagonal entry
  of \\J^\top W J\\, with \\\lambda\\ decreased after successful steps
  and increased after rejected steps.

- `"damped_least_squares"`: \\\Delta q = J^\top (J J^\top + \lambda^2
  I)^{-1} e\\ with fixed \\\lambda\\, plus backtracking.

- `"pseudoinverse"`: \\\Delta q = J^{+} e\\ from a truncated SVD, plus
  backtracking.

With `respect_limits = TRUE` each iterate is projected onto the joint
limits. When the solve from `initial` fails, up to `restarts` further
attempts start from random configurations within the limits; the best
result is returned. Numerical IK can fail even for reachable targets and
never proves that a target is unreachable.

## Examples

``` r
robot <- robot_example("planar2")
sol <- robot_ik(robot, c(1, 1, 0), initial = c(0.3, 0.3))
sol$q
#>        joint1        joint2 
#> -2.670105e-13  1.570796e+00 
sol$converged
#> [1] TRUE
```
